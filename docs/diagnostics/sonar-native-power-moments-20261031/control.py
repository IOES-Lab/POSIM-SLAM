"""Conditional native-kernel power controls on retained range samples."""
import argparse
import ctypes as ct
import hashlib
import json
from pathlib import Path
import time
import numpy as np
from posim_slam.sonar_source_normals import ray_surface_normals
from posim_slam.sonar_speckle import periodic_hamming_amplitude, power_moments


def moments(depth, normal, mixing, pixels, *, area, alpha, bandwidth=29900., speed=1500., n=599):
    beam, ray = np.nonzero((depth > 0) & (depth <= 15.))
    d = depth[beam, ray].astype(float)
    q = (1e10*np.exp(-4*float(alpha)*d)*d**-3*float(area)
         *abs(normal[beam, ray, 2].astype(float))**2*.001)
    delay = d[None, :]/(speed/(2*bandwidth))-pixels[:, 0, None]
    h = (mixing[pixels[:, 1, None], beam].astype(float)
         *periodic_hamming_amplitude(delay, n)*bandwidth/np.sqrt(n))
    return power_moments(h, q)


def statistics(samples, mean, covariance):
    p = abs(samples)**2
    standardized_mean = float(np.max(abs(p.mean(axis=0)/mean-1))*np.sqrt(len(p)))
    norm = np.outer(mean, mean)
    error = (np.cov(p, rowvar=False)-covariance)/norm
    covariance_error = float(np.linalg.norm(error)/np.linalg.norm(covariance/norm))
    return [standardized_mean, covariance_error]


def null_control(k, protocol):
    mean = k.diagonal().real
    correlation = k/np.sqrt(np.outer(mean, mean))
    values, vectors = np.linalg.eigh((correlation+correlation.conj().T)/2)
    assert values.min() >= -1e-10*values.max()
    factor = vectors*np.sqrt(np.maximum(values, 0))
    rng = np.random.default_rng(protocol['null_seed'])
    stats = []
    frames = len(protocol['frame_ids'])
    for _ in range(protocol['null_replicates']):
        xi = (rng.normal(size=(frames, len(k)))+1j*rng.normal(size=(frames, len(k))))/np.sqrt(2)
        z = xi@factor.T
        stats.append(statistics(z, np.ones(len(k)), abs(correlation)**2))
    return np.asarray(stats)


def acquire(args, protocol):
    native = ct.CDLL(str(args.binary.resolve()), mode=ct.RTLD_GLOBAL)
    digest = hashlib.sha256(args.binary.read_bytes()).hexdigest()
    assert digest == '0090433756cb425eea803ce56a70c1af5adf844223054b2f1c25739bee0d8564'
    ptr = ct.POINTER(ct.c_float)
    compute, free = native.sonar_wgpu_compute, native.sonar_wgpu_free
    compute.argtypes = [ptr]*5+[ct.c_uint32]*4+[ct.c_float]*10+[ct.c_uint64]*2+[ct.c_bool]
    compute.restype = ptr
    free.argtypes = [ptr, ct.c_size_t]
    free.restype = None
    class DlInfo(ct.Structure):
        _fields_ = [('fname', ct.c_char_p), ('base', ct.c_void_p),
                    ('symbol', ct.c_char_p), ('address', ct.c_void_p)]
    info = DlInfo()
    dl = ct.CDLL(None).dladdr
    dl.argtypes = [ct.c_void_p, ct.POINTER(DlInfo)]
    assert dl(ct.cast(compute, ct.c_void_p), ct.byref(info))
    assert Path(info.fname.decode()).resolve() == args.binary.resolve()
    with np.load(args.input, allow_pickle=False) as z:
        depth = np.ascontiguousarray(np.linalg.norm(z['xyz'].astype(float), axis=2).T, dtype='f4')
        directions = z['beam_directions']
    angles = np.arctan2(directions[:, 1], directions[:, 0]).astype(float)
    beams, rays = depth.shape
    assert (beams, rays) == (256, 256)
    hfov, vfov = angles[-1]-angles[0], np.pi/9
    elevation = np.linspace(-vfov/2, vfov/2, rays)
    normal = np.ascontiguousarray(ray_surface_normals(depth.T, angles, elevation, 15.).transpose(1, 0, 2))
    mixing = abs(np.sinc(.884/(hfov/(beams-1))*np.sin(angles[:, None]-angles)))
    mixing = np.ascontiguousarray(mixing/np.linalg.norm(mixing), dtype='f4')
    window = .54-.46*np.cos(2*np.pi*(np.arange(599)+1)/599)
    window = (window/np.linalg.norm(window)).astype('f4')
    alpha = np.float32(.0354*np.log(10)/20)
    area = np.float32(np.float32(hfov)/(beams-1)*(np.float32(vfov)/(rays-1))*2)
    rho = np.full_like(depth, .001)
    pixels = np.asarray(protocol['queries'])
    selected, spots, durations = [], {}, []
    power_sum = np.zeros((beams, 599)); square_sum = np.zeros_like(power_sum)
    pointers = [a.ctypes.data_as(ptr) for a in [depth, normal, rho, mixing, window]]
    for index, frame in enumerate(protocol['frame_ids']):
        t = time.monotonic()
        output = compute(*pointers, beams, rays, 599, 1, 1500., 15., 220., alpha,
                         1., 29900., 1., area, hfov, vfov, frame, protocol['seed'], False)
        assert output
        try:
            raw = np.ctypeslib.as_array(output, shape=(beams*599*2,)).copy().reshape(beams, 599, 2)
        finally:
            free(output, beams*599*2)
        z = raw[..., 0]+1j*raw[..., 1]
        assert np.isfinite(z).all()
        selected.append(z[pixels[:, 1], pixels[:, 0]])
        p = abs(z.astype(np.complex128))**2
        power_sum += p; square_sum += p*p
        if index in (0, 255, 511):
            spots[str(frame)] = z
        durations.append(time.monotonic()-t)
        if index%128 == 0:
            print(json.dumps(dict(frames=index+1, total=len(protocol['frame_ids']))), flush=True)
        time.sleep(max(0., protocol['operator_min_interval_s']-(time.monotonic()-t)))
    np.savez_compressed(args.output/'inputs.npz', depth=depth, normal=normal, mixing=mixing,
                        window=window, angles=angles, elevation=elevation, area=area, alpha=alpha)
    np.savez_compressed(args.output/'native-output.npz', selected=np.asarray(selected),
                        power_sum=power_sum, power_square_sum=square_sum, **spots)
    (args.output/'native-identity.json').write_text(json.dumps(dict(
        binary_sha256=digest, actual_symbol_path=info.fname.decode(), source=protocol['source'],
        input_sha256=hashlib.sha256(args.input.read_bytes()).hexdigest(),
        median_operator_s=float(np.median(durations)), max_operator_s=max(durations)), indent=2)+'\n')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--binary', type=Path)
    parser.add_argument('--input', type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--verify', action='store_true')
    args = parser.parse_args()
    protocol = json.loads((args.output/'protocol.json').read_text())
    if not args.verify:
        assert args.binary and args.input and not (args.output/'native-output.npz').exists()
        acquire(args, protocol)
    with np.load(args.output/'inputs.npz', allow_pickle=False) as z:
        predicted = moments(z['depth'], z['normal'], z['mixing'],
                            np.asarray(protocol['queries']), area=float(z['area']), alpha=float(z['alpha']))
    with np.load(args.output/'native-output.npz', allow_pickle=False) as z:
        observed = statistics(z['selected'], predicted['mean'], predicted['covariance'])
        measured_mean = (abs(z['selected'].astype(np.complex128))**2).mean(axis=0)
    print('Native samples complete; simulating frozen correlated null control', flush=True)
    null = null_control(predicted['complex_covariance'], protocol)
    threshold = np.quantile(null, protocol['null_quantile'], axis=0)
    report = dict(observed_statistics=observed, thresholds=threshold.tolist(),
                  passed=bool(np.all(observed <= threshold)),
                  source=protocol['source'], frames=len(protocol['frame_ids']), pixels=64,
                  scope=protocol['scope'], no_gain_fit=True)
    if not args.verify:
        np.savez_compressed(args.output/'moments.npz', **predicted, measured_mean=measured_mean,
                            null_statistics=null)
        (args.output/'result.json').write_text(json.dumps(report, indent=2)+'\n')
    else:
        original = json.loads((args.output/'result.json').read_text())
        assert original['passed'] == report['passed']
        np.testing.assert_allclose(report['observed_statistics'], original['observed_statistics'], rtol=2e-6)
    print(json.dumps(report), flush=True)
    if not report['passed']:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
