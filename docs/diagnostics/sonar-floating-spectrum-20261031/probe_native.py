"""Compare production WGPU with floating/quantized identical-input controls.

The stored renderer geometry is a controlled producer input, never an estimator
observation. The RGB8 normal approximation is supplied identically to each path.
"""
from pathlib import Path
import ctypes as ct
import hashlib
import importlib.util
import json
import numpy as np

import argparse
import time
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--binary', type=Path, required=True, help='Source-bound production multibeam sonar library')
parser.add_argument('--output', type=Path, required=True, help='New directory outside this record')
parser.add_argument('--save-full', action='store_true', help='Retain every complex output of all three controls')
parser.add_argument('--benchmark', type=int, default=0)
parser.add_argument('--benchmark-only', action='store_true')
args = parser.parse_args()
ROOT = Path(__file__).resolve().parent.parent / 'sonar-range-response-20261030'
OUT = args.output.resolve()
assert not OUT.is_relative_to(ROOT), 'Use a new external output directory'
OUT.mkdir(parents=True, exist_ok=False)
spec = importlib.util.spec_from_file_location('fixed_point_control', ROOT / 'fixed_point_reference.py')
control = importlib.util.module_from_spec(spec)
spec.loader.exec_module(control)
binary = args.binary.resolve(strict=True)
digest=hashlib.sha256(binary.read_bytes()).hexdigest()
identities={
 '5d1c6e45ab4110ae8f0dbbe082fb72f7e0fd0e2a4b46111e95ab1586a718913b': ('7afd71e7b7a1a13eca8c4b8deb9c6e8461285cd5','fixed-point'),
 '0090433756cb425eea803ce56a70c1af5adf844223054b2f1c25739bee0d8564': ('ffecee9be9976f68c9e883d6cf3bbd6d40eb7729','floating'),
}
revision,mode=identities[digest]
native = ct.CDLL(str(binary), mode=ct.RTLD_GLOBAL)
ptr = ct.POINTER(ct.c_float)
compute = native.sonar_wgpu_compute
compute.argtypes = [ptr] * 5 + [ct.c_uint32] * 4 + [ct.c_float] * 10 + [ct.c_uint64] * 2 + [ct.c_bool]
compute.restype = ptr
free = native.sonar_wgpu_free
free.argtypes = [ptr, ct.c_size_t]
free.restype = None
class DlInfo(ct.Structure):
    _fields_=[('fname',ct.c_char_p),('base',ct.c_void_p),('symbol',ct.c_char_p),('address',ct.c_void_p)]
info=DlInfo();dl=ct.CDLL(None).dladdr;dl.argtypes=[ct.c_void_p,ct.POINTER(DlInfo)]
assert dl(ct.cast(compute,ct.c_void_p),ct.byref(info))
assert Path(info.fname.decode()).resolve()==binary

with np.load(ROOT / "captures/sonoptix/summary.npz", allow_pickle=False) as data:
    depth = np.ascontiguousarray(np.linalg.norm(data["xyz"].astype(float), axis=2).T, dtype=np.float32)
    normal = np.ascontiguousarray(data["normal_rgb8"].transpose(1, 0, 2).astype(np.float32) / np.float32(255))
    directions = data["beam_directions"].copy()
    ranges = data["ranges"].copy()
beams, rays = depth.shape
n = len(ranges)
angles = np.arctan2(directions[:, 1], directions[:, 0]).astype(float)
hfov, vfov = angles[-1] - angles[0], np.pi / 9
h = hfov / (beams - 1)
mix = abs(np.sinc(.884 / h * np.sin(angles[:, None] - angles[None, :])))
mix = np.ascontiguousarray(mix / np.linalg.norm(mix), dtype=np.float32)
reflectivity = np.full_like(depth, 1e-3)
window = .54 - .46 * np.cos(2 * np.pi * (np.arange(n) + 1) / n)
window = (window / np.linalg.norm(window)).astype(np.float32)
alpha = np.float32(.0354 * np.log(10) / 20)
area = np.float32(np.float32(hfov) / (beams - 1) * (np.float32(vfov) / (rays - 1)) * 2)
frames = [] if args.benchmark_only else [20, 21, 22]
seed = 73
protocol = dict(scope="Frozen renderer-derived geometry; source numerical comparison only",
    source_revision=revision, native_accumulation=mode, actual_symbol_path=info.fname.decode(),
    binary_file=binary.name, binary_sha256=hashlib.sha256(binary.read_bytes()).hexdigest(),
    seed=seed, frames=frames, shape=[beams, rays, n], range_max_m=15.,
    source_level_db=220., attenuation=float(alpha), reflectivity=1e-3,
    area_scaler=float(area), full_resolution=True, selected_tail_m=[10.,15.],
    normal="Captured RGB8/255, supplied identically to all controls",
    changes="Only per-scatterer spectral rounding differs between CPU references",
    installed_sdk_changed=False, renderer_or_live_estimator_restarted=False)
(OUT / "protocol.json").write_text(json.dumps(protocol, indent=2) + "\n")
amplitude_scale = (np.float32(1e5) * np.exp(-np.float32(2) * alpha * depth)
    / (depth * depth) * np.sqrt(reflectivity) * abs(normal[..., 2]) * np.sqrt(depth * area))
tail = ranges >= 10.
result = []
arrays = {}
inputs = [depth, normal, reflectivity, mix, window]
for frame in frames:
    output = compute(*[v.ctypes.data_as(ptr) for v in inputs], beams, rays, n, 1,
        1500., 15., 220., alpha, 1., 29900., 1., area, hfov, vfov, frame, seed, False)
    assert output
    try:
        raw = np.ctypeslib.as_array(output, shape=(beams * n * 2,)).copy().reshape(beams, n, 2)
    finally:
        free(output, beams * n * 2)
    native_complex = raw[..., 0] + 1j * raw[..., 1]
    actual = native_complex.astype(np.complex128)
    x, y = control.philox(seed, frame, np.arange(beams * rays))
    amplitude = (x.reshape(beams, rays) / np.sqrt(np.float32(2))
        + 1j * y.reshape(beams, rays) / np.sqrt(np.float32(2))) * amplitude_scale
    spectrum_float = np.empty((beams, n), dtype=np.complex128)
    spectrum_quant = np.empty_like(spectrum_float)
    for beam in range(beams):
        values = control.source_spectrum(depth[beam], amplitude[beam], n)
        # Int64 sum is a diagnostic reference. Assert actual i32 accumulator
        # bounds; do not silently model overflow as ordinary rounding noise.
        re = np.rint(values.real * 1024).astype(np.int64).sum(axis=0)
        im = np.rint(values.imag * 1024).astype(np.int64).sum(axis=0)
        assert np.max(abs(re)) < 2147483520 and np.max(abs(im)) < 2147483520
        spectrum_quant[beam] = re / 1024. + 1j * im / 1024.
        spectrum_float[beam] = values.astype(np.complex128).sum(axis=0)
    floating = np.fft.fft(mix.astype(float) @ spectrum_float, axis=1) * (29900. / n)
    quantized = np.fft.fft(mix.astype(float) @ spectrum_quant, axis=1) * (29900. / n)
    row = dict(frame=frame, tail_cells=int(beams * tail.sum()),
        native_tail_mean_power=float(np.mean(abs(actual[:, tail]) ** 2)),
        quantized_tail_mean_power=float(np.mean(abs(quantized[:, tail]) ** 2)),
        floating_tail_mean_power=float(np.mean(abs(floating[:, tail]) ** 2)),
        whole_complex_residual_quantized=float(np.linalg.norm(actual-quantized)/np.linalg.norm(actual)),
        whole_complex_residual_floating=float(np.linalg.norm(actual-floating)/np.linalg.norm(actual)),
        tail_complex_residual_quantized=float(np.linalg.norm(actual[:,tail]-quantized[:,tail])/np.linalg.norm(actual[:,tail])),
        tail_complex_residual_floating=float(np.linalg.norm(actual[:,tail]-floating[:,tail])/np.linalg.norm(actual[:,tail])))
    result.append(row)
    for label, value in [("native",native_complex),("quantized",quantized),("floating",floating)]:
        arrays[f"frame{frame}_{label}"] = value
    print(json.dumps(row), flush=True)
timings=[]
for index in range(-3,args.benchmark):
    if not args.benchmark:break
    start=time.perf_counter()
    output=compute(*[v.ctypes.data_as(ptr) for v in inputs],beams,rays,n,1,
        1500.,15.,220.,alpha,1.,29900.,1.,area,hfov,vfov,100+index,seed,False)
    assert output
    try:values=np.ctypeslib.as_array(output,shape=(beams*n*2,)).copy()
    finally:free(output,beams*n*2)
    elapsed=time.perf_counter()-start
    assert np.isfinite(values).all()
    if index>=0:timings.append(elapsed)
benchmark=dict(scope='Warm production operator/readback/copy/free only; renderer/ROS throughput is not qualified',
    warmups=3,samples=len(timings),times_s=timings,
    median_s=None if not timings else float(np.median(timings)),
    p95_s=None if not timings else float(np.quantile(timings,.95)),
    maximum_s=None if not timings else max(timings),
    meets_operator_2hz=None if not timings else max(timings)<.5)
if args.save_full:
    np.savez_compressed(OUT / "full-output.npz", **arrays)
(OUT / "source-comparison.json").write_text(json.dumps(dict(protocol=protocol, rows=result,benchmark=benchmark), indent=2) + "\n")
