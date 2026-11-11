# SPDX-License-Identifier: Apache-2.0
"""Verify all recorded rays and pixels; no world, SDK or private app imports."""
import argparse
import hashlib
import json
from pathlib import Path
import sys
import numpy as np

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT))
from trace import read_trace
from sampling import cubemap_rays
from posim_slam.sonar_surface_power import TriangleSurface, SurfacePowerModel
from posim_slam.sonar_source_surface import SourceSurfaceObservation
from posim_slam.sonar_speckle import periodic_hamming_amplitude


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def rotation(q):
    w, x, y, z = np.asarray(q)/np.linalg.norm(q)
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def summarize(value):
    value = np.asarray(value)
    return dict(count=int(value.size), rms=float(np.sqrt(np.mean(value**2))),
                absolute_max=float(abs(value).max()), median_absolute=float(np.median(abs(value))))


def mean_image(depth, normal, trace):
    """Exact finite Hamming power sums; one shared gain is profiled later."""
    a, s = trace['arrays'], trace['scalars']
    beams, _, bins, _ = trace['dimensions']
    reflectivity = a['reflectivity'].T
    radial = np.zeros((bins, beams))
    for b in range(beams):
        d = depth[:, b].astype(float)
        keep = np.isfinite(d) & (d > 0) & (d <= s['distance'])
        d = d[keep]
        q = (reflectivity[keep, b].astype(float)*d**-3*np.exp(-4*s['attenuation']*d)
             * normal[keep, b, 2].astype(float)**2)
        h = periodic_hamming_amplitude(d[None, :]/(s['speed']/(2*s['bandwidth']))-
                                      np.arange(bins)[:, None], bins)
        radial[:, b] = abs(h)**2@q
    return radial@(a['mixing'].astype(float)**2).T


def score(power, mean):
    assert np.isfinite(mean).all() and np.all(mean > 0)
    gain = float(np.mean(power/mean))
    residual = power/(gain*mean)-1
    return dict(pixels=power.size, gain=gain, residual_rms=float(np.sqrt(np.mean(residual**2))),
                residual_max=float(abs(residual).max()))


def run(scene_override=None):
    manifest = ROOT/'sha256.json'
    if manifest.exists():
        for name, digest in json.loads(manifest.read_text()).items():
            assert sha(ROOT/name) == digest, name
    data = json.loads((ROOT/'inputs.json').read_text())
    scene_path = scene_override if scene_override is not None else ROOT/data['scene_record']
    assert sha(scene_path) == data['scene_sha256']
    with np.load(scene_path, allow_pickle=False) as z:
        mesh = np.concatenate(list(dict(z).values()))
    with np.load(ROOT/'calibration.npz', allow_pickle=False) as z:
        az, el, sampled = [z[k] for k in ['azimuth', 'elevation', 'sampled_ray_directions']]
    reconstructed, texels = cubemap_rays(np.linspace(-np.pi/3, np.pi/3, 256),
                                        np.linspace(-np.pi/18, np.pi/18, 256), 256)
    np.testing.assert_allclose(sampled.reshape(-1, 3), reconstructed, rtol=0, atol=2e-15)
    cases = []
    for row in data['frames']:
        trace = read_trace(ROOT/row['trace'])
        assert trace['sha256'] == row['trace_sha256']
        assert trace['dimensions'] == (256, 256, 599, 1)
        meta, a = trace['metadata'], trace['arrays']
        assert meta['context_source'] == 'render_node'
        assert meta['capture_stamp_ns'] == meta['render_begin_stamp_ns'] == row['stamp_ns']
        assert meta['render_sensor_world_xyz_wxyz'] == meta['render_begin_sensor_world_xyz_wxyz']
        assert sha(ROOT/row['raw']) == row['raw_sha256']
        with np.load(ROOT/row['raw'], allow_pickle=False) as z:
            raw = dict(z)
        assert int(raw['stamp'][0])*10**9+int(raw['stamp'][1]) == row['stamp_ns']
        x, y = a['output'][..., 0].astype(float), a['output'][..., 1].astype(float)
        db = (20*np.log10(np.hypot(x, y))).T
        # Cross-platform libm arithmetic check; exact native C++ byte equality
        # is separately retained for every pixel in the publication receipt.
        np.testing.assert_allclose(db, raw['intensity_db'], rtol=0, atol=2e-5)
        pose = np.asarray(meta['render_sensor_world_xyz_wxyz'])
        local_mesh = (mesh-pose[:3])@rotation(pose[3:])
        surface = TriangleSurface(local_mesh)
        power = np.power(10., raw['intensity_db'].astype(float)/10.)
        captured = a['depth'].T
        captured_valid = (captured > 0) & (captured <= 15)
        result = dict(index=row['index'], stamp_ns=row['stamp_ns'], rays=65536,
                      raw_conversion_delta_db=summarize(db-raw['intensity_db']), arms={})
        mean = mean_image(captured, a['normal'].transpose(1, 0, 2), trace)
        result['arms']['captured_inputs'] = score(power, mean)
        for name, directions in [('ideal', None), ('sampled', sampled)]:
            model = SurfacePowerModel(az, el, raw['ranges'], maximum_range_m=15.,
                                      sampled_ray_directions=directions)
            observation = SourceSurfaceObservation(model, surface)
            depth = observation.ranges
            valid = (depth > 0) & (depth <= 15)
            both = captured_valid & valid
            mean = mean_image(depth, observation.normals, trace)
            result['arms'][name] = dict(**score(power, mean),
                actual_only=int((captured_valid & ~valid).sum()),
                expected_only=int((~captured_valid & valid).sum()),
                range_residual_m=summarize((captured-depth)[both]))
            if name == 'sampled':
                assert np.array_equal(valid, captured_valid)
        cases.append(result)
        print(json.dumps(dict(index=row['index'],
            range_rms_m={k:v['range_residual_m']['rms'] for k,v in result['arms'].items() if 'range_residual_m' in v},
            power_residual_rms={k:v['residual_rms'] for k,v in result['arms'].items()})), flush=True)
    return dict(assessment_only=True, map_accepted=False, scene_used_for_inference=False,
                complete_frames=4, complete_raw_pixels=4*599*256, complete_ray_cells=4*65536,
                sampled_unique_texels=len(np.unique(texels, axis=0)), results=cases)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--save-new-assessment', type=Path)
    parser.add_argument('--scene', type=Path, help='Same hash-bound scene at a relocated path')
    args = parser.parse_args()
    result = run(args.scene)
    if args.save_new_assessment:
        with args.save_new_assessment.open('x') as f:
            json.dump(result, f, indent=2, allow_nan=False)
            f.write('\n')
    else:
        frozen = json.loads((ROOT/'assessment.json').read_text())
        assert result['complete_raw_pixels'] == frozen['complete_raw_pixels']
        for actual, expected in zip(result['results'], frozen['results'], strict=True):
            for arm in actual['arms']:
                np.testing.assert_allclose(actual['arms'][arm]['residual_rms'],
                    expected['arms'][arm]['residual_rms'], rtol=1e-3, atol=1e-6)
                if 'range_residual_m' in actual['arms'][arm]:
                    np.testing.assert_allclose(actual['arms'][arm]['range_residual_m']['rms'],
                        expected['arms'][arm]['range_residual_m']['rms'], rtol=1e-3, atol=1e-7)
        print('PASS: all four complete renderer inputs, every ray and raw pixel; no map acceptance')


if __name__ == '__main__':
    main()
