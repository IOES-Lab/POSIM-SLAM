"""Verify selected native moments, source normals and all field baseline rows."""
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import numpy as np
from posim_slam.sonar_source_normals import ray_surface_normals
from posim_slam.sonar_power_evidence import profile_power_evidence


def main():
    root = Path(__file__).resolve().parent
    for name, expected in json.loads((root/'sha256.json').read_text()).items():
        assert hashlib.sha256((root/name).read_bytes()).hexdigest() == expected, name
    for name, expected in json.loads((root/'references.json').read_text()).items():
        assert hashlib.sha256((root/name).read_bytes()).hexdigest() == expected, name
    import posim_slam.sonar_power_evidence as module
    package = Path(module.__file__).resolve().parent
    for name, expected in json.loads((root/'component-source.json').read_text()).items():
        assert hashlib.sha256((package/name).read_bytes()).hexdigest() == expected, name
    protocol = json.loads((root/'protocol.json').read_text())
    with np.load(root/'inputs.npz', allow_pickle=False) as z:
        actual = ray_surface_normals(z['depth'].T, z['angles'], z['elevation'], 15.)
        np.testing.assert_allclose(actual.transpose(1, 0, 2), z['normal'], atol=2e-5, rtol=0)
    q = np.asarray(protocol['queries'])
    with np.load(root/'native-output.npz', allow_pickle=False) as z:
        assert z['selected'].shape == (512, 64) and np.isfinite(z['selected']).all()
        for index in (0, 255, 511):
            spot = z[str(protocol['frame_ids'][index])]
            assert spot.shape == (256, 599) and np.isfinite(spot).all()
            np.testing.assert_array_equal(spot[q[:, 1], q[:, 0]], z['selected'][index])
            p = abs(spot.astype(np.complex128))**2
            assert np.all(z['power_sum'] >= p) and np.all(z['power_square_sum'] >= p*p)
    r = subprocess.run([sys.executable, str(root/'control.py'), '--output', str(root), '--verify'],
                       capture_output=True, text=True)
    if r.returncode:
        raise RuntimeError((r.stdout+r.stderr)[-4000:])
    with np.load(root/'moments.npz', allow_pickle=False) as z:
        actual = profile_power_evidence(z['measured_mean'], z['mean'], z['covariance']/512)
    original = json.loads((root/'gain-control.json').read_text())
    np.testing.assert_allclose(actual['gain'], original['gain'], rtol=2e-9)
    np.testing.assert_allclose(actual['normalized_residual'], original['normalized_residual'], atol=2e-9)
    with tempfile.TemporaryDirectory(prefix='sonar-power-field-') as folder:
        output = Path(folder)
        (output/'protocol.json').write_bytes((root/'protocol.json').read_bytes())
        r = subprocess.run([sys.executable, str(root/'field_baseline.py'),
                            '--records', str(root.parent), '--output', str(output)],
                           capture_output=True, text=True)
        if r.returncode:
            raise RuntimeError((r.stdout+r.stderr)[-4000:])
        with np.load(output/'field-ground-baseline.npz') as actual, \
                np.load(root/'field-ground-baseline.npz') as expected:
            assert sorted(actual.files) == sorted(expected.files)
            for name in actual.files:
                np.testing.assert_allclose(actual[name], expected[name], rtol=2e-7,
                                            atol=2e-10*max(float(np.max(abs(expected[name]))), 1e-30))
    print('PASS: 512 native kernel evaluations, all 64 selected pixels, gain and 3 raw-field baselines')


if __name__ == '__main__':
    main()
