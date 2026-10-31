"""Recompute complete source-normal and conditional moment controls."""
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import numpy as np
from posim_slam.sonar_source_normals import ray_surface_normals
from run_controls import cases


def main():
    here = Path(__file__).resolve().parent
    for name, expected in json.loads((here/'sha256.json').read_text()).items():
        if hashlib.sha256((here/name).read_bytes()).hexdigest() != expected:
            raise ValueError('record_hash_mismatch:'+name)
    import posim_slam.sonar_source_normals as module
    package = Path(module.__file__).resolve().parent
    sources = json.loads((here/'component-source.json').read_text())
    for name, expected in sources['python_sha256'].items():
        if hashlib.sha256((package/name).read_bytes()).hexdigest() != expected:
            raise ValueError('model_source_mismatch:'+name)
    protocol = json.loads((here/'protocol.json').read_text())
    first = json.loads((here/'normal-reference-first-comparison.json').read_text())
    corrected = json.loads((here/'normal-corrected-comparison.json').read_text())
    assert not first['passed'] and corrected['passed']
    assert first['source_header_sha256'] == sources['native_header_sha256']
    names, worst = [], 0.
    with np.load(here/'native-normals.npz', allow_pickle=False) as z:
        for name, az, el, ranges in cases():
            names.append(name)
            for suffix, expected in (('ranges', ranges), ('azimuth', az), ('elevation', el)):
                np.testing.assert_array_equal(z[name+'_'+suffix], expected)
            actual = ray_surface_normals(ranges, az, el, 15.)
            expected = z[name+'_normals']
            np.testing.assert_array_equal(np.linalg.norm(actual, axis=-1) > 0,
                                           np.linalg.norm(expected, axis=-1) > 0)
            error = float(np.max(abs(actual.astype(float)-expected)))
            worst = max(worst, error)
            np.testing.assert_allclose(actual, expected,
                                        atol=protocol['normal_absolute_tolerance'], rtol=0)
    assert names == protocol['normal_controls']
    assert len(names) == len(first['cases']) == len(corrected['cases']) == 10
    with tempfile.TemporaryDirectory(prefix='posim-source-local-') as directory:
        output = Path(directory)/'moments.json'
        run = subprocess.run([sys.executable, str(here/'benchmark_moments.py'),
                              '--output', str(output)], capture_output=True, text=True)
        if run.returncode:
            raise RuntimeError('moment_reproduction_failed:\n'+(run.stdout+run.stderr)[-4000:])
        report = json.loads(output.read_text())
        assert report['passed'] and len(report['cases']) == 2
        with np.load(here/'moment-control.npz', allow_pickle=False) as original, \
                np.load(output.with_suffix('.npz'), allow_pickle=False) as reproduced:
            assert sorted(original.files) == sorted(reproduced.files)
            for name in original.files:
                expected, actual = original[name], reproduced[name]
                if name.endswith('_pixels'):
                    np.testing.assert_array_equal(actual, expected)
                else:
                    np.testing.assert_allclose(actual, expected, rtol=2e-8,
                                                atol=2e-12*max(float(np.max(abs(expected))), 1e-30))
    print(f'PASS: all 10 native normal grids (max component difference {worst:.7g}); '
          '16/64-pixel full correlated moments reproduced')


if __name__ == '__main__':
    main()
