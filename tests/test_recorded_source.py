import hashlib
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest
import numpy as np
from posim_slam.sonar_surface_power import SurfacePowerModel, TriangleSurface
from posim_slam.sonar_source_surface import SourceSurfaceObservation

ROOT = Path(__file__).resolve().parents[1]
SNAPSHOT = ROOT/'docs/diagnostics/recorded-source-20261031'


def load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class RecordedSourceTests(unittest.TestCase):
    def test_default_observations_match_exact_original_modules(self):
        profile = json.loads((SNAPSHOT/'sources.json').read_text())['files']
        for name, digest in profile.items():
            self.assertEqual(hashlib.sha256((SNAPSHOT/name).read_bytes()).hexdigest(), digest)
        old = load('posim_slam._recorded_power', SNAPSHOT/'sonar_surface_power.py')
        source = load('posim_slam._recorded_surface', SNAPSHOT/'sonar_source_surface.py')
        for az, el in [(np.linspace(-.2, .2, 7), np.linspace(-.1, .1, 5)),
                       (np.linspace(-np.pi/3, np.pi/3, 256).astype(np.float32),
                        np.linspace(-np.pi/18, np.pi/18, 16).astype(np.float32))]:
            current = SurfacePowerModel(az, el, np.arange(80)*.1, maximum_range_m=7.9)
            previous = old.SurfacePowerModel(az, el, np.arange(80)*.1, maximum_range_m=7.9)
            triangles = np.array([[[3., -20., -20.], [3., 20., -20.], [3., 20., 20.]],
                                  [[3., -20., -20.], [3., 20., 20.], [3., -20., 20.]]])
            a = SourceSurfaceObservation(current, TriangleSurface(triangles))
            b = source.SourceSurfaceObservation(previous, old.TriangleSurface(triangles))
            for name in ['directions', 'range_mix', 'beam_mix']:
                np.testing.assert_array_equal(getattr(current, name), getattr(previous, name))
            for name in ['ranges', 'hit_triangles', 'normals', 'unit_power']:
                np.testing.assert_array_equal(getattr(a, name), getattr(b, name))
            pixels = np.array([[30, 1], [31, len(az)//2]])
            for name in ['mean', 'covariance']:
                np.testing.assert_array_equal(a.moments([.001, .001], pixels)[name],
                                              b.moments([.001, .001], pixels)[name])

    def test_tampered_snapshot_is_rejected_before_any_package_copy(self):
        helper = load('recorded_runner', ROOT/'tools/verify_recorded_diagnostic.py')
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary);snapshot = root/'snapshot';snapshot.mkdir()
            (snapshot/'example.py').write_text('value = 2\n')
            target = root/'target'
            with self.assertRaisesRegex(ValueError, 'hash_mismatch'):
                helper.prepare(root/'missing-package', snapshot, {'example.py': '0'*64}, target)
            self.assertFalse(target.exists())


if __name__ == '__main__':
    unittest.main()
