import hashlib
import importlib.util
import json
from pathlib import Path
import re
import tempfile
import unittest
import zipfile
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
    def test_source_bound_ci_verifiers_use_the_recorded_package(self):
        package = json.loads((SNAPSHOT/'package.json').read_text())['files']
        workflow = (ROOT/'.github/workflows/tests.yml').read_text()
        checked = []
        for line in workflow.splitlines():
            if line.lstrip().startswith('#'):
                continue
            for runner, verifier in re.findall(
                    r'python\s+(?:(tools/verify_recorded_diagnostic\.py)\s+)?'
                    r'(docs/diagnostics/[^\s]+/verify\.py)', line):
                manifest = ROOT/verifier
                manifest = manifest.parent/'component-source.json'
                if not manifest.is_file():
                    continue
                expected = {name: digest for name, digest in json.loads(manifest.read_text()).items()
                            if name.endswith('.py') and isinstance(digest, str)}
                if not expected:
                    continue
                with self.subTest(verifier=verifier):
                    self.assertEqual({name: package.get(name) for name in expected}, expected)
                    self.assertEqual(runner, 'tools/verify_recorded_diagnostic.py',
                                     'A source-bound historical record must use its recorded package')
                checked.append(verifier)
        self.assertTrue(checked)

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

    def test_complete_snapshot_does_not_mix_in_current_or_untracked_modules(self):
        helper=load('recorded_complete_runner',ROOT/'tools/verify_recorded_diagnostic.py')
        with tempfile.TemporaryDirectory() as temporary:
            root=Path(temporary);snapshot=root/'snapshot';snapshot.mkdir()
            current=root/'current';current.mkdir();(current/'private.py').write_text('private = True\n')
            data={'__init__.py':b'', 'example.py':b'value = 1\n'}
            archive=snapshot/'package.zip'
            with zipfile.ZipFile(archive,'w') as out:
                for name,raw in data.items():out.writestr(name,raw)
            profile=dict(archive=archive.name,sha256=hashlib.sha256(archive.read_bytes()).hexdigest(),
                files={name:hashlib.sha256(raw).hexdigest() for name,raw in data.items()},
                uncompressed_bytes=sum(map(len,data.values())))
            target=root/'target'
            helper.prepare(current,snapshot,{},target,package_snapshot=profile)
            self.assertEqual((target/'example.py').read_bytes(),data['example.py'])
            self.assertFalse((target/'private.py').exists())
            self.assertEqual(sorted(p.name for p in target.iterdir()),sorted(data))

    def test_archive_hash_and_member_escape_fail_before_output(self):
        helper=load('recorded_package_rejections',ROOT/'tools/verify_recorded_diagnostic.py')
        with tempfile.TemporaryDirectory() as temporary:
            root=Path(temporary);archive=root/'package.zip'
            with zipfile.ZipFile(archive,'w') as out:out.writestr('../escape.py',b'bad\n')
            profile=dict(archive=archive.name,sha256='0'*64,
                files={'../escape.py':hashlib.sha256(b'bad\n').hexdigest()},uncompressed_bytes=4)
            for digest,error in [('0'*64,'hash_mismatch'),
                    (hashlib.sha256(archive.read_bytes()).hexdigest(),'invalid_recorded_package_member')]:
                with self.subTest(error=error),self.assertRaisesRegex(ValueError,error):
                    helper.prepare(root/'missing',root,{},root/'target',
                        package_snapshot=dict(profile,sha256=digest))
                self.assertFalse((root/'target').exists())
                self.assertFalse((root.parent/'escape.py').exists())


if __name__ == '__main__':
    unittest.main()
