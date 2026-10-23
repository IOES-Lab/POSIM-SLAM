"""Check independent ownership, distributable resources and pinned notices."""
import ast,hashlib,json,unittest
from pathlib import Path
import posim_slam
from posim_slam import resources


class IntegrityTests(unittest.TestCase):
    def test_notice_hashes_match_exact_visual_dependency_pins(self):
        manifest=json.loads((resources.NOTICES/'manifest.json').read_text())
        pins=json.loads((resources.VISUAL/'sources.json').read_text())
        self.assertEqual(manifest['sources'],pins)
        for entry in manifest['files']:
            self.assertEqual(entry['revision'],pins[entry['component']])
            self.assertEqual(hashlib.sha256((resources.NOTICES/entry['file']).read_bytes()).hexdigest(),entry['sha256'])

    def test_package_has_no_application_or_flat_sibling_imports(self):
        package=Path(posim_slam.__file__).parent
        siblings={p.stem for p in package.glob('*.py')}
        forbidden={'runtime','browser_adapter','practice_acoustic','library_paths','workspace','slam'}
        for path in package.glob('*.py'):
            for node in ast.walk(ast.parse(path.read_text())):
                if isinstance(node,ast.ImportFrom) and node.level==0 and node.module:
                    top=node.module.split('.')[0]
                    self.assertNotIn(top,forbidden,path.name)
                    self.assertNotIn(top,siblings,path.name)
                elif isinstance(node,ast.Import):
                    for alias in node.names:self.assertNotIn(alias.name.split('.')[0],forbidden,path.name)

    def test_native_sources_and_patch_are_package_resources(self):
        for path in (resources.NATIVE/'acoustic_cv.cc',resources.NATIVE/'visual/visual.cc',
                     resources.NATIVE/'visual/CMakeLists.txt',resources.VISUAL/'eigen-solvers.patch'):
            self.assertTrue(path.is_file(),str(path))
        self.assertEqual(posim_slam.__version__,'0.1.0')


if __name__=='__main__':unittest.main()
