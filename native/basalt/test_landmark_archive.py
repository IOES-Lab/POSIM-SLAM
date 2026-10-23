"""Compile and exercise the same bounded archive used by the native worker."""
import shutil,subprocess,tempfile,unittest
from pathlib import Path

class ArchiveTests(unittest.TestCase):
    def test_maturity_identity_updates_retention_budget_faults_and_segment_reset(self):
        compiler=shutil.which('c++')
        self.assertIsNotNone(compiler,'C++17 compiler required')
        with tempfile.TemporaryDirectory() as folder:
            target=Path(folder)/'archive_probe'
            subprocess.run([compiler,'-std=c++17','-Wall','-Wextra','-Werror',
                str(Path(__file__).with_name('archive_probe.cc')),'-o',str(target)],check=True,capture_output=True,timeout=30)
            subprocess.run([str(target)],check=True,capture_output=True,timeout=5)

if __name__=='__main__':unittest.main()
