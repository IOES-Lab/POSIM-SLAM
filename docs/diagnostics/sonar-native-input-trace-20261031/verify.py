# SPDX-License-Identifier: Apache-2.0
"""Portable byte audit and optional standard-C++ test, without SDK or world."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import tempfile
from verify_native import ROOT, verify


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--native-unit', action='store_true')
    a=p.parse_args()
    for name,digest in json.loads((ROOT/'sha256.json').read_text()).items():
        assert hashlib.sha256((ROOT/name).read_bytes()).hexdigest()==digest,name
    operand=ROOT.parent/'sonar-reflector-birth-20261031/native-reference/inputs.npz'
    result=verify(operand,suffix='-final')
    original=json.loads((ROOT/'unmodified-native-comparison.json').read_text())
    baseline=json.loads((ROOT/'native-disabled-final/result.json').read_text())
    assert original['complete_original_images_match_instrumented_adapter']
    assert original['actual_symbol_path_verified'] and original['frames']==baseline['frames']
    assert not original['field_capture']
    build=json.loads((ROOT/'build-receipt-native-final.json').read_text())
    assert build['binary_sha256']==baseline['binary_sha256']
    for name,expected in build['source_sha256'].items():
        file=ROOT/name if name=='native_replay.cc' else ROOT/'source'/name
        assert hashlib.sha256(file.read_bytes()).hexdigest()==expected,name
    if a.native_unit:
        cc=shutil.which('c++')
        if not cc:raise RuntimeError('C++17 compiler required for requested native unit control')
        with tempfile.TemporaryDirectory(prefix='posim-native-trace-') as temporary:
            folder=Path(temporary);binary=folder/'trace-test'
            subprocess.run([cc,'-std=c++17','-O2','-DNDEBUG','-pthread','-I'+str(ROOT/'source'),
                            str(ROOT/'source/NativeTrace_TEST.cc'),'-o',str(binary)],check=True)
            subprocess.run([str(binary),'ctest'],check=True)
            import run_unit_controls
            run_unit_controls.run(binary,folder/'isolated')
    assert result==json.loads((ROOT/'native-verification-final.json').read_text())
    print('PASS: every captured operand/output, original ABI comparison and declared source identity',flush=True)


if __name__=='__main__':main()
