"""Run an unchanged dated verifier with its hash-bound public source files.

Current implementation tests and the new sampled-ray field controls run on
current source separately. This runner preserves original input/source guards;
it does not rewrite experiment hashes or impersonate another module's path.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import stat
import zipfile

ROOT = Path(__file__).resolve().parents[1]
SNAPSHOT = ROOT/'docs/diagnostics/recorded-source-20261031'
LIMIT = 32*1024*1024


def prepare(package, snapshot, files, target, *, package_snapshot=None):
    """Verify snapshots before creating a bounded, independent package tree."""
    for name, digest in files.items():
        path = snapshot/name
        if Path(name).name != name or path.suffix != '.py' or path.is_symlink():
            raise ValueError('invalid_recorded_source_name')
        if hashlib.sha256(path.read_bytes()).hexdigest() != digest:
            raise ValueError('recorded_source_hash_mismatch')
    if package_snapshot is not None:
        name=package_snapshot['archive'];path=snapshot/name
        if Path(name).name!=name or path.is_symlink():
            raise ValueError('invalid_recorded_package_name')
        if hashlib.sha256(path.read_bytes()).hexdigest()!=package_snapshot['sha256']:
            raise ValueError('recorded_package_hash_mismatch')
        expected=package_snapshot['files'];payload={};size=0
        with zipfile.ZipFile(path) as source:
            members=source.infolist()
            if len(members)>512:raise ValueError('recorded_source_workspace_limit')
            for member in members:
                relative=Path(member.filename);mode=member.external_attr>>16
                if (relative.is_absolute() or '..' in relative.parts or '\\' in member.filename
                        or relative.suffix!='.py' or member.is_dir()
                        or member.filename in payload or member.filename not in expected
                        or stat.S_IFMT(mode) not in (0,stat.S_IFREG)):
                    raise ValueError('invalid_recorded_package_member')
                size+=member.file_size
                if size>LIMIT:raise ValueError('recorded_source_workspace_limit')
                data=source.read(member)
                if hashlib.sha256(data).hexdigest()!=expected[member.filename]:
                    raise ValueError('recorded_package_member_hash_mismatch')
                payload[member.filename]=data
        if set(payload)!=set(expected) or size!=package_snapshot['uncompressed_bytes']:
            raise ValueError('recorded_package_members_mismatch')
        target.mkdir(parents=True)
        for name,data in payload.items():
            destination=target/name;destination.parent.mkdir(parents=True,exist_ok=True)
            destination.write_bytes(data)
        return
    size = sum(p.stat().st_size for p in package.rglob('*')
               if p.is_file() and '__pycache__' not in p.parts)
    if size > LIMIT:
        raise ValueError('recorded_source_workspace_limit')
    shutil.copytree(package, target, symlinks=True,
                    ignore=shutil.ignore_patterns('__pycache__', '*.pyc'))
    for name in files:
        destination = target/name
        if destination.is_symlink():
            destination.unlink()
        shutil.copy2(snapshot/name, destination)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('verifier', type=Path)
    parser.add_argument('arguments', nargs=argparse.REMAINDER)
    args = parser.parse_args()
    verifier = args.verifier.resolve(strict=True)
    if not verifier.is_relative_to(ROOT/'docs/diagnostics'):
        raise ValueError('recorded_verifier_outside_diagnostics')
    profile = json.loads((SNAPSHOT/'sources.json').read_text())
    temporary_root = os.environ.get('POSIM_DIAGNOSTIC_TEMP_ROOT') or os.environ.get('RUNNER_TEMP')
    with tempfile.TemporaryDirectory(prefix='posim-recorded-source-', dir=temporary_root) as temporary:
        parent = Path(temporary)/'src'
        parent.mkdir()
        recorded_package=json.loads((SNAPSHOT/'package.json').read_text())
        prepare(ROOT/'src/posim_slam', SNAPSHOT, profile['files'], parent/'posim_slam',
                package_snapshot=recorded_package)
        environment = os.environ.copy()
        environment['PYTHONPATH'] = str(parent)+os.pathsep+environment.get('PYTHONPATH', '')
        print('Recorded-source replay: original source hashes and unchanged dated verifier; '
              'current implementation checks are separate.', flush=True)
        result = subprocess.run([sys.executable, str(verifier), *args.arguments], env=environment)
    return result.returncode


if __name__ == '__main__':
    raise SystemExit(main())
