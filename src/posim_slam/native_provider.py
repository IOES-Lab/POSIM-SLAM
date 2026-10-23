"""Check the optional SDK-relative acoustic frontend before process launch."""
import hashlib,json
from pathlib import Path

def digest(path):return hashlib.sha256(path.read_bytes()).hexdigest()
def checked_library(path,engine=None):
    library=Path(path).resolve()
    if library.name!='libwwos_acoustic_cv.dylib' or library.parent.name!='lib' or library.parent.parent.name!='install':
        raise ValueError('invalid_acoustic_native_layout')
    provider=library.parents[2]
    receipt=json.loads((provider/'receipt.json').read_text())
    if not isinstance(receipt,dict):raise ValueError('invalid_acoustic_native_receipt')
    if (receipt.get('schema')!='wwwposim-acoustic-native-1' or receipt.get('passed') is not True or receipt.get('abi')!=1 or receipt.get('opencv_version')!='4.13.0' or receipt.get('library_sha256')!=digest(library)):
        raise ValueError('acoustic_native_provider_changed')
    if receipt.get('source_sha256')!=digest(Path(__file__).with_name('data')/'native/acoustic_cv.cc'):
        raise ValueError('acoustic_native_source_changed')
    if engine is not None:
        engine=Path(engine).resolve();files=receipt.get('sdk_files')
        if not isinstance(files,dict) or not files:raise ValueError('acoustic_sdk_manifest_missing')
        for relative,checksum in files.items():
            candidate=(engine/relative).resolve()
            if not candidate.is_relative_to(engine) or digest(candidate)!=checksum:raise ValueError('acoustic_sdk_changed')
    return library
