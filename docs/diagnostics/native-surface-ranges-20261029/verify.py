"""Read-only record and original nonlinear-initialization verification."""
from pathlib import Path
import gzip,hashlib,json,sys
root=Path(__file__).resolve().parent
project=root.parents[2]
sys.path.insert(0,str(project/'src' if (project/'src/posim_slam').is_dir() else project/'libraries/posim-slam/src'))
from posim_slam.native_surface import NativeSurfaceSnapshot
from posim_slam.native_surface_ranges import prepare_native_surface_ranges
for name,entry in json.loads((root/'artifact-sha256.json').read_text()).items():
    path=root/name
    assert path.stat().st_size==entry['bytes'] and hashlib.sha256(path.read_bytes()).hexdigest()==entry['sha256'],name
index=json.loads((root/'snapshot-index.json').read_text())
for mode in ('guarded-dvl-only','guarded-dvl-fls'):
    request=json.loads((root/mode/'original-request.json').read_text());request['planes']={int(k):v for k,v in request['planes'].items()}
    before=json.loads(gzip.decompress((root/index[mode+'/snapshot-before.json.gz']).read_bytes()))
    after=json.loads(gzip.decompress((root/index[mode+'/snapshot-after.json.gz']).read_bytes()))
    snapshot=NativeSurfaceSnapshot.read(before)
    if mode=='guarded-dvl-only':
        result=prepare_native_surface_ranges(snapshot,**request)
        assert len(result.increment.original_keys)==2 and not result.diagnostics['initialization']['added_prior']
        assert result.diagnostics['initialization']['final_cost']<1e-12
    else:
        try:prepare_native_surface_ranges(snapshot,**request)
        except ValueError as error:assert any(word in str(error) for word in ('support','planar','nonlinear_seed')),str(error)
        else:raise AssertionError('Incorrect original correspondence was accepted')
        for field in ('owner_revision','prior_revision','variables','delta','conditionals','range_ledger'):
            assert before[field]==after[field],field
summary=json.loads((root/'summary.json').read_text())
assert summary['all_original_hypotheses']==664
assert sum(summary['original_errors_above_10cm_by_status'].values())==40
assert summary['protected_files_exact'] and summary['map_qualified'] is False
print('PASS: all record hashes; original DVL initialization; whole FLS candidate rejection; exact native owner preservation; all original errors retained')
