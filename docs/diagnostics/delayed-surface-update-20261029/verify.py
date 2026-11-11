"""Check complete retained widths, numerical controls and truthful scope."""
import argparse
import gzip
import hashlib
import json
from pathlib import Path

root=Path(__file__).resolve().parent
parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--library-root',type=Path)
args=parser.parse_args()
read=lambda name:json.loads((root/name).read_text())
for name,digest in read('sha256.json').items():assert hashlib.sha256((root/name).read_bytes()).hexdigest()==digest,name
q=read('qualification.json')
if args.library_root:
    for name,digest in {**q['source_sha256'],**q['input_sha256']}.items():
        assert hashlib.sha256((args.library_root/name).read_bytes()).hexdigest()==digest,name
value=json.loads(gzip.decompress((root/'standalone-audit.json.gz').read_bytes()))
rows=value['rows'];by_row={row['row']:row for row in rows}
assert len(rows)==len(by_row)==value['requests']==664 and set(by_row)==set(range(664))
assert sum(r['dense_columns']>1024 for r in rows)==value['dense_over_1024']==187
assert max(r['dense_columns'] for r in rows)==value['max_dense_columns']==1444
assert max(r['symbolic_front_columns'] for r in rows)==value['max_symbolic_front_columns']==304
assert {row['row'] for row in value['null_controls']}=={67,69}
for row in value['null_controls']:
    original=by_row[row['row']]
    assert row['dense_columns']==original['dense_columns']
    assert row['peak_front_columns']==original['symbolic_front_columns']
    assert row['new_information_rows_norm']==row['new_information_offset_norm']==0
    assert row['maximum_mean_change']<4e-16 and row['covariance_difference_norm']<7e-17
    assert row['workspace_bound_bytes']<6*1024*1024
assert value['synthetic_zero_information_root'] and value['new_physical_range_measurements']==0
assert not value['live_complete_prior_recorded'] and not value['native_worker_updated']
assert not value['navigation_covariance_qualified'] and not value['map_qualified']
initial=json.loads(gzip.decompress((root/'initial-standalone-audit.json.gz').read_bytes()))
assert any(r['new_information_rows_norm']>0 for r in initial['null_controls'])
assert initial['requests']==664 and initial['max_dense_columns']==1444
wheel=read('wheel-receipt.json')
assert (wheel['tests_run'],wheel['failures'],wheel['errors'],len(wheel['skipped']))==(487,0,0,16)
for name in ('logs/final-source-suite.log.gz','logs/final-installed-wheel-tests.log.gz'):
    log=gzip.decompress((root/name).read_bytes()).decode()
    assert 'Ran 487 tests' in log and 'OK (skipped=16)' in log,name
assert q['new_controls']==10 and (q['source_tests'],q['source_passed'],q['source_skipped'])==(487,471,16)
assert q['source_tests']==q['wheel_tests'] and q['new_dependencies']==[]
assert q['initial_dense_prototype_rejected'] and q['initial_projection_roundoff_rejected']
assert not any(q[k] for k in ('native_worker_updated','default_live_surface_fusion_enabled','map_qualified',
                             'installed_application_qualified','protected_user_edits_included','full_goal_complete'))
print('PASS: all 664 widths, exact-zero null controls, 487 source/wheel cases and explicit native/map limits')
