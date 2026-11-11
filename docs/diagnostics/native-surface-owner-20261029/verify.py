"""Verify native-owner evidence without an application or simulator."""
import gzip,hashlib,json,tarfile
from pathlib import Path
root=Path(__file__).resolve().parent;read=lambda name:json.loads((root/name).read_text())
for name,digest in read('sha256.json').items():assert hashlib.sha256((root/name).read_bytes()).hexdigest()==digest,name
q=read('qualification.json');metrics=q['native_metrics']
assert q['native_owner_controls']==10 and q['persistent_prior_controls']==5
for name,limit in [('max_information_error',1e-8),('max_gradient_error',1e-8),('max_squared_residual_error',1e-8),
                   ('max_covariance_error',1e-10),('max_solution_error',1e-7)]:assert 0<=metrics[name]<limit,name
for mode in ('source','wheel'):
    r=read(mode+'-receipt.json');assert r['tests_run']==487 and not r['failures'] and not r['errors'] and len(r['skipped'])==16
assert not read('frozen-source.json')['protected_user_edits_included']
with tarfile.open(root/'group-logs.tar.gz') as logs:
    assert b'Ran 10 tests' in logs.extractfile('native-final/01.log').read()
    assert b'Ran 5 tests' in logs.extractfile('native-regressions/01.log').read()
    for index in (2,3):assert b'"passed":true' in logs.extractfile('native-regressions/0'+str(index)+'.log').read()
for path in sorted((root/'traces').glob('*.gz')):
    trace=json.loads(gzip.decompress(path.read_bytes()))
    assert trace and all('error' not in item['response'] and 'state_error' not in item['response'] for item in trace)
    assert all(item['response']['state']['legacy_zero'] for item in trace)
    if 'second_retirement' in path.name:
        final=trace[-1]['response']['state'];assert len(final['conditionals'])==2 and final['pressure_count']==1
        assert len(final['prior']['ledger']['recent'])==12
    else:
        first=trace[0]['response']['state'];final=trace[-1]['response']['state']
        assert first['imu_count']==2 and final['imu_count']==0 and final['dvl_count']==1 and final['pressure_count']==2
assert len(list((root/'traces').glob('*.gz')))==2
assert q['synthetic_native_owner_updated'] and q['all_16_user_files_unchanged']
assert not any(q[k] for k in ('installed_sdk_modified','live_worker_updated','new_sensor_capture','physical_sonar_map','full_goal_complete'))
print('PASS: ten actual-owner controls, five persistent-prior controls, DVL/pressure regressions and independent 487-case source/wheel suites; physical map and worker integration remain separate')
