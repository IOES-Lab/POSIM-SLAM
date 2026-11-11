"""Check all retained controlled ranges, scope and conditional null directions."""
from pathlib import Path
import hashlib
import json
import math

root=Path(__file__).resolve().parent
def read(name):return json.loads((root/name).read_text())
for name,digest in read('sha256.json').items():
    assert hashlib.sha256((root/name).read_bytes()).hexdigest()==digest,name
results=read('geometry-results.json');original=read('fixture/native.json')
assert results['recorded_samples']==9 and len(results['rows'])==36
assert results['locked_returns']==24 and results['unlocked_beams']==12
assert results['map_qualified'] is False and results['truth_is_estimator_input'] is False
assert results['full_native_reexecution'] is False and results['continuous_is_discrete_renderer'] is False
assert results['uncertainty']['calibrated'] is False
measurements={(case['case'],int(sample['header']['stamp'].get('sec',0)),beam['id']):beam for case in original for sample in case['measurements'] for beam in sample['beams']}
for row in results['rows']:
    beam=measurements[(row['case'],int(row['stamp'].get('sec',0)),row['beam_id'])]
    assert row['locked'] is beam.get('locked',False)
    if row['locked']:
        assert row['measured_range_m']==beam['range']['mean']
        assert row['range_variance_reported'] is None
        assert math.isclose(row['measured_range_m']-row['continuous_prediction_m'],row['continuous_residual_m'],abs_tol=1e-14)
        assert row['continuous_residual_m']>0
    else:assert 'measured_range_m' not in row
assert all(row['conditional_pose_rank']==3 and row['horizontal_and_yaw_column_norm']<1e-10 for row in results['frames'])
assert abs(results['continuous_absolute_error_m']['maximum']-.006171735379618459)<1e-12
assert abs(results['centre_absolute_error_m']['maximum']-.009486751546435634)<1e-12
verification=read('verification.json')
assert verification['library_tests']==443 and verification['skipped']==16
assert verification['failures']==verification['errors']==0
assert verification['source_export_excludes_protected_work'] is True
wheel=read('wheel-receipt.json')
assert wheel['tests']==443 and len(wheel['skipped'])==16
assert wheel['failures']==wheel['errors']==0
assert wheel['imported_from'].startswith('installed/') and wheel['cwd']=='outside'
assert wheel['dvl_surface_sha256']==verification['source_files_sha256']['src/posim_slam/dvl_surface.py']
print('PASS: all original ranges, nonzero discretization residuals and geometric null directions retained; no live fusion or map acceptance')
