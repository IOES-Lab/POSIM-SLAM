"""Verify complete failed maps and independent range prediction, optionally raw data."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

root=Path(__file__).resolve().parent
parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--raw',action='store_true');parser.add_argument('--data-root',type=Path)
args=parser.parse_args()
def read(name):return json.loads((root/name).read_text())
def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()
for name,digest in read('sha256.json').items():assert sha(root/name)==digest,name
pointer=read('data-pointer.json')
assert not pointer['raw_data_in_git'] and pointer['total_bytes']==97241159
assert len(pointer['files'])==494 and sum(f['bytes'] for f in pointer['files'].values())==pointer['total_bytes']
assert not list(root.glob('*.tar.gz'))
model=ET.parse(root/'sensor-model.sdf').getroot()
sensor=next(s for s in model.findall('.//sensor') if s.findtext('topic')=='/vehicle/sensors/multibeam_forward')
geometry=sensor.find('{http://gazebosim.org}multibeam_sonar')
assert geometry.findtext('ray/range/max')==geometry.findtext('spec/maxDistance')=='15'
audit=read('audit.json')
assert audit['streams']['sonar']['count']==116 and audit['streams']['camera']['count']==292
assert audit['streams']['imu']['count']==5828 and audit['streams']['imu']['max_gap_s']==.01
assert audit['streams']['dvl']['count']==58 and audit['streams']['pressure']['count']==582
assert audit['camera_sonar_exact_pairs']==11 and audit['replay_window']['covered_intervals']==113
assert read('dvl-integrity.json')['matched_beam_ranges']==232
assert not read('dvl-integrity.json')['unmatched_common_stamps']
paired=read('paired-trajectory.json')['cases']
assert paired['baseline']['input_sha256']==paired['no-acoustic']['input_sha256']
for method in ('baseline','no-acoustic'):
    receipt=read(method+'/receipt.json')
    assert receipt['replay_complete'] and receipt['native_states']==178
    assert receipt['pressure_optimization_states']==receipt['dvl_optimization_states']==57
    assert receipt['camera_frames']==receipt['ping_fused_states']==0 and receipt['exit_code']==-15
    assert receipt['sonar_factors_omitted']==(method=='no-acoustic')
    assert receipt['worker_sha256']=='cb5cbc7c254b668ea0b0e3c7ac0f2fe8dac0d4f07810bab2e8d3237ca18d1d87'
assert read('no-acoustic/receipt.json')['sonar_track_updates_supplied']==0
assert read('no-acoustic/acoustic-map.json')['points']==[]
assert paired['baseline']['trajectory']['position_rmse_m']>paired['no-acoustic']['trajectory']['position_rmse_m']
archive=read('baseline/acoustic-map.json');assessment=read('baseline/assessment.json')
assert len(archive['points'])==assessment['point_count']==len(assessment['per_point'])==102
assert {p['source_id'] for p in archive['points']}=={p['source_id'] for p in assessment['per_point']}
assert sum(p['error_m']>.1 for p in assessment['per_point'])==56
assert math.isclose(math.sqrt(sum(p['error_m']**2 for p in assessment['per_point'])/102),assessment['surface_error_m']['rmse'])
assert not assessment['map_qualified'] and not assessment['joint_uncertainty_available']
assert len((root/'baseline/acoustic-map.ply').read_text().split('end_header\n')[1].splitlines())==102
conditional=read('conditional-fit.json')
assert conditional['source_points']==conditional['fitted']==len(conditional['rows'])==102
assert conditional['summary']['major_three_sigma_over_010_m']==102
assert conditional['summary']['outside_vertical_fan']==11 and not conditional['truth_is_estimator_input']
plane=read('dvl-plane-probe.json')['methods']['continuous_cone']
assert plane['all_point_hypotheses_retained']==216 and sum(plane['support_counts'].values())==216
assert plane['support_counts']['within_hypothesis_support']==54 and not plane['map_qualified']
spatial=read('dvl-spatial-probe.json')
assert spatial['all_range_points_retained']==228 and spatial['association_counts']['within_local_hypothesis_support']==56
overlap=read('dvl-sonar-overlap.json')
assert overlap['all_ground_onsets']==5903 and overlap['patch_count']==67
assert overlap['totals']['sonar_first']['distinct_sonar_onsets']==251
assert overlap['totals']['dvl_first']['distinct_sonar_onsets']==87
assert overlap['totals']['contemporaneous']['five_samples_supported']==0
prediction=read('dvl-sonar-prediction.json');rows=prediction['rows']
assert prediction['selected_predictions']==prediction['observed_onsets']==len(rows)==664
assert prediction['missing_onsets']==0 and not prediction['onset_range_used_for_prediction_or_support']
assert prediction['pose_replay_omits_sonar'] and not prediction['truth_is_estimator_input']
assert not prediction['map_qualified'] and not prediction['native_state_updated']
assert all(math.isclose(r['observed_range_m']-r['predicted_range_m'],r['residual_m'],abs_tol=1e-12) for r in rows)
assert sum(abs(r['residual_m'])<=.1 for r in rows)==624
assert sum(r['residual_m']<-.1 for r in rows)==35 and sum(r['residual_m']>.1 for r in rows)==5
assert prediction['absolute_residual_m']['maximum']>2.67
figure=read('comparison-baseline.json')
assert figure['camera_stamp_ns']==figure['sonar_stamp_ns']==figure['isometric_stamp_ns']==35_000_000_000
assert figure['all_points_retained']==102 and figure['marker_area_pt2']==64
assert figure['colours']['colour_saturated_points']==0
assert read('context-receipt.json')['recorded_pose_stamp_ns']==35_000_000_000
for name in ('lifecycle.json','context-receipt.json'):
    report=read(name);assert report['exit']==0 and not report['forced']
assert all(not g['live_members'] for g in read('lifecycle.json')['groups'])
assert read('cleanup.json')['protected_changed']==[] and not read('cleanup.json')['raw_data_in_git']
assert read('verification.json')['source_tests']==451 and read('verification.json')['source_skipped']==16
assert 'Ran 451 tests' in (root/'source-tests.log').read_text()
if args.raw:
    dataset=(args.data_root or Path(pointer['canonical_directory'])).resolve()
    for name,record in pointer['files'].items():
        path=(dataset/name).resolve()
        assert path.is_relative_to(dataset) and path.is_file(),name
        assert path.stat().st_size==record['bytes'] and sha(path)==record['sha256'],name
    import numpy as np
    timeline=json.loads((dataset/'inputs/timeline.json').read_text())
    native={r['stamp_ns']:r['message'] for r in json.loads((dataset/'inputs/native-dvl.json').read_text())}
    for row in timeline['raw_dvl']:
        original={b['id']:b for b in native[row['stamp_ns']]['beams']}
        for beam in row['bottom_ranges']['beams']:
            assert beam['range_m']==original[beam['id']]['range']['mean'] and beam['locked']==original[beam['id']]['locked']
            assert beam['variance_m2'] is None
    for row in timeline['sonar']:
        with np.load(dataset/'inputs'/row['file'],allow_pickle=False) as frame:
            assert frame['intensity_db'].shape==(598,256)
            assert int(frame['stamp'][0])*10**9+int(frame['stamp'][1])==row['stamp_ns']
            assert hashlib.sha256(frame['intensity_db'].astype('<f4').tobytes()).hexdigest()==row['raw_sha256']
    for row in timeline['camera']:
        with np.load(dataset/'inputs'/row['file'],allow_pickle=False) as frame:
            assert int(frame['stamp_ns'])==row['stamp_ns']
            assert hashlib.sha256(frame['data'].tobytes()).hexdigest()==row['source_sha256']
print('PASS: full failed point maps, independent geometric predictions and all outliers retained; original raw data separately indexed')
