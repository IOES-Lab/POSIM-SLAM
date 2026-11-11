"""Verify retained beam identities, failed hypotheses and overlap boundaries."""
import argparse
import hashlib
import io
import json
import math
from pathlib import Path
import sys
import tarfile

root=Path(__file__).resolve().parent
parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--raw',action='store_true');args=parser.parse_args()
def read(name):return json.loads((root/name).read_text())
def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()
for name,digest in read('sha256.json').items():assert sha(root/name)==digest,name
source=read('source.json');audit=read('audit.json')
assert not source['ping_present'] and not source['truth_is_estimator_input']
assert not source['native_estimator_changed']
assert source['input_component_commit']=='000fc4b7f0f8f189fa98bca59c67312ccdd44e27'
assert audit['streams']['sonar']['count']==135 and audit['streams']['camera']['count']==338
assert audit['streams']['imu']['count']==6748 and audit['streams']['imu']['max_gap_s']==.01
assert audit['actual_imu_rate_hz']==100 and audit['requested_imu_rate_hz']==1000
assert audit['camera_sonar_exact_pairs']==18 and audit['replay_window']['covered_intervals']==132
with tarfile.open(root/'capture.tar.gz') as tar:
    assert set(tar.getnames())==set(source['capture_files'])
    assert all(m.isfile() and not m.name.startswith('/') and '..' not in Path(m.name).parts for m in tar.getmembers())
    assert not any(n.endswith('.py') for n in tar.getnames())
    timeline=json.load(tar.extractfile('timeline.json'))
    native={row['stamp_ns']:row['message'] for row in json.load(tar.extractfile('native-dvl.json'))}
    common=json.load(tar.extractfile('dvl.json'));count=0
    assert len(native)==68 and len(common)==67 and not timeline['raw_ping'] and not timeline['errors']
    for row in common:
        original=native[row['stamp_ns']];beams={b['id']:b for b in original['beams']}
        assert row['valid'] and row['locked_beams']==4
        for beam in row['bottom_ranges']['beams']:
            raw=beams[beam['id']]
            assert beam['locked']==raw['locked'] and beam['range_m']==raw['range']['mean']
            assert beam['variance_m2'] is None and raw['range'].get('variance',0)==0
            count+=1
    assert count==268
    if args.raw:
        sys.dont_write_bytecode=True
        sys.path.insert(0,str(root.parents[2]/'src'))
        import numpy as np
        for member,digest in source['capture_files'].items():
            assert hashlib.sha256(tar.extractfile(member).read()).hexdigest()==digest,member
        for row in timeline['sonar']:
            with np.load(io.BytesIO(tar.extractfile(row['file']).read()),allow_pickle=False) as raw:
                assert raw['intensity_db'].shape==(399,256)
                assert int(raw['stamp'][0])*10**9+int(raw['stamp'][1])==row['stamp_ns']
                assert hashlib.sha256(raw['intensity_db'].astype('<f4').tobytes()).hexdigest()==row['raw_sha256']
        for row in timeline['camera']:
            with np.load(io.BytesIO(tar.extractfile(row['file']).read()),allow_pickle=False) as raw:
                assert hashlib.sha256(raw['data'].tobytes()).hexdigest()==row['source_sha256']
                assert int(raw['stamp_ns'])==row['stamp_ns']
assert sha(root/'capture.tar.gz')==source['archive_sha256']
assessment=read('baseline/assessment.json');points=read('baseline/acoustic-map.json')['points'];receipt=read('baseline/receipt.json')
assert len(points)==assessment['point_count']==len(assessment['per_point'])==61
assert {p['source_id'] for p in points}=={p['source_id'] for p in assessment['per_point']}
assert sum(p['error_m']>.1 for p in assessment['per_point'])==28
assert math.isclose(math.sqrt(sum(p['error_m']**2 for p in assessment['per_point'])/61),assessment['surface_error_m']['rmse'])
assert not assessment['map_qualified'] and not assessment['joint_uncertainty_available']
assert receipt['replay_complete'] and receipt['native_states']==214
assert receipt['dvl_optimization_states']==receipt['pressure_optimization_states']==66
assert receipt['camera_frames']==receipt['ping_fused_states']==0 and receipt['exit_code']==-15
assert len((root/'baseline/acoustic-map.ply').read_text().split('end_header\n')[1].splitlines())==61
main=read('dvl-plane-probe.json');control=read('dvl-plane-probe-assessment-control.json')
assert not main['truth_is_plane_diagnostic_input'] and control['truth_is_plane_diagnostic_input']
assert not main['truth_is_native_estimator_input'] and not control['truth_is_native_estimator_input']
for report in (main,control):
    assert not report['online_state_updated']
    for method in report['methods'].values():
        predictions=[b for row in method['rows'] for b in row.get('beams',[])]
        assert len(predictions)==method['all_point_hypotheses_retained']==252
        assert sum(method['support_counts'].values())==252 and not method['map_qualified']
        assert method['heldout_absolute_residual_m']['p90']>.36
spatial=read('dvl-spatial-probe.json')
assert spatial['all_range_points_retained']==264
assert spatial['association_counts']==dict(no_spatial_plane=212,dvl_cone_outside_surface_support=34,within_local_hypothesis_support=18)
assert not spatial['residual_is_independent_holdout'] and not spatial['map_qualified']
overlap=read('dvl-sonar-overlap.json')
assert overlap['all_ground_onsets']==6812 and overlap['pose_covered_sonar_frames']==133
assert overlap['patch_count']==52 and overlap['strict_dvl_patch_count']==18
assert not overlap['map_qualified'] and not overlap['physical_identity_verified'] and not overlap['native_state_updated']
assert all(v['five_samples_supported']==v['centre_supported']==0 for v in overlap['totals'].values())
assert overlap['totals']['sonar_first']['patch_pairs_with_support_sample_in_fan']==140
assert overlap['totals']['contemporaneous']['patch_pairs_with_support_sample_in_fan']==0
synthetic=read('synthetic-control/receipt.json')
assert synthetic['synthetic'] and synthetic['not_native_sensor_acceptance']
assert synthetic['results']['supported']['five_samples_supported']==22
assert synthetic['results']['outside']['five_samples_supported']==0
assert read('lifecycle.json')['exit']==0 and not read('lifecycle.json')['forced']
assert all(not g['live_members'] for g in read('lifecycle.json')['groups'])
assert read('context-receipt.json')['exit']==0 and not read('context-receipt.json')['forced']
assert read('cleanup.json')['protected_changed']==[] and read('cleanup.json')['protected_count']==16
figure=read('comparison-baseline.json')
assert figure['all_points_retained']==61 and figure['marker_area_pt2']==64
assert figure['camera_stamp_ns']==figure['sonar_stamp_ns']==36_000_000_000
for name in ('verification.json','wheel-receipt.json'):
    tested=read(name)
    assert tested['failures']==tested['errors']==0
assert read('verification.json')['source_tests']==read('wheel-receipt.json')['tests']==448
assert len(read('wheel-receipt.json')['skipped'])==16
assert read('wheel-receipt.json')['module_sha256']==read('wheel-receipt.json')['source_sha256']
print('PASS: original native beam ranges, all failed maps and zero supported overlap retained; no range fusion or map acceptance')
