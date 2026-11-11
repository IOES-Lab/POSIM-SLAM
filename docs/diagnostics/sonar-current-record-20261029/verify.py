"""Verify retained raw data and every hypothesis without application imports."""
from pathlib import Path
import argparse,gzip,hashlib,io,json,math,sys,tarfile
root=Path(__file__).resolve().parent
parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--raw',action='store_true');args=parser.parse_args()
def read(name):return json.loads((root/name).read_text())
def sha(path):
    h=hashlib.sha256()
    with path.open('rb') as f:
        for block in iter(lambda:f.read(1024*1024),b''):h.update(block)
    return h.hexdigest()
for name,digest in read('sha256.json').items():assert sha(root/name)==digest,name
source=read('source.json');audit=read('audit.json')
assert source['ping_present'] is False and source['truth_is_estimator_input'] is False
assert source['input_component_commit']=='e0521d6eaf306b2e836aa3bb9445830e0f626b1a'
assert audit['source_camera_checksums_verified'] and audit['raw_sonar_checksums_verified']
assert audit['streams']['sonar']['count']==132 and audit['streams']['camera']['count']==330
assert audit['streams']['imu']['count']==6592 and audit['streams']['imu']['max_gap_s']==.01
assert audit['dvl_valid']==audit['dvl_total']==66
assert audit['camera_sonar_exact_pairs']==15 and audit['replay_window']['covered_intervals']==129
assert audit['requested_imu_rate_hz']==1000 and audit['actual_imu_rate_hz']==100
with tarfile.open(root/'capture.tar.gz') as tar:
    names=tar.getnames()
    assert set(names)==set(source['capture_files'])
    assert all(m.isfile() and not m.name.startswith('/') and '..' not in Path(m.name).parts for m in tar.getmembers())
    assert not any(n.endswith('.py') for n in names)
    assert len([n for n in names if n.startswith('sonar-') and n.endswith('.npz')])==132
    assert len([n for n in names if n.startswith('camera-') and n.endswith('.npz')])==330
    timeline=json.load(tar.extractfile('timeline.json'))
    assert not timeline['raw_ping'] and not timeline['errors']
    assert len(timeline['raw_pressure'])==659 and len(timeline['raw_dvl'])==66
    if args.raw:
        sys.dont_write_bytecode=True
        sys.path.insert(0,str(Path(__file__).resolve().parents[3]/'src'))
        import numpy as np
        from posim_slam.image_input import gray8
        for row in timeline['sonar']:
            with np.load(io.BytesIO(tar.extractfile(row['file']).read())) as raw:
                assert int(raw['stamp'][0])*10**9+int(raw['stamp'][1])==row['stamp_ns']
                assert raw['intensity_db'].shape==(399,256)
                assert hashlib.sha256(raw['intensity_db'].astype('<f4').tobytes()).hexdigest()==row['raw_sha256']
                assert not np.isnan(raw['intensity_db']).any() and not np.isposinf(raw['intensity_db']).any()
        for row in timeline['camera']:
            with np.load(io.BytesIO(tar.extractfile(row['file']).read())) as raw:
                pixels=raw['data'].tobytes()
                assert hashlib.sha256(pixels).hexdigest()==row['source_sha256']
                converted=gray8(int(raw['width']),int(raw['height']),int(raw['step']),str(raw['encoding']),pixels)
                assert hashlib.sha256(converted.tobytes()).hexdigest()==row['input_gray8_sha256']
                assert int(raw['stamp_ns'])==row['stamp_ns']
for method,count,outside in (('baseline',96,43),('boundary',34,26)):
    assessment=read(method+'/assessment.json');archive=read(method+'/acoustic-map.json');receipt=read(method+'/receipt.json')
    assert receipt['replay_complete'] and receipt['native_states']==207
    assert receipt['camera_frames']==receipt['ping_fused_states']==0
    assert receipt['dvl_optimization_states']==receipt['pressure_optimization_states']==65
    assert receipt['exit_code']==-15 and receipt['worker_shutdown']=='client close after replay'
    assert len(archive['points'])==len(assessment['per_point'])==assessment['point_count']==count
    assert {p['source_id'] for p in archive['points']}=={p['source_id'] for p in assessment['per_point']}
    errors=[p['error_m'] for p in assessment['per_point']]
    assert all(math.isfinite(e) and e>=0 for e in errors)
    assert sum(e>.1 for e in errors)==outside
    assert abs(math.sqrt(sum(e*e for e in errors)/count)-assessment['surface_error_m']['rmse'])<1e-12
    assert assessment['map_qualified'] is False and archive['joint_uncertainty_available'] is False
    assert len((root/method/'acoustic-map.ply').read_text().split('end_header\n')[1].splitlines())==count
    figure=read('comparison-'+method+'.json')
    assert figure['camera_stamp_ns']==figure['sonar_stamp_ns']==36_000_000_000
    assert figure['all_points_retained']==count and figure['marker_area_pt2']==64
    assert figure['context_is_clear_water_reference'] is True
assert read('native-lifecycle.json')['exit']==0 and not read('native-lifecycle.json')['forced']
assert all(not r['live_members'] for r in read('native-lifecycle.json')['groups'])
assert read('context-receipt.json')['recorded_pose_stamp_ns']==36_000_000_000
assert read('context-receipt.json')['exit']==0 and not read('context-receipt.json')['forced']
for name,total,outside in (('conditional-fit.json',890,12),('conditional-fit-full-history.json',1133,17)):
    fitted=read(name)
    assert fitted['truth_is_estimator_input'] is False
    assert fitted['source_points']==fitted['fitted']==len(fitted['rows'])==96
    assert sum(row['observations'] for row in fitted['rows'])==total
    assert sum(row['outside_vertical_fan'] for row in fitted['rows'])==outside
    assert all(row['optimistic_three_sigma_major_m']>.1 for row in fitted['rows'])
    assert 'no calibrated confidence claim' in fitted['scope']
ranges=read('dvl-minimum-range-assessment.json')
assert ranges['recorded_samples']==len(ranges['rows'])==66
assert ranges['truth_is_estimator_input'] is False and ranges['native_fusion_range_used'] is False
assert ranges['per_beam_measurements_retained'] is False
assert [(r['stamp_ns'],r['minimum_slant_range_m']) for r in ranges['rows']]==[(r['stamp_ns'],r['range_m']) for r in timeline['raw_dvl']]
example=read('bridge-schema-example.json')['bottom_ranges']
assert all(b['locked'] and b['variance_m2'] is None for b in example['beams'])
assert example['minimum_slant_range_m']==min(b['range_m'] for b in example['beams'])
geometry=read('range-calibration-example.json')
assert geometry['range_variance_m2'] is None
assert all(abs(sum(v*v for v in b['axis'])-1)<1e-12 for b in geometry['beams'])
print('PASS: original raw input and complete failed hypothesis comparisons; mapping/package acceptance remains open')
