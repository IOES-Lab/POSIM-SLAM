"""Verify the retained public subset; no raw replay or scene-truth reconstruction."""
from pathlib import Path
import ast,csv,gzip,hashlib,json,math,statistics,struct,zipfile
BASE=Path(__file__).resolve().parent
def raw(name):
    p=BASE/name;data=p.read_bytes();return gzip.decompress(data) if p.suffix=='.gz' else data
def read(name):return json.loads(raw(name))
def npy(z,name):
    data=z.read(name+'.npy');assert data[:6]==b'\x93NUMPY' and data[6:8]==b'\x01\x00'
    size=struct.unpack('<H',data[8:10])[0];header=ast.literal_eval(data[10:10+size].decode())
    assert not header['fortran_order'];return header,data[10+size:]

def main():
    hashes=read('manifest.json')['sha256'];actual={p.relative_to(BASE).as_posix() for p in BASE.rglob('*') if p.is_file() and p.name!='manifest.json' and '__pycache__' not in p.parts}
    assert actual==set(hashes)
    for name,digest in hashes.items():assert hashlib.sha256((BASE/name).read_bytes()).hexdigest()==digest,name
    record=read('input-record-receipt.json');assert not record['passed']
    assert [e['error'] for e in record['errors']]==['bounded_recording_timeout','incomplete_camera_interval','noncontiguous_camera_clock']
    assert record['frames']==301 and record['interval_ns']==[40_000_000,60_000_000_000]
    audit=read('input-window-audit.json');assert audit['replay_inputs_covered'] and not audit['whole_recording_harness_passed']
    assert audit['consumed_camera_frames']==294 and audit['consumed_camera_interval_ns']==[1_400_000_000,60_000_000_000]
    assert audit['max_imu_gap_ns']<=20_000_000 and audit['bytes']<audit['budget_bytes']==128*1024**2
    assert audit['dvl']['gyro_model_sigma_rad_s']==.01
    frame=read('matched-input-frame.json');capture=read('initial/capture.json')
    assert frame['stamp_ns']==capture['mapping_stamp_ns']==capture['reference_stamp_ns']==capture['vehicle_truth'][0]
    assert hashlib.sha256(raw('matched-input-gray8.npz')).hexdigest()==record['input_sha256'][frame['file']]
    with zipfile.ZipFile(BASE/'matched-input-gray8.npz') as z:
        header,pixels=npy(z,'data');assert header['descr']=='|u1' and len(pixels)==640*480
        header,stamp=npy(z,'stamp_ns');assert header['descr']=='<i8' and struct.unpack('<q',stamp)[0]==frame['stamp_ns']
        assert hashlib.sha256(pixels).hexdigest()==frame['input_gray8_sha256']
    model=read('sensor-summary.json');assert not model['ping_link_present'] and not model['ping_joint_present'] and not model['autopilot_sonar_inputs']
    assert not any('ping' in (s['name']+' '+(s['topic'] or '')).lower() for s in model['sensors'])
    assert read('lifecycle.json')['exit_code']==0 and not read('lifecycle.json')['forced_stop']
    assert all(not x['live_members'] for x in read('owned-process-check.json'))
    exported=read('initial/export.json');status=read('initial/final-status.json');assert exported['points']==48
    assert status['ready'] and status['frame']==exported['frame']==capture['frame'] and status['failures']==[]
    assert status['dvl']['fused_states']>0 and status['pressure']['fused_states']>0 and status['ping']['received']==status['ping']['fused_states']==0
    stamps=[int(r['stamp_ns']) for r in csv.DictReader(raw('initial/evaluation-states.csv.gz').decode().splitlines())]
    assert stamps[0]<=capture['mapping_stamp_ns']<=stamps[-1]<=exported['stamp_ns']
    header,payload=raw('initial/map.ply').split(b'end_header\n',1);assert len(payload)==48*12 and b'element vertex 48\n' in header
    points=list(struct.iter_unpack('<fff',payload));assert all(math.isfinite(x) for p in points for x in p)
    assessment=read('initial/assessment.json.gz');vertices=assessment['per_point'];assert len(vertices)==48 and [v['export_vertex_index'] for v in vertices]==list(range(48))
    errors=[v['distance_m'] for v in vertices]
    assert math.isclose(assessment['rmse_m'],math.sqrt(sum(e*e for e in errors)/48),rel_tol=1e-12)
    assert assessment['max_m']==max(errors) and assessment['above_10cm']==sum(e>.1 for e in errors)==3
    examples=read('feature-examples.json.gz');assert len(examples)==278 and [examples[0]['stamp_ns'],examples[-1]['stamp_ns']]==[4_600_000_000,60_000_000_000]
    for row in read('feature-stages.json')['rows']:
        tracks=[t for f in examples for t in f['tracks'] if t['category']==row['category']]
        assert len(tracks)==row['observations'] and len({t['id'] for t in tracks})==row['unique_ids']
        assert len({t['id'] for t in tracks if t['triangulated']})==row['triangulated_ids']
        assert len({t['id'] for t in tracks if t['archived']})==row['archived_ids']
    assert sum(r['observations'] for r in read('feature-stages.json')['rows'])==61_433
    native=read('native/segment-0-calibration.json');assert native['gravity_m_s2']==9.80665
    assert read('source-identity.json')['world_gravity_m_s2']==9.8
    for name in ['dvl-initializer-control.json','dvl-initializer-world-gravity.json']:
        d=read(name);assert not d['native_vio_handoff'] and not d['map_qualified']
        assert d['first_tracking']['state']=='tracking' and not d['first_tracking']['navigation_ready']
        assert d['first_tracking']['initialization']['fixed_bias_directions']==6
        assert d['first_tracking_assessment']['velocity_error_m_s']>d['first_tracking']['initialization']['velocity_three_sigma_m_s']
    print(json.dumps(dict(public_subset_verified=True,full_vertices=48,above_10cm=3,raw_record_included=False,
        whole_recorder_passed=False,replay_window_audited=True,forward_map_accepted=False,scope=__doc__),indent=2))

if __name__=='__main__':main()
