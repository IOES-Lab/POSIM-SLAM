"""Verify retained full-map/clock/fusion evidence; does not rerun the simulator."""
from pathlib import Path
import csv,gzip,hashlib,json,math,struct
BASE=Path(__file__).resolve().parent

def read(name):
    p=BASE/name;raw=p.read_bytes()
    return json.loads(gzip.decompress(raw) if p.suffix=='.gz' else raw)

def main():
    manifest=read('manifest.json')['sha256']
    actual={p.relative_to(BASE).as_posix() for p in BASE.rglob('*') if p.is_file() and p.name!='manifest.json' and '__pycache__' not in p.parts}
    assert actual==set(manifest)
    for name,digest in manifest.items():assert hashlib.sha256((BASE/name).read_bytes()).hexdigest()==digest,name
    result=read('result.json');assert result['passed'] and not result['ping_input'] and result['dvl_fused'] and result['pressure_fused']
    lifecycle=read('lifecycle.json');assert lifecycle['exit_code']==0 and not lifecycle['forced_stop']
    assert all(not g['live_members'] for g in read('owned-process-check.json'))
    assert not read('shutdown.json')['escalation_logged']
    hydro=read('hydrodynamics.json')['proof'];assert hydro['passed'] and hydro['input_sha256']==read('source-identity.json')['hydrodynamics_input_sha256']
    coverage={row['phase']:row for row in read('coverage.json')['rows']}
    model=read('sensor-summary.json');assert not model['ping_link_present'] and not model['ping_joint_present'] and not model['autopilot_sonar_inputs']
    assert not any('ping' in (sensor['name']+' '+(sensor['topic'] or '')).lower() for sensor in model['sensors'])
    args=read('worker-command.json')['arguments'];assert '--joint-dvl' in args and '--joint-pressure' in args
    assert '--joint-ping' not in args and '--ping-topic' not in args
    assert not any(t.startswith('/vehicle/sensors/ping_down') or t=='/model/bluerov2/slam/ping' for t in read('sensor-topics.json'))
    frames=[];rows=[]
    for phase in ['initial','recovered']:
        prefix=phase+'/';export=read(prefix+'export.json');frames.append(export['frame'])
        raw=(BASE/prefix/'map.ply').read_bytes();header,body=raw.split(b'end_header\n',1)
        count=export['points'];assert ('element vertex '+str(count)) in header.decode().splitlines()
        assert ('comment frame_id '+export['frame']) in header.decode().splitlines()
        assert 'format binary_little_endian 1.0' in header.decode().splitlines() and len(body)==12*count
        assert all(math.isfinite(v) for point in struct.iter_unpack('<fff',body) for v in point)
        assessment=read(prefix+'assessment.json.gz');assert len(assessment['per_point'])==assessment['points']==count
        assert [p['export_vertex_index'] for p in assessment['per_point']]==list(range(count))
        distances=[p['distance_m'] for p in assessment['per_point']];assert all(math.isfinite(v) and v>=0 for v in distances)
        assert math.isclose(assessment['rmse_m'],math.sqrt(sum(v*v for v in distances)/count),rel_tol=1e-12)
        assert math.isclose(assessment['max_m'],max(distances),rel_tol=1e-12)
        assert assessment['above_10cm']==sum(v>.1 for v in distances)
        status=read(prefix+'final-status.json');assert status['ready'] and status['frame']==export['frame']
        assert [f['reason'] for f in status['failures']]==([] if phase=='initial' else ['dvl_stale'])
        assert not status['ping']['enabled'] and status['ping']['received']==status['ping']['fused_states']==0
        assert status['dvl']['enabled'] and status['dvl']['fused_states']>0 and status['pressure']['enabled'] and status['pressure']['fused_states']>0
        capture=read(prefix+'capture.json');comparison=read(prefix+'comparison.json');proof=comparison['reference_provenance']
        stamps=[int(row['stamp_ns']) for row in csv.DictReader(gzip.decompress((BASE/prefix/'evaluation-states.csv.gz').read_bytes()).decode().splitlines())]
        assert capture['frame']==proof['frame']==export['frame'] and proof['reference_kind']=='same_segment_input'
        stamp=capture['mapping_stamp_ns'];assert stamp==capture['reference_stamp_ns']==capture['vehicle_truth'][0]==proof['exposure_ns']
        assert proof['state_interval_ns']==[stamps[0],stamps[-1]] and stamps[0]<=stamp<=stamps[-1]<=export['stamp_ns']
        low,high=capture['truth_bracket_ns'];assert low<=stamp<=high and high-low<=50_000_000
        assert comparison['total_points']==count and not comparison['coordinates_filtered']
        assert comparison['marker_area_pt2']==64 and comparison['height_limits_m']==[-4.,-1.5]
        assert coverage[phase]['capture_within_state_interval'] and coverage[phase]['cloud_vertices']==count
        rows.append(dict(phase=phase,points=count,surface_rmse_m=assessment['rmse_m'],max_m=assessment['max_m'],above_10cm=assessment['above_10cm'],heading_span_deg=coverage[phase]['heading_span_deg']))
    assert frames[0]!=frames[1]
    print(json.dumps(dict(artifacts_verified=True,maps=rows,scope=__doc__,navigation_or_installed_qualified=False),indent=2))

if __name__=='__main__':main()
