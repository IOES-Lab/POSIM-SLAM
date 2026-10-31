"""Check retained complete exports and actual gravity transport; no simulator rerun."""
from pathlib import Path
import csv,gzip,hashlib,json,math,struct,xml.etree.ElementTree as ET
BASE=Path(__file__).resolve().parent
def raw(name):
    p=BASE/name;data=p.read_bytes();return gzip.decompress(data) if p.suffix=='.gz' else data
def read(name):return json.loads(raw(name))
def points(name,export):
    header,body=raw(name).split(b'end_header\n',1);lines=header.decode().splitlines()
    assert 'format binary_little_endian 1.0' in lines and 'element vertex '+str(export['points']) in lines
    assert 'comment frame_id '+export['frame'] in lines and len(body)==12*export['points']
    result=list(struct.iter_unpack('<fff',body));assert all(math.isfinite(v) for p in result for v in p)
    return result

def main():
    hashes=read('manifest.json')['sha256'];actual={p.relative_to(BASE).as_posix() for p in BASE.rglob('*') if p.is_file() and p.name!='manifest.json' and '__pycache__' not in p.parts}
    assert actual==set(hashes)
    for name,digest in hashes.items():assert hashlib.sha256((BASE/name).read_bytes()).hexdigest()==digest,name
    summary=read('ros-map-summary.json');assert len(summary)==10 and sum(r['points'] for r in summary)==14_116
    profiles=['dvl-transport','dvl-lock','no-ping','ping-fused','ping-received'];ros_points=0
    for profile in profiles:
        prefix='ros/'+profile+'/';receipt=read(prefix+'receipt.json');assert receipt['passed'] and receipt['gravity_m_s2']==9.8 and receipt['frozen_sources_unchanged']
        assert all(w['threads_stopped'] and w['returncode'] in (0,-15) for w in receipt['cleanup'])
        assert read(prefix+'segment-0-calibration.json')['gravity_m_s2']==9.8
        if (BASE/prefix/'segment-1-calibration.json').is_file():assert read(prefix+'segment-1-calibration.json')['gravity_m_s2']==9.8
        if profile.startswith('dvl'):assert receipt['joint_dvl'] and not receipt['ping_fusion']['input_enabled']
        if profile=='dvl-lock':assert receipt['dvl_loss_status']
        for phase,file,suffix in [('initial','initial-map.ply',''),('recovered','map.ply','-recovered')]:
            export=receipt['visual_export'] if phase=='initial' else receipt['recovery']['export']
            vertices=points(prefix+file,export);assessment=read(prefix+'visual-surface-evaluation'+suffix+'.json.gz')
            assert len(vertices)==assessment['points']==len(assessment['per_point'])
            assert [v['export_vertex_index'] for v in assessment['per_point']]==list(range(len(vertices)))
            errors=[v['distance_m'] for v in assessment['per_point']];row=next(r for r in summary if r['profile']==profile and r['phase']==phase)
            assert row['points']==len(vertices) and not row['coordinates_filtered']
            assert math.isclose(row['surface_rmse_m'],math.sqrt(math.fsum(e*e for e in errors)/len(vertices)),rel_tol=1e-12)
            assert row['max_surface_m']==max(errors) and row['above_10cm']==sum(e>.1 for e in errors)
            ros_points+=len(vertices)
    descriptor=read('provider-matrix.json')['validation']['live_vio_profiles'];assert len(descriptor)==4 and all(p['gravity_m_s2']==9.8 for p in descriptor)
    prefix='native/';chain=read(prefix+'gravity-chain.json');assert chain['passed'] and chain['world_gravity_vector_m_s2']==[0,0,-9.8]
    assert ET.parse(BASE/prefix/'world.sdf').findtext('world/gravity').split()==['0','0','-9.8']
    assert read(prefix+'common-imu-calibration.json')['gravity_m_s2']==read(prefix+'inertial-dvl-calibration.json')['estimator']['gravity_m_s2']==9.8
    assert chain['native_argument_m_s2']==9.8 and chain['native_segment_gravity_m_s2']==[9.8,9.8]
    for index in [0,1]:assert read(prefix+f'segment-{index}-calibration.json')['gravity_m_s2']==9.8
    model=read(prefix+'sensor-summary.json');assert not model['ping_link_present'] and not model['ping_joint_present'] and not model['autopilot_sonar_inputs']
    assert not any('ping' in (s['name']+' '+(s['topic'] or '')).lower() for s in model['sensors'])
    assert read(prefix+'lifecycle.json')['exit_code']==0 and not read(prefix+'lifecycle.json')['forced_stop']
    assert all(not g['live_members'] for g in read(prefix+'owned-process-check.json')) and not read(prefix+'shutdown.json')['escalation_logged']
    frames=[];native_points=0
    for phase,count in [('initial',1117),('recovered',1086)]:
        folder=prefix+phase+'/';export=read(folder+'export.json');vertices=points(folder+'map.ply',export);assert len(vertices)==count
        frames.append(export['frame']);status=read(folder+'final-status.json');capture=read(folder+'capture.json')
        assert status['ready'] and status['frame']==capture['frame']==export['frame']
        assert [f['reason'] for f in status['failures']]==([] if phase=='initial' else ['dvl_stale'])
        assert status['dvl']['fused_states']>0 and status['pressure']['fused_states']>0 and status['ping']['received']==status['ping']['fused_states']==0
        assessment=read(folder+'assessment.json.gz');assert len(assessment['per_point'])==count
        errors=[v['distance_m'] for v in assessment['per_point']];assert assessment['above_10cm']==sum(e>.1 for e in errors)==0
        assert math.isclose(assessment['rmse_m'],math.sqrt(math.fsum(e*e for e in errors)/count),rel_tol=1e-12) and assessment['max_m']==max(errors)
        stamps=[int(r['stamp_ns']) for r in csv.DictReader(raw(folder+'evaluation-states.csv.gz').decode().splitlines())]
        t=capture['mapping_stamp_ns'];assert stamps[0]<=t==capture['reference_stamp_ns']==capture['vehicle_truth'][0]<=stamps[-1]<=export['stamp_ns']
        comparison=read(folder+'comparison.json');assert comparison['reference_provenance']['reference_kind']=='same_segment_input'
        assert comparison['total_points']==count and not comparison['coordinates_filtered'] and comparison['metrics']['evaluation_alignment']['scale']==1
        assert comparison['marker_area_pt2']==64 and comparison['height_limits_m']==[-4,-1.5]
        native_points+=count
    assert len(set(frames))==2
    print(json.dumps(dict(retained_evidence_verified=True,ros_maps=10,ros_vertices=ros_points,native_maps=2,native_vertices=native_points,
        gravity_m_s2=9.8,installed_qualified=False,obstacle_completeness_qualified=False,scope=__doc__),indent=2))

if __name__=='__main__':main()
