"""Verify retained artifacts; does not rerun the simulator or certify navigation."""
from pathlib import Path
import csv,gzip,hashlib,json,math,struct

BASE=Path(__file__).resolve().parent

def read(path):
    p=BASE/path;data=p.read_bytes()
    return json.loads(gzip.decompress(data) if p.suffix=='.gz' else data)

def vertices(path):
    data=(BASE/path).read_bytes();header,body=data.split(b'end_header\n',1)
    lines=header.decode('ascii').splitlines()
    assert 'format binary_little_endian 1.0' in lines
    assert [line for line in lines if line.startswith('property ')]==['property float x','property float y','property float z']
    count=int(next(line.split()[2] for line in lines if line.startswith('element vertex ')))
    assert len(body)==12*count
    assert all(math.isfinite(v) for point in struct.iter_unpack('<fff',body) for v in point)
    return count

def measured(path,count):
    d=read(path);assert d['points']==count
    rows=d['per_point'];assert len(rows)==count
    assert [row['export_vertex_index'] for row in rows]==list(range(count))
    distances=[row['distance_m'] for row in rows]
    assert all(math.isfinite(v) and v>=0 for v in distances)
    return dict(points=count,rmse_m=math.sqrt(sum(v*v for v in distances)/count),
                max_m=max(distances),above_10cm=sum(v>.1 for v in distances))

def close(a,b):assert math.isclose(a,b,rel_tol=1e-10,abs_tol=1e-10),(a,b)

def main():
    manifest=read('manifest.json')['sha256']
    actual={p.relative_to(BASE).as_posix() for p in BASE.rglob('*') if p.is_file() and p.name!='manifest.json' and '__pycache__' not in p.parts}
    assert actual==set(manifest)
    for name,digest in manifest.items():assert hashlib.sha256((BASE/name).read_bytes()).hexdigest()==digest,name
    rows=[];sources=None;inputs=None
    summaries={(r['profile'],r['phase']):r for r in read('ros/map-summary.json')}
    for name in ['dvl-transport','dvl-lock','no-ping','ping-fused','ping-received']:
        prefix='ros/'+name;d=read(prefix+'/receipt.json.gz')
        assert d['passed'] and d['frozen_sources_unchanged'] and d['replay_speed']==1
        assert d['worker_sha256']==read('source-identity.json')['native_binary_sha256']
        assert d['joint_pressure'] and not d['joint_sonar']
        assert d['processing_profile']=={'optical_flow_type':'patch','max_image_pixels':120000,'max_feature_cells':200}
        assert d['max_image_rate_hz']==10
        if sources is None:sources=d['runtime_source_files'];inputs=d['input_sha256']
        assert d['runtime_source_files']==sources and d['input_sha256']==inputs
        assert all(c['threads_stopped'] and c['returncode'] is not None for c in d['cleanup'])
        assert not d['status_file_cleanup']['running'] and not d['status_file_cleanup']['pending']
        assert d['input_delivery']['fault'] is None
        recovery=d['recovery'];same=recovery['old_frame']==recovery['new_frame']
        assert same==(name=='dvl-lock')
        assert [f['reason'] for f in d['failures']]==([] if name=='dvl-lock' else ['dvl_stale' if name=='dvl-transport' else 'pressure_stale'])
        if name.startswith('dvl-'):
            assert d['joint_dvl'] and not d['joint_ping']
            assert d['ping_fusion']=={'input_enabled':False,'joint_enabled':False,'received':0,'optimized_states':0}
            assert all('ping_down' not in t and not t.endswith('/ping') for t in d['subscribed_topics'])
        else:assert not d['joint_dvl']
        for phase,export in [('initial',d['visual_export']),('recovered',recovery['export'])]:
            p=prefix+'/'+phase;count=vertices(p+'/map.ply');assert count==export['points']
            a=measured(p+'/assessment.json.gz',count);c=read(p+'/comparison.json');s=summaries[name,phase]
            assert c['total_points']==count and not c['coordinates_filtered'] and c['marker_area_pt2']==64
            assert c['height_limits_m']==[-4,-1.5]
            close(a['rmse_m'],s['surface_rmse_m']);close(a['max_m'],s['max_surface_m']);assert a['above_10cm']==s['above_10cm']
            rows.append(dict(case=prefix,phase=phase,**a))
    native_points=0;native_assessed=0
    for name in ['native-first','native-second']:
        result=read(name+'/result.json');assert result['passed'] and not result['ping_input'] and result['dvl_fused'] and result['pressure_fused']
        model=read(name+'/sensor-summary.json');assert not model['ping_link_present'] and not model['ping_joint_present'] and not model['autopilot_sonar_inputs']
        assert not any('ping' in s['name'].lower() or 'ping' in (s['topic'] or '').lower() for s in model['sensors'])
        assert not any(t.startswith('/vehicle/sensors/ping_down') or t=='/model/bluerov2/slam/ping' for t in read(name+'/sensor-topics.json'))
        if name=='native-second':
            args=read(name+'/worker-command.json')['arguments'];assert '--joint-dvl' in args and '--joint-pressure' in args
            assert '--joint-ping' not in args and '--ping-topic' not in args
        assert all(not g['live_members'] for g in read(name+'/owned-process-check.json'))
        assert read(name+'/shutdown-diagnostic.json')['dvl_child_forced_kill']
        for phase in ['initial','recovered']:
            p=name+'/'+phase;count=vertices(p+'/map.ply');assert count==read(p+'/export.json')['points'];native_points+=count
            state=read(p+'/final-status.json');assert [f['reason'] for f in state['failures']]==([] if phase=='initial' else ['dvl_stale'])
            assert state['ping']['received']==state['ping']['fused_states']==0 and not state['ping']['enabled']
            assert state['dvl']['fused_states']>0 and state['pressure']['fused_states']>0
            if name=='native-first' and phase=='initial':
                assert not read(name+'/assessment-limit.json')['all_point_assessment_complete'];continue
            a=measured(p+'/assessment.json.gz',count);d=read(p+'/assessment.json.gz');native_assessed+=count
            close(a['rmse_m'],d['rmse_m']);close(a['max_m'],d['max_m']);assert a['above_10cm']==d['above_10cm']
            c=read(p+'/comparison.json');assert c['total_points']==count and not c['coordinates_filtered'] and c['marker_area_pt2']==64
            assert c['height_limits_m']==[-4,-1.5]
            capture=c['reference_capture'];assert capture['reference_stamp_ns']==capture['mapping_stamp_ns']
            stamps=[int(row['stamp_ns']) for row in csv.DictReader(gzip.decompress((BASE/p/'evaluation-states.csv.gz').read_bytes()).decode().splitlines())]
            reference=c['reference_provenance'];assert reference['reference_kind']=='same_recording_scene_reference'
            assert reference['state_interval_ns']==[stamps[0],stamps[-1]]
            assert reference['exposure_ns']==capture['mapping_stamp_ns']
            assert reference['within_exported_segment']==(stamps[0]<=reference['exposure_ns']<=stamps[-1])
            rows.append(dict(case=name,phase=phase,**a))
    assert sum(r['points'] for r in rows if r['case'].startswith('ros/'))==14038
    assert native_points==7430 and native_assessed==5533
    assert not read('native-second/lifecycle.json')['forced_stop']
    assert read('native-first/lifecycle.json')['forced_stop']
    controls=[read('shutdown-controls/'+name+'.json') for name in ['signal-controls','signal-pause-controls','signal-native-env-controls','signal-active-controls']]
    assert all(not row['forced'] and row['exit_code']==0 for group in controls for row in group)
    unpatched=read('shutdown-followup/heartbeat-unpatched.json');corrected=read('shutdown-followup/heartbeat-corrected.json')
    assert not unpatched['passed'] and unpatched['watchdog_forced']
    assert corrected['passed'] and not corrected['watchdog_forced'] and corrected['heartbeat_ticks']==3 and corrected['child_returncode']==0
    assert all(row['exit_code']==0 and not row['parent_forced'] and not row['child_sigkill'] for row in read('shutdown-followup/launch-corrected.json'))
    assert sum(row['parent_forced'] for row in read('shutdown-followup/launch-unpatched.json'))==3
    phase=(BASE/'shutdown-followup/ros-only-paused.phases.jsonl').read_text();assert '"main-exit"' in phase and '"atexit"' in phase
    assert ' Z ' in (BASE/'shutdown-followup/ros-only-paused.ps.txt').read_text()
    native=read('shutdown-followup/native-corrected/result.json');assert native['passed'] and native['ping_physically_absent'] and native['valid_dvl_messages']>=3 and native['max_receipt_gap_s']>=4
    lifecycle=read('shutdown-followup/native-corrected/lifecycle.json');assert lifecycle['exit_code']==0 and not lifecycle['forced_stop']
    assert all(not g['live_members'] for g in read('shutdown-followup/native-corrected/owned-process-check.json'))
    shutdown=read('shutdown-followup/native-corrected/shutdown.json');assert not shutdown['sigkill_logged'] and not shutdown['termination_deadline_logged']
    assert any('process has finished cleanly' in line for line in shutdown['dvl_process_lines'])
    print(json.dumps(dict(artifact_checks_passed=True,ros_maps=10,ros_vertices=14038,native_maps=4,native_vertices=native_points,
                         assessed_native_vertices=native_assessed,development_child_wait_fix_passed=True,installed_qualification=False,scope=__doc__),indent=2))

if __name__=='__main__':main()
