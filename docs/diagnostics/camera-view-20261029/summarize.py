"""Verify retained full maps, synchronized views and IMU-basis controls.

This checks retained arithmetic/identity, not simulator reruns or surface truth.
"""
from pathlib import Path
import csv,gzip,hashlib,json,math,struct
BASE=Path(__file__).resolve().parent

def raw(name):
    p=BASE/name;data=p.read_bytes()
    return gzip.decompress(data) if p.suffix=='.gz' else data
def read(name):return json.loads(raw(name))
def matrix(q):
    x,y,z,w=q
    return [[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
            [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
            [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]]
def mul(a,b):return [[sum(a[i][k]*b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]
def transpose(a):return list(map(list,zip(*a)))
def vector(a,v):return [sum(a[i][j]*v[j] for j in range(3)) for i in range(3)]

def main():
    manifest=read('manifest.json')['sha256']
    actual={p.relative_to(BASE).as_posix() for p in BASE.rglob('*') if p.is_file() and p.name!='manifest.json' and '__pycache__' not in p.parts}
    assert actual==set(manifest)
    for name,digest in manifest.items():assert hashlib.sha256((BASE/name).read_bytes()).hexdigest()==digest,name
    shallow=raw('downward-shallow/geographic-route.json')
    assert shallow==raw('forward-shallow/geographic-route.json')
    maps=[]
    for run,camera in [('downward-shallow','camera_down'),('forward-shallow','camera'),('forward-inspection','camera')]:
        prefix=run+'/';result=read(prefix+'result.json');model=read(prefix+'sensor-summary.json')
        assert not model['ping_link_present'] and not model['ping_joint_present'] and not model['autopilot_sonar_inputs']
        assert not any('ping' in (s['name']+' '+(s['topic'] or '')).lower() for s in model['sensors'])
        assert not any(t.startswith('/vehicle/sensors/ping_down') or t=='/model/bluerov2/slam/ping' for t in read(prefix+'sensor-topics.json'))
        assert read(prefix+'session.json')['sensor_configuration']['sensors']==[camera,'dvl']
        lifecycle=read(prefix+'lifecycle.json');assert lifecycle['exit_code']==0 and not lifecycle['forced_stop']
        assert all(not g['live_members'] for g in read(prefix+'owned-process-check.json'))
        assert not read(prefix+'shutdown.json')['escalation_logged']
        phases=['initial','recovered'] if result['passed'] else ['initial']
        if result['passed']:
            assert not result['ping_input'] and result['dvl_fused'] and result['pressure_fused'] and result['all_map_points_retained']
            args=read(prefix+'worker-command.json')['arguments']
            assert '--joint-dvl' in args and '--joint-pressure' in args and '--joint-ping' not in args
            assert args[args.index('--camera-topic')+1]=='/vehicle/sensors/'+camera
        else:
            limits=read(prefix+'evidence-limits.json');assert limits['mission_completed'] and not limits['complete_export_retained']
            assert not limits['final_operating_status_retained'] and not limits['export_http_status_retained']
            assert not (BASE/prefix/'initial/map.ply').exists()
        frames=[]
        for phase in phases:
            folder=prefix+phase+'/';capture=read(folder+'capture.json')
            states=list(csv.DictReader(raw(folder+'evaluation-states.csv.gz').decode().splitlines()))
            stamps=[int(s['stamp_ns']) for s in states]
            photo=capture['mapping_stamp_ns'];low,high=capture['truth_bracket_ns']
            assert capture['mapping_topic']=='/vehicle/sensors/'+camera and capture['mapping_frame_id']=='vehicle/'+camera
            assert photo==capture['reference_stamp_ns']==capture['vehicle_truth'][0]
            assert stamps[0]<=photo<=stamps[-1] and low<=photo<=high and high-low<=50_000_000
            if not result['passed']:continue
            export=read(folder+'export.json');frames.append(export['frame'])
            header,body=raw(folder+'map.ply').split(b'end_header\n',1);header=header.decode().splitlines();count=export['points']
            assert 'format binary_little_endian 1.0' in header and 'element vertex '+str(count) in header and len(body)==12*count
            assert 'comment frame_id '+export['frame'] in header and stamps[-1]<=export['stamp_ns']
            points=list(struct.iter_unpack('<fff',body));assert all(math.isfinite(v) for p in points for v in p)
            assessment=read(folder+'assessment.json.gz');vertices=assessment['per_point'];assert len(vertices)==assessment['points']==count
            assert [v['export_vertex_index'] for v in vertices]==list(range(count))
            distances=[v['distance_m'] for v in vertices]
            assert all(math.isfinite(v) and v>=0 for v in distances)
            assert math.isclose(assessment['rmse_m'],math.sqrt(sum(d*d for d in distances)/count),rel_tol=1e-12)
            assert assessment['max_m']==max(distances) and assessment['above_10cm']==sum(d>.1 for d in distances)
            status=read(folder+'final-status.json');assert status['ready'] and status['frame']==export['frame']==capture['frame']
            assert [f['reason'] for f in status['failures']]==([] if phase=='initial' else ['dvl_stale'])
            assert not status['ping']['enabled'] and status['ping']['received']==status['ping']['fused_states']==0
            assert status['dvl']['fused_states']>0 and status['pressure']['fused_states']>0
            comparison=read(folder+'comparison.json');alignment=comparison['metrics']['evaluation_alignment']
            assert comparison['total_points']==count and not comparison['coordinates_filtered'] and alignment['scale']==1
            assert comparison['marker_area_pt2']==64 and comparison['height_limits_m']==[-4.,-1.5]
            for point,vertex in zip(points,vertices):
                transformed=[a+b for a,b in zip(vector(alignment['rotation'],point),alignment['translation'])]
                assert math.dist(transformed,vertex['world_point'])<1e-10
            maps.append(dict(run=run,phase=phase,points=count,surface_rmse_m=assessment['rmse_m'],max_m=assessment['max_m'],above_10cm=assessment['above_10cm']))
        if result['passed']:assert len(set(frames))==2
    basis=read('basis-control/summary.json');assert basis['passed'] and len(basis['reports'])==6
    references={}
    for row in basis['reports']:
        folder='basis-control/'+row['case']+'-'+str(row['recent_states'])+'/'
        states=read(folder+'states.json.gz');B=matrix(read(folder+'calibration.json')['T_i_c_q_xyzw'])
        C=mul(matrix(states[0]['orientation_xyzw']),B);Ct=transpose(C)
        positions=[vector(Ct,s['position']) for s in states]
        traces=[json.loads(line) for line in raw(folder+'calibration.visual-tracks.jsonl.gz').decode().splitlines()]
        points={(f['stamp_ns'],t['id']):(vector(Ct,t['point']),t.get('joint_position_sigma_m'))
                for f in traces for t in f['tracks'] if t.get('point') is not None}
        ref_positions,ref_points=references.setdefault(row['recent_states'],(positions,points))
        assert len(states)==121 and points.keys()==ref_points.keys() and len(points)==row['triangulated_observations_compared']
        assert max(math.dist(p,q) for p,q in zip(positions,ref_positions))<1e-8
        assert max(math.dist(p,ref_points[k][0]) for k,(p,s) in points.items())<1e-6
        assert max(abs(s-ref_points[k][1]) for k,(p,s) in points.items() if s is not None)<1e-6
        assert not read(folder+'export.json')['success'] and not row['archive_equivalence_tested']
    print(json.dumps(dict(artifacts_verified=True,full_maps=maps,all_assessed_points=sum(m['points'] for m in maps),
        synthetic_basis_cases=6,forward_map_utility_accepted=False,installed_qualified=False,scope=__doc__),indent=2))

if __name__=='__main__':main()
