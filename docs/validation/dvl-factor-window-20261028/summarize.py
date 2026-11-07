"""Read-only verification of complete DVL state-window and recovery artifacts."""
from pathlib import Path
import gzip,hashlib,json,math,struct
ROOT=Path(__file__).resolve().parent
PROFILES=[('dvl-default-no-ping-factors','lock',False,True),
 ('dvl-default-joint-ping','lock',True,True),
 ('dvl-default-transport-no-ping-factors','transport',False,True),
 ('dvl-default-transport-joint-ping','transport',True,True),
 ('joint-ping-default-state','pressure',True,False),
 ('receive-only-ping-default-state','pressure',False,False),
 ('no-ping-default-state','pressure',False,False)]

def raw(name):
 p=(ROOT/name).resolve()
 if not p.is_relative_to(ROOT):raise ValueError('external evidence path')
 data=p.read_bytes();return gzip.decompress(data) if p.suffix=='.gz' else data

def read(name):return json.loads(raw(name))

def ply(name):
 head,body=raw(name).split(b'end_header\n',1);lines=head.decode().splitlines()
 count=next(int(l.split()[-1]) for l in lines if l.startswith('element vertex '))
 assert count>0 and len(body)==count*12 and 'format binary_little_endian 1.0' in lines
 assert all(math.isfinite(v) for xyz in struct.iter_unpack('<fff',body) for v in xyz)
 return count,lines

def verify():
 for name,h in read('manifest.json')['sha256'].items():assert hashlib.sha256((ROOT/name).read_bytes()).hexdigest()==h,name
 control=read('native-control/input-identity.json');native=read('native-control/results.json')
 assert control['truth_input'] is False and control['event_count']==287
 assert [r['recent_states'] for r in native]==[16,3,3,16]
 config=None
 for i,r in enumerate(native):
  folder=f"native-control/{i:02d}-window-{r['recent_states']:02d}"
  assert r['passed'] and r['threads_stopped'] and r['returncode'] in (0,-15)
  assert r['event_count']==control['event_count'] and r['ready']['max_recent_states']==r['recent_states']
  assert ply(folder+'/worker-map.ply')[0]==r['export']['points']
  cal=read(folder+'/calibration.json');assert cal.pop('max_recent_states')==r['recent_states']
  if config is None:config=cal
  assert cal==config
  events=read(folder+'/events.json.gz');states=[e for e in events if e['type']=='state']
  assert len(states)==control['event_count'] and states[-1]['stamp_ns']==control['last_stamp_ns']
  latencies=read(folder+'/latencies.json.gz');assert len(latencies)==len(states)
  assert all(a['stamp_ns']<b['stamp_ns'] for a,b in zip(states,states[1:]))
 assert read('native-camera-dvl.json')['subcases']==4
 rows=read('map-summary.json');assert len(rows)==2*len(PROFILES)
 source=None;runtime=None;inputs=None;totals=[];coverage=[]
 for name,loss,ping,dvl in PROFILES:
  proof=read(name+'/receipt.json.gz');assert proof['passed'] and proof.get('error') is None
  assert proof['frozen_sources_unchanged'] and proof['worker_sha256']==control['binary_sha256']
  assert proof['joint_ping'] is ping and proof['joint_dvl'] is dvl and proof['joint_pressure'] is True
  assert proof['max_image_rate_hz']==10. and proof['replay_speed']==1.
  if source is None:source=proof['component_source_files'];runtime=proof['runtime_source_files'];inputs=proof['input_sha256']
  assert proof['component_source_files']==source and proof['runtime_source_files']==runtime and proof['input_sha256']==inputs
  recovery=proof['recovery'];assert proof['stale_visual_export_rejected']=='vio_map_unavailable_or_busy'
  expected=[] if loss=='lock' else ['dvl_stale' if loss=='transport' else 'pressure_stale']
  assert [f['reason'] for f in proof['failures']]==expected
  if loss=='lock':
   assert proof['bottom_lock_loss']['statuses']==3 and proof['bottom_lock_loss']['retained_visual_landmarks']>0
   assert proof['stale_visual_map_cleared'] is False and recovery['empty_archive_before_resume'] is False
   assert recovery['new_frame']==recovery['old_frame']
  else:
   assert proof['stale_visual_map_cleared'] and recovery['empty_archive_before_resume']
   assert recovery['new_frame']!=recovery['old_frame']
  if dvl:assert proof['initial_dvl_fusion']['optimized_states']>0 and proof['dvl_fusion']['optimized_states']>0
  assert (proof['ping_fusion']['optimized_states']>0) is ping
  ping_input=name!='no-ping-default-state'
  assert proof['ping_fusion']['input_enabled'] is ping_input
  assert (proof['ping_fusion']['received']>0) is ping_input
  assert proof['pressure_fusion']['optimized_states']>0 and recovery['pressure_optimized_states']>0
  assert all(w['threads_stopped'] and w['returncode'] in (0,-15) for w in proof['cleanup'])
  writer=proof['status_file_cleanup'];assert writer['running'] is False and writer['pending'] is False and writer['error'] is None
  assert writer['written']+writer['coalesced']==writer['submitted'] and proof['input_delivery']['rejected']==0
  for phase in ('initial','recovered'):
   prefix=name+'/'+phase;count,head=ply(prefix+'/map.ply');export=proof['visual_export'] if phase=='initial' else recovery['export']
   assert count==export['points'] and 'comment frame_id '+export['frame'] in head
   assert 'comment session_nonce '+export['session_nonce'] in head
   assessment=read(prefix+'/assessment.json.gz');points=assessment['per_point']
   assert assessment['points']==count==len(points)
   assert sorted(p['export_vertex_index'] for p in points)==list(range(count))
   errors=[p['distance_m'] for p in points];assert all(math.isfinite(v) and v>=0 for v in errors)
   summary=next(r for r in rows if r['profile']==name and r['phase']==phase)
   assert summary['points']==count and summary['above_10cm']==sum(e>.1 for e in errors)
   assert math.isclose(summary['rmse_m'],math.sqrt(math.fsum(e*e for e in errors)/count),abs_tol=1e-12)
   assert math.isclose(summary['max_m'],max(errors),abs_tol=1e-12)
   comparison=read(prefix+'/comparison.json');assert comparison['coordinates_filtered'] is False and comparison['total_points']==count
   assert comparison['marker_area_pt2']==64 and comparison['height_limits_m']==[-4.,-1.5]
   assert comparison['metrics']['evaluation_alignment']['scale']==1.
   totals.append(count)
   if loss=='lock' and phase=='recovered':
    names=['acoustic_validation_target_'+str(i) for i in range(3)]+['acoustic_validation_wall_'+str(i) for i in range(2)]
    coverage.append(dict(profile=name,within_10cm={surface:sum(p['nearest_surface']==surface and p['distance_m']<=.1 for p in points) for surface in names}))
 print(json.dumps(dict(passed=True,ros_maps=len(totals),all_emitted_vertices_assessed=sum(totals),native_window_controls=4,source_bound_profiles=len(PROFILES),recovered_object_surface_support=coverage,installed_packages_qualified=False,initialization_uncertainty_qualified=False)))

if __name__=='__main__':verify()
