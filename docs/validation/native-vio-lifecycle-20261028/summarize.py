"""Verify complete native lifecycle evidence with only Python's standard library."""
from pathlib import Path
import csv,gzip,hashlib,io,json,math,struct

ROOT=Path(__file__).resolve().parent

def raw(name):
 path=(ROOT/name).resolve()
 if not path.is_relative_to(ROOT):raise ValueError('external evidence path')
 data=path.read_bytes()
 return gzip.decompress(data) if path.suffix=='.gz' else data

def read(name):return json.loads(raw(name))

def ply(name,nonce,frame=None):
 head,body=raw(name).split(b'end_header\n',1);lines=head.decode().splitlines()
 assert 'format binary_little_endian 1.0' in lines
 count=next(int(line.split()[-1]) for line in lines if line.startswith('element vertex '))
 assert count>0 and len(body)==12*count
 assert 'comment session_nonce '+nonce in lines
 if frame:assert 'comment frame_id '+frame in lines
 assert all(math.isfinite(v) for row in struct.iter_unpack('<fff',body) for v in row)
 return count

def stopped(run):
 assert read(run+'/lifecycle.json')==dict(exit_code=0,forced_stop=False,error=None)
 assert all(not row['live_members'] for row in read(run+'/owned-process-check.json'))
 workers=read(run+'/worker-cleanup.json')
 assert workers and all(w['threads_stopped'] and w['returncode'] in (0,-15) for w in workers)
 probe=read(run+'/probe-result.json')
 assert probe['writer_stopped'] and probe['pending']==0 and not probe['faults']
 rows=list(csv.DictReader(io.StringIO(raw(run+'/direct-receipts.csv.gz').decode())))
 assert len(rows)==probe['written']==sum(probe['counts'].values())
 assert probe['counts']['local_pose']>0 and probe['counts']['mavlink_32']>0
 analysis=read(run+'/navigation-analysis.json')
 assert analysis['direct_counts']==probe['counts']
 assert analysis['max_active_api_pose_age_s']<read('source-identity.json')['unchanged_navigation_freshness_s']

def verify():
 for name,expected in read('manifest.json')['sha256'].items():
  assert hashlib.sha256((ROOT/name).read_bytes()).hexdigest()==expected,name
 identity=read('source-identity.json')
 assert identity['estimator_truth_input'] is False and identity['development_overlay'] is True
 assert identity['distribution_qualified'] is False and identity['unchanged_navigation_freshness_s']==1.
 run='nav-06';trial=read(run+'/trial-result.json');status=read(run+'/final-status.json.gz')
 assert trial['passed'] and trial['error'] is None and trial['operating_s']>=1200
 assert trial['completed_missions']==11 and trial['exports']==12
 assert not trial['failures'] and not status['failures'] and status['segment']==0
 assert status['ping']['fused_states']>0 and status['pressure']['fused_states']>0
 maps=sorted((ROOT/run).glob('*.ply'));assert len(maps)==trial['exports']
 for p in maps:ply(run+'/'+p.name,trial['session_nonce'],status['frame'])
 assert ply(run+'/mission-11-map.ply',trial['session_nonce'])==status['landmarks']==trial['points']==6312
 for i in range(1,12):
  mission=read(run+f'/mission-{i:02d}-final.json.gz')
  assert mission['status']=='completed' and not mission['active'] and mission['error'] is None
 stopped(run)
 mode=read('modes-07/trial-result.json');assert mode['passed'] and mode['error'] is None
 assert mode['completed_round_trips']==3 and mode['completed_phases']==7
 frames=[];counts=[]
 for i in range(7):
  prefix=f'modes-07/phase-{i:02d}';result=read(prefix+'/result.json');state=read(prefix+'/final-status.json.gz')
  joint=i%2==0
  assert result['passed'] and result['joint_ping'] is joint and result['segment']==i
  assert not result['failures'] and not state['failures'] and state['segment']==i
  assert state['ping']['joint_enabled'] is joint and state['ping']['received']>0
  assert (state['ping']['fused_states']>0) is joint
  assert state['pressure']['fused_states']>0 and result['first_seed_s']>0
  mission=read(prefix+'/mission-final.json.gz');assert mission['status']=='completed' and mission['error'] is None
  frames.append(state['frame']);counts.append(ply(prefix+'/map.ply',mode['session_nonce'],state['frame']))
  if i:
   gate=read(prefix+'/switch-gate.json.gz')
   assert gate['passed'] and gate['export']['http']==409 and gate['after']['landmarks']==0
   assert gate['before']['segment']==i-1 and gate['after']['segment']==i
   assert gate['before']['frame']!=gate['after']['frame']
 assert len(set(frames))==7
 stopped('modes-07')
 assert len(read('modes-07/worker-cleanup.json'))==7
 failure=read('dvl-first-failure/receipt.json.gz')
 assert failure['passed'] is False and not failure.get('visual_export')
 assert len(failure['failures'])==3 and all(f['reason']=='sensor_events_backlog' for f in failure['failures'])
 assert all(w['threads_stopped'] and w['returncode'] in (0,-15) for w in failure['cleanup'])
 writer=failure['status_file_cleanup']
 assert writer['running'] is False and writer['pending'] is False and writer['error'] is None
 assert writer['written']+writer['coalesced']==writer['submitted']
 print(json.dumps(dict(passed=True,nominal_operating_minutes=trial['operating_s']/60,completed_long_surveys=11,retained_long_maps=len(maps),final_long_points=trial['points'],mode_round_trips=3,mode_map_points=counts,dvl_loss_recovery_passed=False,old_navigation_failure_fixed=False,geometry_qualified=False,installed_packages_qualified=False)))

if __name__=='__main__':verify()
