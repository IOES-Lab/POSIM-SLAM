"""Independent NumPy assessment of retained complete wall/corner outputs."""
from pathlib import Path
import gzip,json,xml.etree.ElementTree as ET
import numpy as np
ROOT=Path(__file__).resolve().parent
def load(path):
 data=path.read_bytes();return json.loads(gzip.decompress(data) if path.suffix=='.gz' else data)
def matrix(q):
 x,y,z,w=np.asarray(q)/np.linalg.norm(q)
 return np.array([[1-2*(y*y+z*z),2*(x*y-w*z),2*(x*z+w*y)],
  [2*(x*y+w*z),1-2*(x*x+z*z),2*(y*z-w*x)],[2*(x*z-w*y),2*(y*z+w*x),1-2*(x*x+y*y)]])
def slerp(a,b,u):
 a=np.asarray(a);b=np.asarray(b);dot=a@b
 if dot<0:b=-b;dot=-dot
 if dot>.9995:q=(1-u)*a+u*b;return q/np.linalg.norm(q)
 angle=np.arccos(np.clip(dot,-1,1));return (np.sin((1-u)*angle)*a+np.sin(u*angle)*b)/np.sin(angle)
boxes=[]
for model in ET.parse(ROOT/'physical-scene.sdf').getroot().find('world').findall('model'):
 pose=np.array(list(map(float,model.findtext('pose').split())));assert np.all(pose[3:]==0)
 boxes.append((pose[:3],np.array(list(map(float,model.findtext('.//collision/geometry/box/size').split())))/2))
def distance(points):
 result=[]
 for center,half in boxes:
  delta=np.abs(points-center)-half;outside=np.linalg.norm(np.maximum(delta,0),axis=-1)
  result.append(np.where(np.max(delta,axis=-1)>0,outside,np.min(-delta,axis=-1)))
 return np.min(result,axis=0)
clocks=[];checks=0
for mode in ('navigation','uncalibrated-navigation','uncalibrated-baseline'):
 folder=ROOT/('replay-'+mode);assert load(folder/'receipt.json')['replay_complete']
 rows=np.genfromtxt(ROOT/('aligned-'+mode+'.csv'),delimiter=',',names=True);assert len(rows)==177
 clocks.append(rows['stamp_ns']);xyz=np.column_stack([rows[n] for n in ('x','y','z')]);truth=np.column_stack([rows[n] for n in ('truth_x','truth_y','truth_z')]);errors=np.linalg.norm(xyz-truth,axis=1)
 report=load(ROOT/('assessment-'+mode+'.json'));np.testing.assert_allclose(errors,rows['error_m'],atol=1e-12)
 np.testing.assert_allclose(np.sqrt(np.mean(errors**2)),report['trajectory']['position_rmse_m'],atol=1e-12);checks+=1
assert all(np.array_equal(clocks[0],c) for c in clocks[1:])
report=load(ROOT/'assessment-uncalibrated-baseline.json');world=np.array([p['world'] for p in report['per_point']]);errors=distance(world);assert len(world)==46
np.testing.assert_allclose(errors,[p['error_m'] for p in report['per_point']],atol=2e-12)
assert int((errors>.1).sum())==23;checks+=1
truth=np.array(load(ROOT/'truth-assessment-only.json.gz'));cal=load(ROOT/'physical-calibration.json')['sonar']['sonar_mount'];mount=matrix(cal['rotation']);lever=np.array(cal['translation'])
def pose(stamp):
 j=int(np.searchsorted(truth[:,0],stamp));assert 0<j<len(truth) and truth[j,0]-truth[j-1,0]<=50_000_000
 u=(stamp-truth[j-1,0])/(truth[j,0]-truth[j-1,0]);r=matrix(slerp(truth[j-1,4:],truth[j,4:],u));p=(1-u)*truth[j-1,1:4]+u*truth[j,1:4]
 return p+r@lever,r@mount
folder=ROOT/'replay-uncalibrated-baseline';archive=load(folder/'acoustic-map.json')['points'];snapshots=load(folder/'submitted-snapshots.json.gz');fits=load(ROOT/'conditional-fit.json');by_id={r['source_id']:r for r in fits['rows']};sigma=np.array([.025083611719310283,.008213314126139592]);outside=0
for point in archive:
 row=by_id[point['source_id']];native=point['native_id'];matches=[u['observations'] for s in snapshots if s['stamp_ns']<=point['stamp_ns'] and s['sources'].get(str(native))==point['source_id'] for u in s['updates'] if u['id']==native];assert matches
 measured=[];rays=[];jac=[]
 for obs in matches[-1]:
  origin,r=pose(obs['stamp_ns']);q=r.T@(np.array(row['fit'])-origin);radius=np.linalg.norm(q);h2=q[0]**2+q[1]**2
  error=np.array([obs['range_m']-radius,obs['azimuth_rad']-np.arctan2(q[1],q[0])]);error[1]=np.arctan2(np.sin(error[1]),np.cos(error[1]));measured.append(error);rays.append(q)
  jac.append(np.array([q/radius,[-q[1]/h2,q[0]/h2,0]])@r.T/sigma[:,None])
 error=np.array(measured);q=np.array(rays);j=np.vstack(jac);small=np.linalg.eigvalsh(j.T@j)[0]
 np.testing.assert_allclose(np.sqrt(np.mean((error/sigma)**2)),row['normalized_residual_rms'],atol=1e-8)
 np.testing.assert_allclose(distance(np.array(row['fit'])),row['surface_error_m'],atol=2e-12)
 assert small<=0 or 3/np.sqrt(small)>.1
 outside+=int(np.max(np.abs(np.arctan2(q[:,2],np.hypot(q[:,0],q[:,1]))))>np.pi/18);checks+=1
assert outside==11 and len(by_id)==46
for label in ('setup-hold','setup-navigation-grid'):
 d=load(ROOT/'prior-attempts'/label/'lifecycle.json');assert d['exit']==0 and not d['forced'] and all(not g['live_members'] for g in d['groups'])
d=load(ROOT/'lifecycle.json');assert d['exit']==0 and not d['forced'] and all(not g['live_members'] for g in d['groups'])
figure=load(ROOT/'comparison-baseline.json');assert figure['all_points_retained']==figure['all_points_in_view']==46
assert figure['camera_stamp_ns']==figure['sonar_stamp_ns']==figure['isometric_stamp_ns']==35_000_000_000
assert load(ROOT/'replay-baseline/receipt.json')['failure']=="ValueError('worker_sonar_snapshot_unsupported')"
print('PASS:',checks,'complete trajectory, all-point surface and pose-conditioned consistency checks; physical map remains unqualified')
