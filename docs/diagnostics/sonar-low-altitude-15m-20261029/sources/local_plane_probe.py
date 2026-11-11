"""Causal local DVL plane hypotheses from estimated poses; no truth input."""
from pathlib import Path
import csv,hashlib,importlib.util,json,math,sys
import numpy as np
root=Path.cwd();base=root/'.local/sonar-low-altitude-15m-20261029'
sys.path[:0]=[str(base/'runtime/libraries/posim-slam/src'),str(base/'runtime/libraries/posim-slam/tools')]
from posim_slam.dvl_surface import cone_plane_range_factor,require_cone_surface_support
from posim_slam.inertial import skew
from evaluate_vio import matrix,slerp
cal=json.loads((base/'record/calibration.json').read_text())['dvl'];model=cal['bottom_ranges'];mount=cal['estimator']
beam={b['id']:b for b in model['beams']};lever=np.array(mount['imu_to_dvl']);rotation=np.array(mount['dvl_to_imu'])
poses={int(r['stamp_ns']):dict(position=np.array([float(r[k]) for k in ('x','y','z')]),attitude=matrix([float(r[k]) for k in ('qx','qy','qz','qw')])) for r in csv.DictReader((base/'replay-baseline/states.csv').open())}
control='--assessment-pose-control' in sys.argv
if control:
 truth=np.array(json.loads((base/'record/inputs/truth-assessment-only.json').read_text()))
 imu=json.loads((base/'record/calibration.json').read_text())['imu']['mount']
 for stamp in poses:
  index=int(np.searchsorted(truth[:,0],stamp));assert 0<index<len(truth)
  weight=(stamp-truth[index-1,0])/(truth[index,0]-truth[index-1,0]);r=matrix(slerp(truth[index-1,4:],truth[index,4:],weight))
  poses[stamp]=dict(position=(1-weight)*truth[index-1,1:4]+weight*truth[index,1:4]+r@imu['translation'],attitude=r@matrix(imu['rotation']))
raw=json.loads((base/'record/inputs/dvl.json').read_text())
config=dict(prior_frames=3,independent_sigma_m=.02,shared_sigma_m=.01,max_support_distance_m=.75,max_plane_deviation_m=.05,max_iterations=25)

def normal_basis(n):
 axis=np.eye(3)[np.argmin(abs(n))];u=np.cross(n,axis);u/=np.linalg.norm(u)
 return np.column_stack((u,np.cross(n,u)))

def sample(row):
 t=row['stamp_ns'];s=poses[t]
 return [dict(stamp_ns=t,source_id=b['id'],range_m=b['range_m'],axis=beam[b['id']]['axis'],aperture_rad=beam[b['id']]['aperture_rad'],state=s) for b in row['bottom_ranges']['beams'] if b['locked']]

def seeds(observations):
 return np.array([o['state']['position']+o['state']['attitude']@(lever+o['range_m']*rotation@o['axis']) for o in observations])

def evaluate(observations,n,d,aperture=True):
 basis=normal_basis(n);values=[];jac=[];factors=[]
 for o in observations:
  s=o['state'];f=cone_plane_range_factor(position=s['position'],attitude=s['attitude'],imu_to_dvl=lever,dvl_to_imu=rotation,beam_axis=o['axis'],aperture_rad=o['aperture_rad'] if aperture else 0.,plane_normal=n,plane_distance=d,measured_range=o['range_m'],range_min=model['range_min_m'],range_max=model['range_max_m'])
  values.append(-f['residual']);j=f['plane_jacobian'];jac.append(np.r_[j[:3]@skew(n)@basis,j[3]]);factors.append(f)
 covariance=np.eye(len(observations))*config['independent_sigma_m']**2
 for a,o in enumerate(observations):
  for b,p in enumerate(observations):
   if o['stamp_ns']==p['stamp_ns']:covariance[a,b]+=config['shared_sigma_m']**2
 lower=np.linalg.cholesky(covariance)
 return np.linalg.solve(lower,values),np.linalg.solve(lower,jac),factors

def fit(observations,aperture):
 points=seeds(observations);center=points.mean(axis=0);_,_,v=np.linalg.svd(points-center,full_matrices=False);n=v[-1];d=float(n@center)
 if n@(observations[0]['state']['position']-center)<0:n=-n;d=-d
 current=evaluate(observations,n,d,aperture);damping=1e-3
 for _ in range(config['max_iterations']):
  residual,jac,_=current;h=jac.T@jac;step=np.linalg.solve(h+damping*max(float(np.trace(h))/3,1)*np.eye(3),-jac.T@residual)
  proposed=n+normal_basis(n)@step[:2];proposed/=np.linalg.norm(proposed)
  try:trial=evaluate(observations,proposed,d+step[2],aperture)
  except ValueError:damping*=10;continue
  if trial[0]@trial[0]<residual@residual:
   n=proposed;d+=step[2];current=trial;damping=max(damping/3,1e-10)
   if np.linalg.norm(step)<1e-8:break
  else:damping=min(damping*10,1e12)
 return n,d,current,points

output={}
for method,aperture in [('continuous_cone',True),('centre_ray',False)]:
 history=[];rows=[];points=[];origins=[]
 for row in raw:
  t=row['stamp_ns']
  if t not in poses:rows.append(dict(stamp_ns=t,status='outside_estimated_pose_span'));continue
  observations=sample(row)
  if len(observations)!=4:rows.append(dict(stamp_ns=t,status='partial_bottom_lock',locked_beams=len(observations)));history=[];continue
  if len(history)<config['prior_frames']:
   rows.append(dict(stamp_ns=t,status='collecting_prior_frames'));history.append(observations);continue
  prior=[o for frame in history[-config['prior_frames']:] for o in frame]
  assert all(o['stamp_ns']<t for o in prior)
  try:
   n,d,(fit_residual,fit_jac,_),support=fit(prior,aperture)
   _,_,factors=evaluate(observations,n,d,aperture)
   samples=[]
   for o,f in zip(observations,factors):
    state=o['state'];origin=state['position']+state['attitude']@lever
    # Preserve the ACTUAL held-out measured range. Do not snap to the plane.
    point=origin+(f['conditional_hit']-origin)*o['range_m']/f['predicted_range']
    reason=None
    try:require_cone_surface_support(f,support,max_plane_deviation=config['max_plane_deviation_m'],max_support_distance=config['max_support_distance_m'])
    except ValueError as exc:reason=str(exc)
    points.append(point);origins.append(origin)
    samples.append(dict(source_id=o['source_id'],measured_range_m=o['range_m'],predicted_range_m=f['predicted_range'],residual_m=f['residual'],support_rejection=reason,point=point.tolist()))
   rows.append(dict(stamp_ns=t,status='heldout_prediction',prior_stamps_ns=sorted({o['stamp_ns'] for o in prior}),plane_normal=n.tolist(),plane_distance=d,
    prior_normalized_rms=float(np.sqrt(np.mean(fit_residual**2))),minimum_plane_information_eigenvalue=float(np.linalg.eigvalsh(fit_jac.T@fit_jac)[0]),beams=samples))
  except ValueError as exc:rows.append(dict(stamp_ns=t,status='plane_unavailable',reason=str(exc)))
  history.append(observations)
 predictions=[b for r in rows for b in r.get('beams',[])]
 values=np.array([b['residual_m'] for b in predictions]);counts={}
 for r in rows:counts[r['status']]=counts.get(r['status'],0)+1
 rejected={}
 for b in predictions:
  key=b['support_rejection'] or 'within_hypothesis_support';rejected[key]=rejected.get(key,0)+1
 output[method]=dict(rows=rows,all_point_hypotheses_retained=len(points),status_counts=counts,support_counts=rejected,
  heldout_absolute_residual_m=dict(median=float(np.median(abs(values))),p90=float(np.percentile(abs(values),90)),maximum=float(abs(values).max())),
  conditional_on_estimated_poses=not control,assessment_only_pose_control=control,physical_identity_verified=False,joint_pose_plane_uncertainty_available=False,map_qualified=False)
 np.savez_compressed(base/(('dvl-assessment-' if control else 'dvl-')+method+'-hypotheses.npz'),points=np.array(points),origins=np.array(origins))
report=dict(config=config,methods=output,truth_is_native_estimator_input=False,truth_is_plane_diagnostic_input=control,online_state_updated=False,
 scope='Prior-three-frame DVL plane hypotheses and held-out range predictions; '+('assessment-only perfect poses' if control else 'fixed native estimated poses')+'; uncertain association, not native range fusion or admitted map',
 sources_sha256={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in (Path(__file__),base/'record/inputs/dvl.json',base/'record/calibration.json',base/'replay-baseline/states.csv',*([base/'record/inputs/truth-assessment-only.json'] if control else []))})
(base/('dvl-plane-probe-assessment-control.json' if control else 'dvl-plane-probe.json')).write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps({name:{k:v for k,v in result.items() if k!='rows'} for name,result in output.items()}))
