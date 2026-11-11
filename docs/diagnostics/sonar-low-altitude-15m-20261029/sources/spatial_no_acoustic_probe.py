"""Use existing committed spatial surface association on original DVL returns.

No truth is imported. Current centre-ray coordinates are association seeds,
not exact observed XYZ. Selection uses the current measured range, so accepted
residuals are not an independent held-out accuracy claim. Every seed is retained.
"""
from pathlib import Path
import csv,hashlib,json,sys
import numpy as np
root=Path.cwd();base=root/'.local/sonar-low-altitude-15m-20261029'
sys.path[:0]=[str(base/'runtime/libraries/posim-slam/src'),str(base/'runtime/libraries/posim-slam/tools')]
from evaluate_vio import matrix
from posim_slam.terrain_surface import SurfaceConfig,surface_matches
from posim_slam.dvl_surface import cone_plane_range_factor,require_cone_surface_support
cal=json.loads((base/'record/calibration.json').read_text())['dvl'];model=cal['bottom_ranges'];mount=cal['estimator']
beam={b['id']:b for b in model['beams']};lever=np.array(mount['imu_to_dvl']);rotation=np.array(mount['dvl_to_imu'])
poses={int(r['stamp_ns']):dict(position=np.array([float(r[k]) for k in ('x','y','z')]),attitude=matrix([float(r[k]) for k in ('qx','qy','qz','qw')])) for r in csv.DictReader((base/'replay-no-acoustic/states.csv').open())}
raw=json.loads((base/'record/inputs/dvl.json').read_text());history=[];times=[];rows=[];cloud=[];config=SurfaceConfig()
for row in raw:
 t=row['stamp_ns']
 if t not in poses:rows.append(dict(stamp_ns=t,status='outside_estimated_pose_span'));continue
 state=poses[t];r=state['attitude'];p=state['position'];origin=p+r@lever
 returns=[b for b in row['bottom_ranges']['beams'] if b['locked']]
 seeds=np.array([origin+b['range_m']*r@rotation@beam[b['id']]['axis'] for b in returns])
 reference=np.array(history).reshape(-1,3)
 matches=surface_matches(seeds,reference,np.array(times,dtype=np.int64),query_frame=t,config=config)
 eligible={int(k):(matches['normals'][i],matches['centers'][i]) for i,k in enumerate(matches['indices'])}
 samples=[]
 for i,(b,point) in enumerate(zip(returns,seeds)):
  output=point.copy();reason='no_spatial_plane';residual=None;predicted=None;patch=None
  if i in eligible:
   n,center=eligible[i]
   nearby=np.flatnonzero(np.linalg.norm(reference-point,axis=1)<=config.neighborhood_m)
   patch=dict(normal=n.tolist(),distance=float(n@center),support_points=reference[nearby].tolist(),support_stamps_ns=[times[k] for k in nearby])
   try:
    factor=cone_plane_range_factor(position=p,attitude=r,imu_to_dvl=lever,dvl_to_imu=rotation,beam_axis=beam[b['id']]['axis'],aperture_rad=beam[b['id']]['aperture_rad'],plane_normal=n,plane_distance=float(n@center),measured_range=b['range_m'],range_min=model['range_min_m'],range_max=model['range_max_m'])
    # All nearby prior seeds must also be consistent, not just PCA's subset.
    near=reference[np.linalg.norm(reference-point,axis=1)<=config.neighborhood_m]
    require_cone_surface_support(factor,near,max_plane_deviation=config.surface_sigma_m,max_support_distance=config.neighborhood_m)
    output=origin+(factor['conditional_hit']-origin)*b['range_m']/factor['predicted_range'];reason=None
    residual=factor['residual'];predicted=factor['predicted_range']
   except ValueError as exc:reason=str(exc)
  cloud.append(output)
  samples.append(dict(source_id=b['id'],range_m=b['range_m'],seed=point.tolist(),point=output.tolist(),association_rejection=reason,
    residual_m=residual,predicted_range_m=predicted,plane_patch=patch))
 rows.append(dict(stamp_ns=t,status='all_original_returns_retained',prior_frame_count=len(set(times)),prior_latest_stamp_ns=max(times) if times else None,
   spatial_rejections=matches['rejections'],beams=samples))
 history.extend(seeds);times.extend([t]*len(seeds))
samples=[b for row in rows for b in row.get('beams',[])];counts={}
for b in samples:
 key=b['association_rejection'] or 'within_local_hypothesis_support';counts[key]=counts.get(key,0)+1
report=dict(config=config.__dict__,rows=rows,all_range_points_retained=len(samples),association_counts=counts,
 association_uses_current_range=True,residual_is_independent_holdout=False,truth_is_estimator_input=False,
 physical_identity_verified=False,joint_pose_plane_uncertainty_available=False,map_qualified=False,online_state_updated=False,
 scope=__doc__,sources_sha256={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in (Path(__file__),base/'record/inputs/dvl.json',base/'record/calibration.json',base/'replay-no-acoustic/states.csv',base/'runtime/libraries/posim-slam/src/posim_slam/terrain_surface.py')})
(base/'dvl-spatial-no-acoustic.json').write_text(json.dumps(report,indent=2)+'\n')
np.savez_compressed(base/'dvl-spatial-no-acoustic-hypotheses.npz',points=np.array(cloud))
print(json.dumps({k:report[k] for k in ('all_range_points_retained','association_counts','map_qualified')}))
