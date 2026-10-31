from pathlib import Path
import collections,importlib.util,json,sys
import numpy as np
base=Path(__file__).resolve().parent
sys.path[:0]=[str(base/'library/src'),str(base/'library/tools')]
spec=importlib.util.spec_from_file_location('original_block',base/'original-block.py')
original=importlib.util.module_from_spec(spec);spec.loader.exec_module(original)
from posim_slam.native_surface_ranges import conditional_surface_observability
from posim_slam.range_surface_joint import linearize_surface_ranges
from posim_slam.dvl_surface import cone_plane_range_factor,require_cone_surface_support
spatial=json.loads((original.RECORD/'dvl-spatial-no-acoustic.json').read_text())
patches={(r['stamp_ns'],b['source_id']):b['plane_patch'] for r in spatial['rows']
    for b in r.get('beams',[]) if b['association_rejection'] is None}
groups=collections.defaultdict(list)
for row in original.predictions:groups[(row['dvl_stamp_ns'],row['patch_beam_id'])].append(row)
results=[]
for key,predictions in sorted(groups.items()):
    patch=patches[key];n=np.asarray(patch['normal']);d=patch['distance'];support=[];provenance=[]
    for stamp,stored in zip(patch['support_stamps_ns'],patch['support_points']):
        distances={ident:float(np.linalg.norm(original.point(original.cone(stamp,ident),original.baseline)-stored)) for ident in original.beam}
        matches=[ident for ident,error in distances.items() if error<1e-8]
        if len(matches)!=1:raise ValueError('Original provenance is not unique')
        support.append(original.cone(stamp,matches[0]));provenance.append([stamp,0,matches[0]])
    n,d,points=original.fit_seed(support,{t:original.baseline[t] for t in {o['stamp_ns'] for o in support}})
    cones=support+[original.cone(*key)];statuses=[]
    for o in cones:
        o.update(plane_id=0,association='surface_hypothesis')
        p,r=original.baseline[o['stamp_ns']]
        f=cone_plane_range_factor(position=p,attitude=r,imu_to_dvl=o['imu_to_sensor'],dvl_to_imu=o['sensor_to_imu'],
            beam_axis=o['beam_axis'],aperture_rad=o['aperture_rad'],plane_normal=n,plane_distance=d,
            measured_range=o['measured_range'],range_min=o['range_min'],range_max=o['range_max'])
        try:require_cone_surface_support(f,points,max_plane_deviation=.05,max_support_distance=.75);reason='whole_cone_supported'
        except ValueError as exc:o['role']='support_only';reason=str(exc)
        statuses.append(dict(key=[o[k] for k in ('stamp_ns','sensor_id','beam_id')],role=o.get('role','factor'),reason=reason))
    for prediction in predictions:
        s=original.cal['sonar'];ident=prediction['beam_id'];stamp=prediction['sonar_stamp_ns']
        # Existing report retains the exact original detected onset. It never
        # uses that value to choose a patch or support geometry.
        az=-s['horizontal_fov']/2+ident*s['horizontal_fov']/(s['beams']-1)
        fan=dict(model='fan',stamp_ns=stamp,sensor_id=1,beam_id=ident,plane_id=0,association='surface_hypothesis',
            measured_range=prediction['observed_range_m'],azimuth_rad=az,vertical_fov_rad=s['vertical_fov'],
            range_min=s['range_min'],range_max=s['range_max'],imu_to_sensor=s['imu_to_sonar'],sensor_to_imu=s['sonar_to_imu'])
        observations=cones+[fan];stamps=sorted({o['stamp_ns'] for o in observations})
        row=dict(key=[stamp,1,ident],dvl_patch=list(key),source_keys=provenance,cone_roles=statuses,
            original_residual_m=prediction['residual_m'],source_pose_count=len(stamps))
        if len(stamps)>16:row.update(status='source_block_exceeds_unchanged_pose_budget')
        else:
            covariance=np.zeros((len(observations),len(observations)))
            for i,a in enumerate(observations):
                covariance[i,i]=.02**2 if a['model']=='cone' else .025083611719310283**2
                for j,b in enumerate(observations):
                    if a['model']==b['model']=='cone' and a['stamp_ns']==b['stamp_ns']:covariance[i,j]+=.01**2
            states={t:dict(position=original.baseline[t][0],attitude=original.baseline[t][1]) for t in stamps}
            try:
                block=linearize_surface_ranges(states=states,planes={0:dict(normal=n,distance=d,support_keys=provenance,
                    max_plane_deviation=.05,max_support_distance=.75)},observations=observations,
                    measurement_covariance=covariance,uncertainty_source='Uncalibrated diagnostic; original conservative support and known baseline poses')
                info=conditional_surface_observability(block,observations)[0];query=info['fan_predictions'][0]
                row.update(status='unbounded_from_supported_cones' if query['unbounded_from_cone_rows'] else 'finite_known_pose_conditional',
                    cone_rows=info['cone_rows'],cone_rank=info['cone_information']['rank'],
                    fan_stddev_m=query['conditional_cone_only_stddev_m'])
            except ValueError as exc:row.update(status='geometry_or_support_rejection',reason=str(exc))
        results.append(row)
counts=dict(collections.Counter(row['status'] for row in results))
result=dict(scope='All original prediction hypotheses; known-pose conditional block observability only',
    rows=results,status_counts=counts,original_predictions=len(original.predictions),patch_groups=len(groups),
    no_residual_based_selection=True,pose_uncertainty_included=False,owner_prior_included=False,
    physical_identity_verified=False,noise_calibrated=False,map_qualified=False)
def encode(value):
    if isinstance(value,np.ndarray):return value.tolist()
    if isinstance(value,np.generic):return value.item()
    raise TypeError(type(value).__name__)
(base/'all-original-observability.json').write_text(json.dumps(result,default=encode,indent=2,allow_nan=False)+'\n')
print('All original hypotheses',len(results),'patches',len(groups),'statuses',counts)
