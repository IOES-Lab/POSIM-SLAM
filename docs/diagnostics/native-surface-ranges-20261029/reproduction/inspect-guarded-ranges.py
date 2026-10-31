from pathlib import Path
import gzip,json,sys
import numpy as np
base=Path(__file__).resolve().parent
sys.path[:0]=[str(base/'library/src'),str(base/'library/tools')]
from posim_slam.native_surface import NativeSurfaceSnapshot
from posim_slam.native_surface_ranges import linearize_native_surface_ranges
from posim_slam.range_surface_joint import normal_basis,vertical_fan_plane_factor
from posim_slam.dvl_surface import cone_plane_range_factor
from posim_slam.inertial import exp_so3

def encode(value):
    if isinstance(value,np.ndarray):return value.tolist()
    if isinstance(value,np.generic):return value.item()
    raise TypeError(type(value).__name__)

results={}
for mode in ('dvl-only',):
    folder=base/('guarded-'+mode);request=json.loads((folder/'original-request.json').read_text())
    snapshots={key:NativeSurfaceSnapshot.read(json.loads(gzip.decompress((folder/('snapshot-'+key+'.json.gz')).read_bytes())))
        for key in ('before','after')}
    request['planes']={int(k):v for k,v in request['planes'].items()}
    before=linearize_native_surface_ranges(snapshots['before'],**request)
    post=dict(request,planes={ident:{k:v for k,v in p.items() if not k.startswith('initial_')}
        for ident,p in request['planes'].items()},require_support=False)
    # JSON turns the plane mapping key into a string. Restore its API type.
    post['planes']={int(k):v for k,v in post['planes'].items()}
    error=None
    try:after=linearize_native_surface_ranges(snapshots['after'],**post)
    except ValueError as exc:error=str(exc)
    rows=[];snapshot=snapshots['after'];plane=snapshot.reference('plane',0)
    delta=np.array([snapshot._archive.root[('plane',0,i)] for i in range(3)])
    n=exp_so3(normal_basis(plane.reference[:3])@delta[:2])@plane.reference[:3]
    distance=plane.reference[3]+delta[2]
    poses={v['stamp_ns']:v for v in snapshot.recover(sorted({o['stamp_ns'] for o in request['observations']}))['poses']}
    for o in request['observations']:
        if o.get('role','factor')!='factor':continue
        key=[o[k] for k in ('stamp_ns','sensor_id','beam_id')];s=poses[o['stamp_ns']]
        common=dict(position=s['position'],attitude=s['attitude'],plane_normal=n,plane_distance=distance,
            measured_range=o['measured_range'],range_min=o['range_min'],range_max=o['range_max'])
        try:
            if o['model']=='cone':
                value=cone_plane_range_factor(**common,imu_to_dvl=o['imu_to_sensor'],dvl_to_imu=o['sensor_to_imu'],
                    beam_axis=o['beam_axis'],aperture_rad=o['aperture_rad'])
            else:
                value=vertical_fan_plane_factor(**common,imu_to_sensor=o['imu_to_sensor'],sensor_to_imu=o['sensor_to_imu'],
                    azimuth_rad=o['azimuth_rad'],vertical_fov_rad=o['vertical_fov_rad'])
            rows.append(dict(key=key,predicted_range_m=value['predicted_range'],residual_m=value['residual'],
                conditional_hit=value['conditional_hit'],geometry_error=None))
        except ValueError as exc:rows.append(dict(key=key,geometry_error=str(exc),conditional_hit=None))
    summary=dict(plane_normal=n,plane_distance_m=distance,plane_delta=delta,
        before_conditional_information=before.diagnostics['conditional_plane_information'],
        before_nonlinear_residual_m=before.diagnostics['unwhitened_residual'],
        after_full_geometry_error=error,after_rows=rows,all_original_factor_rows_retained=True,
        fixed_gaussian_update_is_not_nonlinear_relinearization=True,
        physical_identity_verified=False,noise_calibrated=False,map_qualified=False)
    results[mode]=summary
    print(mode,'plane delta',delta,'geometry',error)
    info=summary['before_conditional_information'][0]
    print('ranks',info['cone_information']['rank'],info['all_range_information']['rank'],'fan',info['fan_predictions'])
    print('all post rows',rows)
(base/'guarded-nonlinear-inspection.json').write_text(json.dumps(results,default=encode,indent=2,allow_nan=False)+'\n')
