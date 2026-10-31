"""First causal original patch; no error-based selection or truth input."""
from pathlib import Path
import csv,json
import numpy as np
from posim_slam.dvl_surface import cone_plane_range_factor,require_cone_surface_support
from posim_slam.inertial import exp_so3
from posim_slam.range_surface_joint import normal_basis
from posim_slam.sonar_ground import ground_entries
from evaluate_vio import matrix

ROOT=Path('/Volumes/ExternalHDD/POSIM/bluerov-slam-vrx')
DATA=Path('/Volumes/ExternalHDD/POSIM/validation-data/posim-slam/sonar-low-altitude-15m-20261029')
RECORD=ROOT/'docs/validation/sonar-low-altitude-15m-20261029'
cal=json.loads((DATA/'metadata/calibration.json').read_text())
raw={r['stamp_ns']:r for r in json.loads((DATA/'inputs/dvl.json').read_text())}
beam={r['id']:r for r in cal['dvl']['bottom_ranges']['beams']}
predictions=json.loads((RECORD/'dvl-sonar-prediction.json').read_text())['rows']
first=min(predictions,key=lambda r:(r['available_stamp_ns'],r['dvl_stamp_ns'],r['patch_beam_id'],r['sonar_stamp_ns'],r['beam_id']))
rows=[r for r in json.loads((RECORD/'dvl-spatial-no-acoustic.json').read_text())['rows'] if r['stamp_ns']==first['dvl_stamp_ns']]
patch=next(b['plane_patch'] for b in rows[0]['beams'] if b['source_id']==first['patch_beam_id'])
baseline={int(r['stamp_ns']):(np.array([float(r[k]) for k in ('x','y','z')]),matrix([float(r[k]) for k in ('qx','qy','qz','qw')]))
    for r in csv.DictReader((RECORD/'no-acoustic/states.csv').open())}
mount=cal['dvl']['estimator'];lever=np.array(mount['imu_to_dvl']);rotation=np.array(mount['dvl_to_imu'])

def cone(stamp,ident):
    measured=next(v for v in raw[stamp]['bottom_ranges']['beams'] if v['id']==ident)
    if not measured['locked']:raise ValueError('Original support beam lost bottom lock')
    return dict(model='cone',stamp_ns=stamp,sensor_id=0,beam_id=ident,measured_range=measured['range_m'],
        range_min=cal['dvl']['bottom_ranges']['range_min_m'],range_max=cal['dvl']['bottom_ranges']['range_max_m'],
        imu_to_sensor=lever,sensor_to_imu=rotation,beam_axis=beam[ident]['axis'],aperture_rad=beam[ident]['aperture_rad'])

def point(observation,poses):
    p,r=poses[observation['stamp_ns']]
    return p+r@(lever+observation['measured_range']*rotation@observation['beam_axis'])

# Historical compact patches kept support stamps/nominal seeds, but not IDs.
# Recover IDs only through unique equality to ORIGINAL nominal beam endpoints.
# These old XYZ values never become measurements or native support geometry.
keys=[];identity_errors=[]
for stamp,stored in zip(patch['support_stamps_ns'],patch['support_points']):
    distances=[np.linalg.norm(point(cone(stamp,i),baseline)-stored) for i in beam]
    matches=[i for i,distance in zip(beam,distances) if distance<1e-8]
    if len(matches)!=1:raise ValueError('Original support provenance is not uniquely reconstructable')
    keys.append((stamp,0,matches[0]));identity_errors.append(min(distances))
if len(set(keys))!=len(keys):raise ValueError('Duplicate original support key')
timeline=json.loads((DATA/'inputs/timeline.json').read_text())
sonar_row=next(v for v in timeline['sonar'] if v['stamp_ns']==first['sonar_stamp_ns'])
with np.load(DATA/'inputs'/sonar_row['file'],allow_pickle=False) as frame:
    entries=ground_entries({k:frame[k] for k in frame.files})
if entries['range_m'][first['beam_id']]!=first['observed_range_m']:raise ValueError('Original FLS onset changed')

def fit_seed(support,poses):
    points=np.array([point(o,poses) for o in support]);center=points.mean(axis=0)
    n=np.linalg.svd(points-center,full_matrices=False)[2][-1];d=float(n@center)
    if n@(poses[max(o['stamp_ns'] for o in support)][0]-center)<0:n=-n;d=-d
    def evaluate(normal,distance):
        values=[];jac=[]
        for o in support:
            p,r=poses[o['stamp_ns']]
            f=cone_plane_range_factor(position=p,attitude=r,imu_to_dvl=lever,dvl_to_imu=rotation,
                beam_axis=o['beam_axis'],aperture_rad=o['aperture_rad'],plane_normal=normal,
                plane_distance=distance,measured_range=o['measured_range'],range_min=o['range_min'],range_max=o['range_max'])
            values.append(-f['residual']);jac.append(np.r_[f['plane_jacobian'][:3]@normal_basis(normal),f['plane_jacobian'][3]])
        return np.array(values),np.array(jac)
    # A numerical initializer only. Its normal/distance/covariance is not a prior.
    damping=1e-3;residual,j=evaluate(n,d)
    for _ in range(25):
        step=np.linalg.solve(j.T@j+damping*max(float(np.trace(j.T@j))/3,1)*np.eye(3),-j.T@residual)
        trial_n=exp_so3(normal_basis(n)@step[:2])@n;trial_d=d+step[2]
        trial=evaluate(trial_n,trial_d)
        if trial[0]@trial[0]<residual@residual:
            n,d=trial_n,trial_d;residual,j=trial;damping=max(damping/3,1e-10)
        else:damping=min(damping*10,1e12)
    return n,d,points


def prepare(snapshot,include_sonar):
    support=[cone(t,b) for t,_,b in keys]
    current=cone(first['dvl_stamp_ns'],first['patch_beam_id'])
    observations=support+[current]
    if include_sonar:
        s=cal['sonar'];ident=first['beam_id']
        observations.append(dict(model='fan',stamp_ns=first['sonar_stamp_ns'],sensor_id=1,beam_id=ident,
            measured_range=float(entries['range_m'][ident]),azimuth_rad=float(entries['azimuth_rad'][ident]),
            vertical_fov_rad=s['vertical_fov'],range_min=s['range_min'],range_max=s['range_max'],
            imu_to_sensor=s['imu_to_sonar'],sensor_to_imu=s['sonar_to_imu']))
    stamps=sorted({o['stamp_ns'] for o in observations})
    poses={v['stamp_ns']:(v['position'],v['attitude']) for v in snapshot.recover(stamps)['poses']}
    n,d,points=fit_seed(support,poses)
    statuses=[]
    for o in observations:
        o.update(plane_id=0,association='surface_hypothesis')
        if o['model']=='cone':
            p,r=poses[o['stamp_ns']]
            f=cone_plane_range_factor(position=p,attitude=r,imu_to_dvl=lever,dvl_to_imu=rotation,
                beam_axis=o['beam_axis'],aperture_rad=o['aperture_rad'],plane_normal=n,
                plane_distance=d,measured_range=o['measured_range'],range_min=o['range_min'],range_max=o['range_max'])
            try:
                require_cone_surface_support(f,points,max_plane_deviation=.05,max_support_distance=.75)
                reason='whole_cone_supported'
            except ValueError as error:o['role']='support_only';reason=str(error)
        else:reason='explicit_unverified_same_surface_hypothesis'
        statuses.append(dict(key=[o['stamp_ns'],o['sensor_id'],o['beam_id']],role=o.get('role','factor'),reason=reason))
    covariance=np.zeros((len(observations),len(observations)))
    for i,a in enumerate(observations):
        covariance[i,i]=.02**2 if a['model']=='cone' else .025083611719310283**2
        for k,b in enumerate(observations):
            if a['model']==b['model']=='cone' and a['stamp_ns']==b['stamp_ns']:covariance[i,k]+=.01**2
    request=dict(planes={0:dict(initial_normal=n,initial_distance=d,support_keys=keys,
        max_plane_deviation=.05,max_support_distance=.75)},observations=observations,measurement_covariance=covariance,
        uncertainty_source='Explicit uncalibrated diagnostic: DVL 20 mm independent / 10 mm same-frame, FLS 25.0836 mm; velocity/range correlation unmodelled')
    diagnostics=dict(first_original_prediction=first,support_provenance_errors_m=identity_errors,statuses=statuses,
        plane_seed_normal=n.tolist(),plane_seed_distance_m=d,physical_identity_verified=False,
        noise_calibrated=False,velocity_range_cross_covariance_modelled=False,truth_input=False,map_qualified=False)
    return request,diagnostics
