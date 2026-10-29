"""Joint finite-cone / vertical-fan ranges and unknown local surfaces.

Every measured row has one immutable (stamp, sensor, beam) key. Factor rows
have an explicit surface hypothesis. Poses/planes and full covariance stay in one
block. Raw cone ranges can generate nominal support seeds at CURRENT poses;
these seeds are guards, not Cartesian observations or independent map priors.
This block does not identify echoes, calibrate noise or qualify a live map.
"""
from __future__ import annotations
import math
import numpy as np

from .dvl_scale import rotation,vector
from .dvl_surface import cone_plane_range_factor,require_cone_surface_support
from .inertial import exp_so3,left_jacobian_so3,skew
from .inertial_filter import covariance
from .surface_support import require_surface_support


def normal_basis(normal):
    normal=vector(normal)
    if not np.isclose(normal@normal,1.,atol=1e-8,rtol=0):
        raise ValueError('unit_surface_normal_required')
    axis=np.eye(3)[np.argmin(abs(normal))];u=np.cross(normal,axis);u/=np.linalg.norm(u)
    return np.column_stack((u,np.cross(normal,u)))


def vertical_fan_plane_factor(*,position,attitude,imu_to_sensor,sensor_to_imu,
                              azimuth_rad,vertical_fov_rad,plane_normal,
                              plane_distance,measured_range,range_min,range_max):
    """Continuous first geometric intersection, including the interior branch.

    Jacobians describe the prediction: additive map position, right IMU
    rotation, left normal rotation (3) and additive signed plane distance.
    A geometric minimum is not necessarily an observed intensity onset.
    """
    p=vector(position);r=rotation(attitude);lever=vector(imu_to_sensor)
    mount=rotation(sensor_to_imu);n=vector(plane_normal);normal_basis(n)
    if (not np.isfinite([azimuth_rad,vertical_fov_rad,plane_distance,
                        measured_range,range_min,range_max]).all()
            or abs(azimuth_rad)>=math.pi/2 or not 0<vertical_fov_rad<math.pi
            or not 0<range_min<range_max or not range_min<=measured_range<=range_max):
        raise ValueError('invalid_surface_fan_geometry')
    origin=p+r@lever;height=float(plane_distance-n@origin)
    if abs(height)<1e-9:raise ValueError('surface_origin_on_plane')
    local=math.copysign(1.,height)*(r@mount).T@n
    a=local[0]*math.cos(azimuth_rad)+local[1]*math.sin(azimuth_rad);b=local[2]
    half=vertical_fov_rad/2;stationary=math.atan2(b,a)
    candidates=[-half,half]
    if -half<stationary<half:candidates.append(stationary)
    values=[a*math.cos(e)+b*math.sin(e) for e in candidates]
    selected=int(np.argmax(values));elevation=candidates[selected]
    if values[selected]<=1e-10:raise ValueError('surface_fan_has_no_forward_return')
    direction=np.array([math.cos(azimuth_rad)*math.cos(elevation),
                        math.sin(azimuth_rad)*math.cos(elevation),math.sin(elevation)])
    body=mount@direction;world=r@body;denominator=float(n@world)
    predicted=height/denominator
    if not range_min<=predicted<=range_max:raise ValueError('surface_first_return_out_of_range')
    jo=-n/denominator;jd=predicted*jo;hit=origin+predicted*world
    pose=np.r_[jo,-jo@r@skew(lever)-jd@r@skew(body)]
    ambient=-hit/denominator
    return dict(predicted_range=float(predicted),residual=float(measured_range-predicted),
        pose_jacobian=pose,plane_jacobian=np.r_[-ambient@skew(n),1/denominator],
        conditional_hit=hit,selected_elevation_rad=float(elevation),supported=False,
        map_qualified=False,physical_identity_verified=False)


def _pose(state):
    p=vector(state['position'])
    if 'rotation_reference' in state:
        if 'attitude' in state or 'rotation_delta' not in state:
            raise ValueError('complete_surface_pose_coordinates_required')
        delta=vector(state['rotation_delta']);r=exp_so3(delta)@rotation(state['rotation_reference'])
    else:
        if 'rotation_delta' in state:raise ValueError('surface_pose_reference_required')
        delta=np.zeros(3);r=rotation(state['attitude'])
    return p,r,left_jacobian_so3(delta)


def _plane(plane):
    if 'support_points' in plane:raise ValueError('frozen_surface_support_points_not_allowed')
    if 'normal_reference' in plane:
        if 'normal' in plane or 'normal_delta' not in plane:
            raise ValueError('complete_surface_plane_coordinates_required')
        reference=vector(plane['normal_reference']);basis=normal_basis(reference)
        delta=np.asarray(plane['normal_delta'],dtype=float)
        if delta.shape!=(2,) or not np.isfinite(delta).all():raise ValueError('invalid_surface_plane_delta')
        tangent=basis@delta;n=exp_so3(tangent)@reference
        coordinates=left_jacobian_so3(tangent)@basis
    else:
        if 'normal_delta' in plane:raise ValueError('surface_plane_reference_required')
        n=vector(plane['normal']);coordinates=normal_basis(n)
    distance=plane['distance']
    if not isinstance(distance,(int,float)) or not math.isfinite(distance):
        raise ValueError('finite_surface_plane_distance_required')
    return n,float(distance),coordinates


def linearize_surface_ranges(*,states,planes,observations,measurement_covariance,
                             uncertainty_source,require_support=True):
    """One jointly whitened block with explicit per-beam plane membership.

    Columns: sorted poses (map position/LEFT rotation coordinates, 6 each),
    sorted planes (normal rotation coordinates, 2, and distance, 1).
    Optional reference/delta inputs apply the accumulated SO(3) Jacobian.
    Repeated timestamps are valid for different sensor/beam keys. Duplicate
    original readings are errors, including readings assigned to two planes.

    Factor observations declare association='surface_hypothesis', model='cone'
    or 'fan', stamp_ns, sensor_id, beam_id, plane_id, measured_range, limits
    and mounting. Cone rows add beam_axis/aperture_rad; fan rows add azimuth_rad
    and vertical_fov_rad. role='support_only' permits raw cone readings to guard
    another return without manufacturing a plane association or a factor.
    Every input reading remains represented once. Full covariance follows ALL
    reading rows; use its active principal submatrix, not a covariance
    conditioned on unfused readings.

    Plane support_keys refer to CONE rows in this same block. Their immutable
    ranges/axes generate nominal seeds at current poses, preserving gauge.
    Guards supply no extra information rows. Missing/retired support cannot be
    replaced by frozen map XYZ. require_support=False is diagnostic-only.
    Echo visibility/material correspondence remains a caller responsibility.
    """
    if (not 1<=len(states)<=16 or not 1<=len(planes)<=8 or not 1<=len(observations)<=512
            or not isinstance(uncertainty_source,str) or not uncertainty_source.strip()
            or type(require_support) is not bool):
        raise ValueError('bounded_surface_block_and_explicit_noise_required')
    stamps=sorted(states);plane_ids=sorted(planes)
    if (any(type(t) is not int or t<=0 for t in stamps)
            or any(type(i) is not int or i<0 for i in plane_ids)):
        raise ValueError('invalid_surface_state_or_plane_ids')
    poses={t:_pose(states[t]) for t in stamps};surfaces={i:_plane(planes[i]) for i in plane_ids}
    count=len(observations);noise=covariance(measurement_covariance,count)
    pose_columns={t:6*i for i,t in enumerate(stamps)}
    plane_columns={k:6*len(stamps)+3*i for i,k in enumerate(plane_ids)}
    columns=6*len(stamps)+3*len(plane_ids)
    keys=[];lookup={};factor_indices=[];support_only=[]
    for row,o in enumerate(observations):
        key=(o['stamp_ns'],o['sensor_id'],o['beam_id'])
        if (type(key[0]) is not int or key[0] not in poses
                or any(type(i) is not int or i<0 for i in key[1:])
                or key in lookup):
            raise ValueError('missing_pose_duplicate_or_unresolved_surface_row')
        # Validate every raw reading, including support-only sources. Unknown
        # noise is never silently replaced by zero and no fake range is added.
        minimum=o['range_min'];maximum=o['range_max'];measured=o['measured_range']
        if (not np.isfinite([minimum,maximum,measured]).all()
                or not 0<minimum<maximum or not minimum<=measured<=maximum):
            raise ValueError('invalid_surface_range_measurement')
        vector(o['imu_to_sensor']);rotation(o['sensor_to_imu'])
        role=o.get('role','factor')
        if role=='support_only':
            if o['model']!='cone':raise ValueError('unmeasured_fan_elevation_is_not_surface_support')
            axis=vector(o['beam_axis']);aperture=o['aperture_rad']
            if (not np.isclose(axis@axis,1.,atol=1e-8,rtol=0)
                    or not math.isfinite(aperture) or not 0<=aperture<math.pi):
                raise ValueError('invalid_raw_surface_support_cone')
            support_only.append(row)
        elif (role=='factor' and type(o.get('plane_id')) is int and o['plane_id'] in surfaces
                and o.get('association')=='surface_hypothesis'):
            factor_indices.append(row)
        else:raise ValueError('missing_pose_duplicate_or_unresolved_surface_row')
        lookup[key]=row;keys.append(key)
    if not factor_indices:raise ValueError('no_surface_factor_rows')
    j=np.zeros((len(factor_indices),columns));residual=np.zeros(len(factor_indices));factors=[]
    for row,index in enumerate(factor_indices):
        o=observations[index];key=keys[index]
        p,r,pose_coordinates=poses[key[0]];n,d,plane_coordinates=surfaces[o['plane_id']]
        common=dict(position=p,attitude=r,plane_normal=n,plane_distance=d,
            measured_range=o['measured_range'],range_min=o['range_min'],range_max=o['range_max'])
        if o['model']=='cone':
            f=cone_plane_range_factor(**common,imu_to_dvl=o['imu_to_sensor'],dvl_to_imu=o['sensor_to_imu'],
                beam_axis=o['beam_axis'],aperture_rad=o['aperture_rad'])
        elif o['model']=='fan':
            f=vertical_fan_plane_factor(**common,imu_to_sensor=o['imu_to_sensor'],sensor_to_imu=o['sensor_to_imu'],
                azimuth_rad=o['azimuth_rad'],vertical_fov_rad=o['vertical_fov_rad'])
        else:raise ValueError('unknown_surface_range_model')
        pc=pose_columns[key[0]];nc=plane_columns[o['plane_id']]
        residual[row]=-f['residual']
        j[row,pc:pc+3]=f['pose_jacobian'][:3]
        j[row,pc+3:pc+6]=f['pose_jacobian'][3:]@r.T@pose_coordinates
        j[row,nc:nc+2]=f['plane_jacobian'][:3]@plane_coordinates
        j[row,nc+2]=f['plane_jacobian'][3]
        factors.append(f)
    support_seeds={}
    if require_support:
        for ident,plane in planes.items():
            support=plane.get('support_keys',[])
            if not 3<=len(support)<=4096 or len(set(map(tuple,support)))!=len(support):
                raise ValueError('explicit_distinct_raw_surface_support_required')
            points=[]
            for key in map(tuple,support):
                if key not in lookup:raise ValueError('surface_support_pose_or_measurement_retired')
                source=observations[lookup[key]]
                if source['model']!='cone':raise ValueError('unmeasured_fan_elevation_is_not_surface_support')
                p,r,_=poses[key[0]]
                points.append(p+r@(vector(source['imu_to_sensor'])+source['measured_range']*
                    rotation(source['sensor_to_imu'])@vector(source['beam_axis'])))
            support_seeds[ident]=np.array(points)
        for index,f in zip(factor_indices,factors):
            o=observations[index]
            plane=planes[o['plane_id']];n,d,_=surfaces[o['plane_id']]
            points=support_seeds[o['plane_id']]
            deviation=plane['max_plane_deviation'];radius=plane['max_support_distance']
            if o['model']=='cone':
                require_cone_surface_support(f,points,max_plane_deviation=deviation,max_support_distance=radius)
            else:
                if (not math.isfinite(deviation) or deviation<=0 or np.max(abs(points@n-d))>deviation):
                    raise ValueError('surface_support_not_planar')
                require_surface_support(points,[f['conditional_hit']],n,max_distance=radius)
    active_noise=noise[np.ix_(factor_indices,factor_indices)]
    lower=np.linalg.cholesky(active_noise)
    return dict(residual=np.linalg.solve(lower,residual),jacobian=np.linalg.solve(lower,j),
        unwhitened_residual=residual,unwhitened_jacobian=j,
        state_stamps_ns=stamps,plane_ids=plane_ids,source_keys=keys,
        factor_indices=factor_indices,support_only_indices=support_only,
        factor_keys=[keys[i] for i in factor_indices],measurement_covariance=active_noise,
        predicted_ranges=np.array([f['predicted_range'] for f in factors]),
        conditional_hits=np.array([f['conditional_hit'] for f in factors]),
        support_seeds=support_seeds,all_support_checked=require_support,
        uncertainty_source=uncertainty_source,map_qualified=False,physical_identity_verified=False,
        scope='Joint conditional surface rows; no fitted-plane prior, echo identity or navigation qualification')
