"""Snapshot-coordinate sonar rows after eliminating unknown point coordinates.

Uses public POSIM-SLAM geometry and square-root elimination. It is an offline
diagnostic: source association/noise, elevation modes and live admission remain
the caller's responsibility. A fitted point is never an independent prior.
"""
import numpy as np
from posim_slam.inertial import exp_so3,left_jacobian_so3
from posim_slam.ping_joint import split_patch_information
from posim_slam.sonar_factor import point_return_factor

def owner_poses(references,delta):
    references=np.asarray(references,float);delta=np.asarray(delta,float)
    if (references.shape!=(2,12) or delta.shape!=(2,6)
            or not np.isfinite(references).all() or not np.isfinite(delta).all()):
        raise ValueError('two_finite_owner_pose_coordinates_required')
    positions=references[:,:3]+delta[:,:3]
    rotations=np.array([exp_so3(d[3:])@v[3:12].reshape(3,3) for v,d in zip(references,delta)])
    return positions,rotations

def point_rows(*,references,delta,points,measurements,sonar_to_imu,imu_to_sonar,
               geometry,measurement_covariance):
    """Whiten jointly before removing all point-coordinate sensitivity.

    Row order is point, view A/B, range/azimuth. Pose columns are each owner's
    additive position and accumulated LEFT exponential rotation coordinates.
    Full cross-observation covariance is accepted, rather than independent
    fitted points or independent pose estimates.
    """
    points=np.asarray(points,float);observed=np.asarray(measurements,float)
    if (points.ndim!=2 or points.shape[1]!=3 or not 1<=len(points)<=64
            or observed.shape!=(len(points),2,2) or not np.isfinite(points).all()
            or not np.isfinite(observed).all()):raise ValueError('bounded_original_point_observations_required')
    position,rotation=owner_poses(references,delta)
    covariance=np.asarray(measurement_covariance,float);rows=4*len(points)
    if (covariance.shape!=(rows,rows) or not np.isfinite(covariance).all()
            or not np.allclose(covariance,covariance.T,rtol=1e-12,atol=1e-15)):
        raise ValueError('explicit_joint_measurement_covariance_required')
    try:chol=np.linalg.cholesky(covariance)
    except np.linalg.LinAlgError as error:raise ValueError('positive_definite_measurement_covariance_required') from error
    pose=np.zeros((rows,12));point=np.zeros((rows,3*len(points)));error=np.zeros(rows)
    for i,landmark in enumerate(points):
        for view in range(2):
            factor=point_return_factor(position=position[view],attitude=rotation[view],
                imu_to_sonar=imu_to_sonar,sonar_to_imu=sonar_to_imu,landmark=landmark,
                measured_range=observed[i,view,0],measured_azimuth=observed[i,view,1],**geometry)
            k=4*i+2*view;j=factor['pose_jacobian'].copy()
            # Public factor uses fresh RIGHT IMU rotation. Owner delta is fixed
            # accumulated LEFT rotation, including its nonzero exponential map.
            j[:,3:]=j[:,3:]@rotation[view].T@left_jacobian_so3(np.asarray(delta)[view,3:])
            pose[k:k+2,6*view:6*view+6]=j
            point[k:k+2,3*i:3*i+3]=factor['landmark_jacobian']
            error[k:k+2]=-factor['residual']
    white=np.linalg.solve(chol,np.column_stack([pose,point,error]))
    reduced=split_patch_information(white[:,:-1],white[:,-1],retained_columns=12)
    reduced.update(raw_pose_jacobian=pose,raw_point_jacobian=point,raw_residual=error,
        joint_whitened_jacobian=white[:,:-1],joint_whitened_residual=white[:,-1],
        measurement_covariance=covariance,position=position,rotation=rotation,
        original_observations=len(points),snapshot_delta=np.asarray(delta).reshape(-1).copy(),
        navigation_prior_included=False,point_prior_included=False,qualified=False)
    return reduced

def condition_finite_prior(*,covariance,jacobian,residual,unbounded_basis):
    """Condition one finite native joint Gaussian on whitened NEW rows only.

    No pseudo-inverse turns an unbounded prior into certainty. The Joseph form
    retains symmetry/PSD without eigenvalue clipping or added diagonal noise.
    """
    covariance=np.asarray(covariance,float);j=np.asarray(jacobian,float);r=np.asarray(residual,float)
    if (covariance.shape!=(12,12) or j.ndim!=2 or j.shape[1]!=12 or len(j)>256
            or r.shape!=(len(j),) or not all(np.isfinite(v).all() for v in [covariance,j,r])
            or not np.allclose(covariance,covariance.T,rtol=1e-12,atol=1e-15)):
        raise ValueError('finite_joint_navigation_gaussian_required')
    free=np.asarray(unbounded_basis,float)
    if free.shape!=(12,0):raise ValueError('unbounded_prior_requires_information_form')
    tolerance=64*np.finfo(float).eps*max(float(np.linalg.norm(covariance,2)),1e-30)
    if np.linalg.eigvalsh(covariance).min() < -tolerance:raise ValueError('positive_semidefinite_navigation_covariance_required')
    innovation=np.eye(len(j))+j@covariance@j.T
    gain=np.linalg.solve(innovation,j@covariance).T
    step=-gain@r;transition=np.eye(12)-gain@j
    updated=transition@covariance@transition.T+gain@gain.T
    updated=(updated+updated.T)/2
    return dict(step=step,covariance=updated,gain=gain,
                innovation_covariance=innovation,navigation_prior_count=1,
                point_prior_count=0,qualified=False)
