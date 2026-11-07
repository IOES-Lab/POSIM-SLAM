# SPDX-License-Identifier: Apache-2.0
"""Conditional directional-return mean calibration from known reference planes.

Each beam must keep the same effective return ray over its calibration/operating
branch. Known normals/distances are calibration references, not SLAM pose or
plane estimates counted as independent observations. A caller must validate the
physical branch. No noise model, native factor or map admission is invented.
"""
import numpy as np


def calibrate_return_directions(*,plane_normals,plane_distances_m,measured_ranges_m,
                                inverse_measurement_covariance=None,uncertainty_source=None):
    """Generalized linear least squares followed by per-beam unit normalization.

    For known unit normal n and positive perpendicular distance H, H/r=n·u.
    Ranges have shape [planes, beams]; normals are expressed in the sensor frame.
    Optional full covariance belongs to H/r, in BEAM-MAJOR order. Cross-beam
    and cross-plane errors remain in the propagated joint direction covariance.
    Calibration-reference uncertainty is not separately inferred here. The
    result is a normalized linear mean and first-order uncertainty, not a
    constrained nonlinear maximum-likelihood solution or physical validation.
    """
    n=np.asarray(plane_normals,dtype=float);h=np.asarray(plane_distances_m,dtype=float)
    ranges=np.asarray(measured_ranges_m,dtype=float)
    if (n.ndim!=2 or n.shape[1:]!=(3,) or not 3<=len(n)<=64
            or h.shape!=(len(n),) or ranges.ndim!=2 or ranges.shape[0]!=len(n)
            or not 1<=ranges.shape[1]<=16 or not all(np.isfinite(x).all() for x in (n,h,ranges))
            or not np.allclose(np.linalg.norm(n,axis=1),1.,rtol=0,atol=1e-9)
            or np.any(h<=0) or np.any(ranges<=0)):
        raise ValueError('bounded_known_plane_calibration_required')
    singular=np.linalg.svd(n,compute_uv=False)
    tolerance=np.finfo(float).eps*max(n.shape)*singular[0]
    rank=int(np.sum(singular>tolerance));beams=ranges.shape[1]
    design=np.kron(np.eye(beams),n)
    observed=(h[:,None]/ranges).T.reshape(-1)
    covariance=None;lower=None
    if inverse_measurement_covariance is not None:
        covariance=np.asarray(inverse_measurement_covariance,dtype=float)
        if (covariance.shape!=(len(observed),len(observed)) or not np.isfinite(covariance).all()
                or not np.allclose(covariance,covariance.T,rtol=1e-12,atol=1e-15)
                or not isinstance(uncertainty_source,str) or not uncertainty_source.strip()):
            raise ValueError('explicit_full_inverse_range_uncertainty_required')
        try:lower=np.linalg.cholesky(covariance)
        except np.linalg.LinAlgError as error:
            raise ValueError('positive_definite_inverse_range_covariance_required') from error
    base=dict(available=False,normal_rank=rank,normal_singular_values=singular,
              covariance_available=False,map_qualified=False,physical_branch_verified=False,
              known_calibration_references=True)
    if rank!=3:return dict(base,reason='insufficient_calibration_plane_excitation')
    a=design if lower is None else np.linalg.solve(lower,design)
    b=observed if lower is None else np.linalg.solve(lower,observed)
    estimate=np.linalg.lstsq(a,b,rcond=None)[0].reshape(beams,3)
    length=np.linalg.norm(estimate,axis=1)
    if np.any(length<1e-12):return dict(base,reason='direction_mean_unavailable')
    rays=estimate/length[:,None]
    result=dict(base,available=True,reason='conditional_normalized_direction_means',
         rays=rays,linear_solution_norms=length,
         inverse_range_residuals=(design@rays.reshape(-1)-observed).reshape(beams,len(n)).T,
         covariance_order='beam-major Cartesian ray coordinates',
         scope='Known-reference and fixed-return-branch conditional calibration only')
    if covariance is not None:
        inverse=np.linalg.lstsq(a,np.linalg.solve(lower,np.eye(len(observed))),rcond=None)[0]
        normalize=np.zeros((3*beams,3*beams))
        for beam,ray in enumerate(rays):
            normalize[3*beam:3*beam+3,3*beam:3*beam+3]=(np.eye(3)-np.outer(ray,ray))/length[beam]
        jacobian=normalize@inverse
        result.update(covariance_available=True,joint_ray_covariance=jacobian@covariance@jacobian.T,
           inverse_measurement_jacobian=jacobian,uncertainty_source=uncertainty_source)
    return result
