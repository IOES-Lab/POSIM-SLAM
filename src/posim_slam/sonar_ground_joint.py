"""Correlated raw entry-range rows sharing an unknown local seabed plane.

The fitted plane initializes a nuisance variable, never an independent prior.
Association, aperture branch, selected beam identities and range covariance are
caller responsibilities. This block is not a standalone navigation estimator.
"""
import math
import numpy as np
from .dvl_scale import rotation, vector
from .inertial_filter import covariance
from .ping_factor import plane_range_factor
from .sonar_surface import ground_entry_band


def linearize_ground_patch(*, observations, states, plane_normal, plane_distance,
                           measurement_covariance, imu_to_sonar, sonar_to_imu,
                           vertical_fov, range_min, range_max):
    """Whiten raw range residuals with full spatial AND temporal covariance.

    Each exact-time observation supplies ordered azimuth_rad, range_m, beam_ids
    and boundary_sign (-1 or +1). Columns: each pose's world translation/right
    body rotation, then shared plane tangent rotation (2) and distance (1).
    Covariance rows follow frame order then beam order. All geometry must remain
    on the selected first-intersection aperture branch at the current estimate.
    No fitted-normal observation, gravity prior or known terrain is added.
    """
    if not 1 <= len(observations) <= 16:
        raise ValueError('bounded_ground_patch_required')
    stamps = [o['stamp_ns'] for o in observations]
    if (any(type(t) is not int or t <= 0 for t in stamps)
            or any(b <= a for a, b in zip(stamps, stamps[1:]))):
        raise ValueError('ordered_ground_stamps_required')
    if not math.isfinite(vertical_fov) or not 0 < vertical_fov < math.pi:
        raise ValueError('invalid_ground_aperture')
    normal = vector(plane_normal)
    if not np.isclose(normal @ normal, 1., atol=1e-8, rtol=0):
        raise ValueError('unit_plane_normal_required')
    mount = rotation(sonar_to_imu); lever = vector(imu_to_sonar)
    axis = np.eye(3)[np.argmin(abs(normal))]
    tangent = np.cross(normal, axis); tangent /= np.linalg.norm(tangent)
    basis = np.column_stack([tangent, np.cross(normal, tangent)])
    count = sum(len(o['range_m']) for o in observations)
    if not 8 <= count <= 512:
        raise ValueError('ground_patch_row_budget')
    noise = covariance(measurement_covariance, count)
    j = np.zeros((count, 6 * len(stamps) + 3)); residual = np.zeros(count)
    offset = 0
    for i, o in enumerate(observations):
        az = np.asarray(o['azimuth_rad'], dtype=float)
        ranges = np.asarray(o['range_m'], dtype=float)
        ids = o['beam_ids']; sign = o['boundary_sign']
        if (az.shape != ranges.shape or az.ndim != 1 or not 8 <= len(az) <= 128
                or len(ids) != len(az) or any(type(k) is not int or k < 0 for k in ids)
                or len(set(ids)) != len(ids) or not np.isfinite(az).all()
                or np.any(np.diff(az) <= 0) or np.any(abs(az) >= math.pi / 2)
                or type(sign) is not int or sign not in (-1, 1)):
            raise ValueError('invalid_ground_beam_identity_or_geometry')
        if o['stamp_ns'] not in states:
            raise ValueError('ground_state_timestamp_missing')
        state = states[o['stamp_ns']]
        r = rotation(state['attitude']); p = vector(state['position'])
        local_normal = (r @ mount).T @ normal
        local_distance = plane_distance - normal @ (p + r @ lever)
        entry = ground_entry_band(az, vertical_fov, local_normal, local_distance)[:, 0]
        elevation = sign * vertical_fov / 2
        rays = np.column_stack([np.cos(az) * math.cos(elevation),
                                np.sin(az) * math.cos(elevation),
                                np.full(len(az), math.sin(elevation))])
        for k, ray in enumerate(rays):
            f = plane_range_factor(position=p, attitude=r, imu_to_ping=lever,
                ping_to_imu=mount, rays=ray[None], plane_normal=normal,
                plane_distance=plane_distance, measured_range=ranges[k], range_bias=0.,
                range_min=range_min, range_max=range_max)
            if not np.isclose(f['predicted_range'], entry[k], atol=1e-7, rtol=0):
                raise ValueError('ground_first_return_branch_changed')
            residual[offset] = -f['residual']
            j[offset, 6*i:6*i+6] = f['pose_jacobians'][0]
            j[offset, -3:-1] = f['plane_jacobians'][0, :3] @ basis
            j[offset, -1] = f['plane_jacobians'][0, 3]
            offset += 1
    lower = np.linalg.cholesky(noise)
    residual = np.linalg.solve(lower, residual); j = np.linalg.solve(lower, j)
    if not np.isfinite(residual).all() or not np.isfinite(j).all():
        raise ValueError('nonfinite_ground_linearization')
    return dict(residual=residual, jacobian=j, normal_basis=basis,
                state_stamps_ns=stamps, retained_columns=6*len(stamps))
