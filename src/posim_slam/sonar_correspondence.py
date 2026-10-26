"""Pose-conditioned search geometry for imaging-sonar correspondence.

An observed range/azimuth defines an elevation arc, not a 3D landmark. Project
the entire sampled arc into the next sensor frame. Caller-supplied poses must
share one metric map frame. Pose and measurement uncertainty are not estimated
here; this nominal locus must not be used as a calibrated rejection gate.
"""
import math
import numpy as np
from .dvl_scale import rotation, vector
from .sonar_factor import within_fov


def project_return_arc(*, measured_range, measured_azimuth, previous_position,
                       previous_attitude, current_position, current_attitude,
                       imu_to_sonar, sonar_to_imu, range_min, range_max,
                       horizontal_fov, vertical_fov, samples=65):
    """Return sampled elevation and predicted range/azimuth with support flags.

    Both poses are IMU-to-map transforms; lever and mount use the same contract
    as point_return_factor. No seabed plane, camera image or true pose is read.
    Sampling preserves both elevation branches but does not bound the continuous
    curve between samples. Invalid target support is explicit, never clamped.
    """
    values = (measured_range, measured_azimuth, range_min, range_max,
              horizontal_fov, vertical_fov)
    if (not all(math.isfinite(v) for v in values) or not 0 < range_min < range_max
            or not range_min <= measured_range <= range_max
            or not 0 < horizontal_fov <= math.pi or not 0 < vertical_fov < math.pi
            or not within_fov(measured_azimuth, horizontal_fov)):
        raise ValueError('invalid_sonar_measurement')
    if type(samples) is not int or not 3 <= samples <= 257 or samples % 2 != 1:
        raise ValueError('bounded_odd_arc_samples_required')
    p0, p1 = vector(previous_position), vector(current_position)
    r0, r1 = rotation(previous_attitude), rotation(current_attitude)
    lever, mount = vector(imu_to_sonar), rotation(sonar_to_imu)
    elevations = np.linspace(-vertical_fov/2, vertical_fov/2, samples)
    rays = measured_range * np.column_stack((np.cos(elevations)*math.cos(measured_azimuth),
        np.cos(elevations)*math.sin(measured_azimuth), np.sin(elevations)))
    # Current sonar coordinates, including rotating sensor lever arms.
    relative_rotation = mount.T @ r1.T @ r0 @ mount
    relative_translation = mount.T @ (r1.T @ (p0-p1+r0@lever)-lever)
    local = rays @ relative_rotation.T + relative_translation
    ranges = np.linalg.norm(local, axis=1)
    horizontal = np.linalg.norm(local[:, :2], axis=1)
    azimuths = np.arctan2(local[:, 1], local[:, 0])
    target_elevations = np.arctan2(local[:, 2], horizontal)
    tolerance = 4*float(np.finfo(np.float32).eps)
    supported = ((local[:, 0] > 0) & (horizontal > 1e-6)
        & (ranges >= range_min) & (ranges <= range_max)
        & (abs(azimuths) <= horizontal_fov/2+tolerance)
        & (abs(target_elevations) <= vertical_fov/2+tolerance))
    return dict(source_elevation_rad=elevations, range_azimuth=np.column_stack((ranges, azimuths)),
        target_elevation_rad=target_elevations, supported=supported,
        scope='nominal fixed-pose search locus; not an association or uncertainty gate',
        elevation_resolved=False)
