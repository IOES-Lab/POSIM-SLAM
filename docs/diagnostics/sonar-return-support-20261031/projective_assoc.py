"""Nominal full-elevation association diagnostic, without point initialization.

Reuses public POSIM-SLAM arc geometry. Pixel radii are empirical search choices,
not calibrated covariance gates. No pose uncertainty or physical identity claim.
"""
import math
import numpy as np
from posim_slam.sonar_correspondence import project_return_arc


def arc_pixel_distance(pixels, locus, supported):
    """Minimum distance to supported samples/segments; never bridge support gaps."""
    pixels = np.asarray(pixels, dtype=float).reshape(-1, 2)
    locus = np.asarray(locus, dtype=float)
    supported = np.asarray(supported, dtype=bool) & np.isfinite(locus).all(axis=1)
    if not supported.any():
        return np.full(len(pixels), np.inf)
    result = np.linalg.norm(pixels[:, None, :] - locus[None, supported, :], axis=2).min(axis=1)
    valid = supported[:-1] & supported[1:]
    starts, ends = locus[:-1][valid], locus[1:][valid]
    if len(starts):
        direction = ends-starts
        length2 = np.einsum('ij,ij->i', direction, direction)
        nonzero = length2 > 1e-20
        starts, direction, length2 = starts[nonzero], direction[nonzero], length2[nonzero]
        if len(starts):
            relative = pixels[:, None, :]-starts
            fraction = np.clip(np.einsum('pij,ij->pi', relative, direction)/length2, 0, 1)
            residual = relative-fraction[:, :, None]*direction
            result = np.minimum(result, np.linalg.norm(residual, axis=2).min(axis=1))
    return result


def unique_matches(distances, match_radius=2., new_radius=5.):
    distances = np.asarray(distances, dtype=float)
    if (distances.ndim != 2 or np.isnan(distances).any() or (distances < 0).any()
            or max(distances.shape, default=0) > 300
            or not math.isfinite(match_radius) or not math.isfinite(new_radius)
            or not 0 < match_radius < new_radius):
        raise ValueError('invalid_projective_search')
    allowed = distances <= match_radius
    current, database = np.nonzero(allowed)
    mutual = (allowed.sum(axis=1)[current] == 1) & (allowed.sum(axis=0)[database] == 1)
    pairs = list(zip(current[mutual].tolist(), database[mutual].tolist()))
    nearest = distances.min(axis=1) if distances.shape[1] else np.full(distances.shape[0], np.inf)
    novel = np.flatnonzero(nearest > new_radius).tolist()
    return pairs, novel


def project_pixels(observation, previous_pose, current_pose, geometry, ranges, angles, samples):
    arc = project_return_arc(measured_range=observation['range_m'],
        measured_azimuth=observation['azimuth_rad'], previous_position=previous_pose['position'],
        previous_attitude=previous_pose['attitude'], current_position=current_pose['position'],
        current_attitude=current_pose['attitude'], samples=samples, **geometry)
    polar = arc['range_azimuth']
    pixels = np.column_stack((np.interp(polar[:, 1], angles, np.arange(len(angles)), left=np.nan, right=np.nan),
                              np.interp(polar[:, 0], ranges, np.arange(len(ranges)), left=np.nan, right=np.nan)))
    return pixels, arc['supported'] & np.isfinite(pixels).all(axis=1)


class ProjectiveTracks:
    def __init__(self, geometry, *, samples=257, match_radius=2., new_radius=5., max_unseen_ns=2_000_000_000):
        if type(samples) is not int or not 3 <= samples <= 257 or samples % 2 != 1:
            raise ValueError('bounded_odd_arc_samples_required')
        unique_matches(np.empty((0,0)), match_radius, new_radius)
        if type(max_unseen_ns) is not int or not 0 < max_unseen_ns <= 5_000_000_000:
            raise ValueError('bounded_feature_age_required')
        self.geometry, self.samples = geometry, samples
        self.match_radius, self.new_radius, self.max_unseen_ns = match_radius, new_radius, max_unseen_ns
        self.database, self.tracks, self.rows = {}, {}, []
        self.next_id, self.last_stamp = 0, -1

    def push(self, stamp, pose, pixels, observations, ranges, angles):
        if type(stamp) is not int or stamp <= self.last_stamp or len(pixels) > 300 or len(pixels) != len(observations):
            raise ValueError('ordered_bounded_projective_frame_required')
        self.database = {k:v for k,v in self.database.items() if stamp-v['last_stamp'] <= self.max_unseen_ns}
        keys = sorted(self.database)
        matrix = np.full((len(pixels),len(keys)), np.inf)
        for index, ident in enumerate(keys):
            track = self.database[ident]
            locus, supported = project_pixels(track['observations'][0], track['base_pose'], pose,
                self.geometry, ranges, angles, self.samples)
            matrix[:, index] = arc_pixel_distance(pixels, locus, supported)
        pairs, novel = unique_matches(matrix, self.match_radius, self.new_radius)
        current = []
        for pixel_id, database_id in pairs:
            track = self.database[keys[database_id]]
            track['observations'].append(dict(stamp_ns=stamp, **observations[pixel_id]))
            track['last_stamp'] = stamp
            current.append(dict(track_id=keys[database_id], pixel_index=pixel_id, length=len(track['observations'])))
        overflow = 0
        for pixel_id in novel:
            if len(self.database) >= 300:
                overflow += 1
                continue
            ident = self.next_id
            self.next_id += 1
            track = dict(base_pose=pose, last_stamp=stamp,
                         observations=[dict(stamp_ns=stamp, **observations[pixel_id])])
            self.database[ident] = self.tracks[ident] = track
            current.append(dict(track_id=ident, pixel_index=pixel_id, length=1))
        self.last_stamp = stamp
        row = dict(stamp_ns=stamp, detected=len(pixels), matched=len(pairs), new=len(novel)-overflow,
                   ambiguous=len(pixels)-len(pairs)-len(novel), capacity_rejected=overflow,
                   database=len(self.database), observations=current)
        self.rows.append(row)
        return row
