# SPDX-License-Identifier: Apache-2.0
"""Conditional mean-power rendering of finite, opaque acoustic surfaces.

Geometry is supplied by the caller, not measured by this model. Nearest ray
intersections determine visibility independently of reflectivity. Ideal triangle
normals approximate local surfaces; rasterization, source normal estimation,
multiple paths, quantization and joint pose uncertainty are excluded. This is an
experimental observation model, not a map or an estimator admission test.
"""
import numpy as np


class TriangleSurface:
    """Bounded median-split BVH, with two-sided ray/triangle intersections."""

    def __init__(self, triangles):
        t = np.asarray(triangles)
        if (t.ndim != 3 or t.shape[1:] != (3, 3) or not 1 <= len(t) <= 65536
                or t.dtype.kind not in 'fiu' or not np.isfinite(t).all()):
            raise ValueError('bounded_finite_triangles_required')
        t = t.astype(float, copy=True)
        self.triangles = t
        self.a = t[:, 0]
        self.e1 = t[:, 1] - self.a
        self.e2 = t[:, 2] - self.a
        normal = np.cross(self.e1, self.e2)
        length = np.linalg.norm(normal, axis=1)
        if np.any(length <= 1e-12):
            raise ValueError('nondegenerate_triangles_required')
        self.normals = normal / length[:, None]
        self.double_areas = length
        for value in (t, self.e1, self.e2, self.normals, self.double_areas):
            value.setflags(write=False)
        centers = t.mean(axis=1)
        self.nodes = []

        def build(ids):
            lo, hi = t[ids].min(axis=(0, 1)), t[ids].max(axis=(0, 1))
            index = len(self.nodes)
            self.nodes.append(None)
            if len(ids) <= 12:
                self.nodes[index] = (lo, hi, None, None, ids)
            else:
                axis = np.argmax(np.ptp(centers[ids], axis=0))
                ordered = ids[np.argsort(centers[ids, axis], kind='stable')]
                mid = len(ids) // 2
                left, right = build(ordered[:mid]), build(ordered[mid:])
                self.nodes[index] = (lo, hi, left, right, None)
            return index

        build(np.arange(len(t)))

    def first_hits(self, origin, directions, maximum_range_m):
        origin = np.asarray(origin, dtype=float)
        d = np.asarray(directions, dtype=float)
        if (origin.shape != (3,) or not np.isfinite(origin).all()
                or d.ndim != 2 or d.shape[1:] != (3,) or not 1 <= len(d) <= 262144
                or not np.isfinite(d).all()
                or not np.allclose(np.linalg.norm(d, axis=1), 1., atol=1e-9, rtol=0)
                or not np.isfinite(maximum_range_m) or not 0 < maximum_range_m <= 200):
            raise ValueError('bounded_unit_rays_required')
        best = np.full(len(d), np.inf)
        which = np.full(len(d), -1, dtype=np.int32)
        offset = origin - self.a
        q = np.cross(offset, self.e1)
        numerator = np.einsum('ij,ij->i', self.e2, q)
        stack = [(0, np.arange(len(d)))]
        while stack:
            node, ids = stack.pop()
            lo, hi, left, right, triangles = self.nodes[node]
            near = np.full((len(ids), 3), -np.inf)
            far = np.full_like(near, np.inf)
            for axis in range(3):
                v = d[ids, axis]
                moving = v != 0
                a = np.divide(lo[axis] - origin[axis], v, out=np.zeros(len(v)), where=moving)
                b = np.divide(hi[axis] - origin[axis], v, out=np.zeros(len(v)), where=moving)
                near[:, axis] = np.where(moving, np.minimum(a, b), -np.inf)
                far[:, axis] = np.where(moving, np.maximum(a, b), np.inf)
                if not lo[axis] <= origin[axis] <= hi[axis]:
                    far[~moving, axis] = -np.inf
            enter = np.maximum(near.max(axis=1), 0.)
            ids = ids[(far.min(axis=1) >= enter)
                      & (enter <= np.minimum(best[ids], maximum_range_m))]
            if not len(ids):
                continue
            if triangles is None:
                stack.extend(((right, ids), (left, ids)))
                continue
            for start in range(0, len(ids), 1024):
                selected = ids[start:start + 1024]
                rays = d[selected]
                h = np.cross(rays[:, None], self.e2[triangles])
                det = np.einsum('ij,rij->ri', self.e1[triangles], h)
                valid = abs(det) > 1e-12
                inv = np.divide(1., det, out=np.zeros_like(det), where=valid)
                u = np.einsum('ij,rij->ri', offset[triangles], h) * inv
                v = (rays @ q[triangles].T) * inv
                distance = numerator[triangles][None, :] * inv
                valid &= ((u >= -1e-10) & (v >= -1e-10) & (u + v <= 1 + 1e-10)
                          & (distance > 1e-8) & (distance <= maximum_range_m))
                distance = np.where(valid, distance, np.inf)
                index = distance.argmin(axis=1)
                value = distance[np.arange(len(selected)), index]
                take = value < best[selected]
                best[selected[take]] = value[take]
                which[selected[take]] = triangles[index[take]]
        return best, which


class SurfacePowerObservation:
    """Linear reflectivity operator conditional on frozen visible geometry."""

    def __init__(self, model, surface, position, rotation):
        p, r = np.asarray(position, dtype=float), np.asarray(rotation, dtype=float)
        if (p.shape != (3,) or r.shape != (3, 3) or not np.isfinite(p).all()
                or not np.isfinite(r).all()
                or not np.allclose(r.T @ r, np.eye(3), atol=1e-8, rtol=0)
                or not np.isclose(np.linalg.det(r), 1., atol=1e-8, rtol=0)):
            raise ValueError('rigid_surface_observation_pose_required')
        directions = model.directions @ r.T
        ranges, ids = surface.first_hits(p, directions, model.maximum_range_m)
        self.ranges = ranges.reshape(model.ray_shape)
        self.hit_triangles = ids.reshape(model.ray_shape)
        keep = (ids >= 0) & (ranges >= model.minimum_range_m)
        self.triangle_ids = ids[keep]
        self.beam_ids = np.flatnonzero(keep) % model.nbeams
        distance = ranges[keep]
        self.selected_directions = directions[keep]
        self.incidence = np.einsum('ij,ij->i', directions[keep], surface.normals[ids[keep]])
        self.range_gain = (model.power_constant * distance ** -3
                           * np.exp(-4 * model.alpha * distance))
        self.unit_power = self.range_gain * self.incidence ** 2
        self.selected_ranges = distance
        endpoint = p + distance[:, None] * self.selected_directions
        offset = endpoint - surface.a[self.triangle_ids]
        e1, e2 = surface.e1[self.triangle_ids], surface.e2[self.triangle_ids]
        aa = np.sum(e1 * e1, axis=1)
        ab = np.sum(e1 * e2, axis=1)
        bb = np.sum(e2 * e2, axis=1)
        ca = np.sum(offset * e1, axis=1)
        cb = np.sum(offset * e2, axis=1)
        determinant = surface.double_areas[self.triangle_ids] ** 2
        u, v = (bb * ca - ab * cb) / determinant, (aa * cb - ab * ca) / determinant
        self.barycentric = np.column_stack((1 - u - v, u, v))
        self.surface = surface
        coordinate = distance / model.fine_step_m
        self.r0 = np.floor(coordinate).astype(np.int32) % model.fine_bins
        self.r1 = (self.r0 + 1) % model.fine_bins
        self.fraction = coordinate - np.floor(coordinate)
        self.model = model
        self.size = len(surface.triangles)
        self.shape = model.shape

    def forward(self, reflectivity):
        mu = np.asarray(reflectivity, dtype=float)
        if mu.shape != (self.size,) or not np.isfinite(mu).all():
            raise ValueError('finite_surface_coefficient_vector_required')
        m = self.model
        values = mu[self.triangle_ids] * self.unit_power
        image = np.zeros(m.fine_bins * m.nbeams)
        for ri, weight in ((self.r0, 1 - self.fraction), (self.r1, self.fraction)):
            image += np.bincount(ri * m.nbeams + self.beam_ids,
                                 weights=weight * values, minlength=len(image))
        return (m.range_mix @ image.reshape(m.fine_bins, m.nbeams)) @ m.beam_mix.T

    def adjoint(self, image):
        image = np.asarray(image, dtype=float)
        if image.shape != self.shape or not np.isfinite(image).all():
            raise ValueError('finite_surface_image_required')
        m = self.model
        back = m.range_mix.T @ (image @ m.beam_mix)
        values = ((1 - self.fraction) * back[self.r0, self.beam_ids]
                  + self.fraction * back[self.r1, self.beam_ids]) * self.unit_power
        return np.bincount(self.triangle_ids, weights=values, minlength=self.size)

    def geometry_adjoint(self, image, reflectivity):
        """Vertex derivative within the current first-hit/interpolation branch.

        Includes changes to range, attenuation and unit face normals. It excludes
        silhouette/visibility-cut derivatives. Re-trace every candidate geometry
        step and check the actual objective; this is not a visibility surrogate
        or a joint geometry/pose covariance. Shared vertices must be accumulated
        by the caller's mesh topology.
        """
        image = np.asarray(image, dtype=float)
        mu = np.asarray(reflectivity, dtype=float)
        if (image.shape != self.shape or not np.isfinite(image).all()
                or mu.shape != (self.size,) or not np.isfinite(mu).all()):
            raise ValueError('finite_surface_geometry_derivative_inputs_required')
        m, s, ids = self.model, self.surface, self.triangle_ids
        back = m.range_mix.T @ (image @ m.beam_mix)
        response = ((1 - self.fraction) * back[self.r0, self.beam_ids]
                    + self.fraction * back[self.r1, self.beam_ids])
        slope = (back[self.r1, self.beam_ids] - back[self.r0, self.beam_ids]) / m.fine_step_m
        radial = mu[ids] * self.unit_power * (
            (-3 / self.selected_ranges - 4 * m.alpha) * response + slope)
        gradient = np.zeros((self.size, 3, 3))
        motion = radial[:, None] * s.normals[ids] / self.incidence[:, None]
        for vertex in range(3):
            np.add.at(gradient[:, vertex], ids, self.barycentric[:, vertex, None] * motion)
        normal_gradient = (2 * mu[ids] * self.range_gain * self.incidence * response)[:, None] * self.selected_directions
        cross_gradient = (normal_gradient - s.normals[ids]
                          * np.sum(normal_gradient * s.normals[ids], axis=1)[:, None])
        cross_gradient /= s.double_areas[ids, None]
        first = np.cross(s.e2[ids], cross_gradient)
        second = np.cross(cross_gradient, s.e1[ids])
        for vertex, value in ((0, -first - second), (1, first), (2, second)):
            np.add.at(gradient[:, vertex], ids, value)
        return gradient


class SurfacePowerModel:
    """Source-profile ensemble mean, using NumPy and a bounded Hamming table.

    Azimuth/elevation are the sampled spherical ray angles. Range coordinates
    follow c/(2*bandwidth), starting at zero. Source level is in dB re 1 microPa;
    fixed absorption is in dB/m. This profile uses every ray and the selected
    POSIM source's (ray_skips + 1) area factor, hence a factor of two. It does not
    supply a commercial device calibration. Grouping averages power after beam
    mixing, preserving the recorded diagnostic's 4-range/3-beam convention.
    """

    def __init__(self, azimuth_rad, elevation_rad, native_ranges_m, *,
                 minimum_range_m=.2, maximum_range_m=10., source_level_db=220.,
                 sound_speed_m_s=1500., absorption_db_m=.0354,
                 fractional_samples=8, range_group=1, beam_group=1):
        az, el, rr = (np.array(x, dtype=float, copy=True)
                      for x in (azimuth_rad, elevation_rad, native_ranges_m))
        if (any(x.ndim != 1 or not np.isfinite(x).all() or not np.all(np.diff(x) > 0)
                for x in (az, el, rr)) or not 3 <= len(az) <= 512
                or not 2 <= len(el) <= 512 or not 2 <= len(rr) <= 1024
                or len(az) * len(el) > 262144 or rr[0] != 0
                or np.max(abs(az)) >= np.pi / 2 or np.max(abs(el)) >= np.pi / 2
                or fractional_samples not in (4, 8, 16)
                or any(type(x) is not int or x < 1 for x in (range_group, beam_group))
                or range_group > len(rr) or beam_group > len(az)
                or not np.isfinite([minimum_range_m, maximum_range_m, source_level_db,
                                    sound_speed_m_s, absorption_db_m]).all()
                or not 0 < minimum_range_m < maximum_range_m <= 200
                or not 0 <= source_level_db <= 240 or not 1000 <= sound_speed_m_s <= 2000
                or not 0 <= absorption_db_m <= 10):
            raise ValueError('bounded_surface_acoustic_profile_required')
        steps = [(x[-1] - x[0]) / (len(x) - 1) for x in (az, el, rr)]
        for x, step in zip((az, el, rr), steps):
            # Source float32 start + index*step can cancel near angular zero.
            # Bound its arithmetic by the full interval, not the local value.
            if not np.allclose(x, x[0] + np.arange(len(x)) * step,
                               rtol=0, atol=4 * np.finfo(np.float32).eps
                               * max(abs(x[0]), abs(x[-1]), step)):
                raise ValueError('uniform_source_ray_and_range_grids_required')
        if maximum_range_m > len(rr) * steps[2]:
            raise ValueError('surface_range_alias_period_exceeded')
        self.nbeams = len(az)
        self.azimuth_rad = az
        self.native_ranges_m = rr
        self.fractional_samples = fractional_samples
        for value in (self.azimuth_rad, self.native_ranges_m):
            value.setflags(write=False)
        self.ray_shape = (len(el), len(az))
        self.minimum_range_m, self.maximum_range_m = minimum_range_m, maximum_range_m
        self.directions = np.stack(np.broadcast_arrays(
            np.cos(el[:, None]) * np.cos(az), np.cos(el[:, None]) * np.sin(az),
            np.sin(el[:, None])), axis=-1).reshape(-1, 3)
        self.fine_bins = len(rr) * fractional_samples
        self.fine_step_m = steps[2] / fractional_samples
        if len(rr) * self.fine_bins > 8_388_608:
            raise MemoryError('surface_response_table_budget')
        f = np.arange(len(rr))
        window = .54 - .46 * np.cos(2 * np.pi * (f + 1) / len(rr))
        window /= np.linalg.norm(window)
        response = abs(np.fft.fft(window, n=self.fine_bins)) ** 2 / len(rr)
        columns = np.arange(self.fine_bins)
        starts = np.arange(0, len(rr), range_group)
        self.range_mix = np.array([
            response[(np.arange(start, min(start + range_group, len(rr)))[:, None]
                      * fractional_samples - columns) % self.fine_bins].mean(axis=0)
            for start in starts])
        amplitude = abs(np.sinc(.884 / steps[0] * np.sin(az[:, None] - az)))
        power = amplitude ** 2 / np.sum(amplitude ** 2)
        self.beam_mix = np.array([power[i:i + beam_group].mean(axis=0)
                                  for i in range(0, len(az), beam_group)])
        bandwidth = sound_speed_m_s / (2 * steps[2])
        area = 2 * steps[0] * steps[1]
        self.power_constant = (10 ** (source_level_db / 10) * 1e-12 * area
                               * len(rr) * (bandwidth / len(rr)) ** 2)
        self.alpha = absorption_db_m * np.log(10) / 20
        self.shape = (len(self.range_mix), len(self.beam_mix))

    def observation(self, surface, position, rotation):
        return SurfacePowerObservation(self, surface, position, rotation)
