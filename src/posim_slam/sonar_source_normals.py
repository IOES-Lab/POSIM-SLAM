# SPDX-License-Identifier: Apache-2.0
"""POSIM range-grid normals for conditional, sensor-local acoustic models.

Adapted from POSIM RaySurfaceNormals.hh, Apache-2.0, at revision
ffecee9be9976f68c9e883d6cf3bbd6d40eb7729. This NumPy adaptation retains the
1.5 degree support, one-sided tangents and conditional local plane fit. The
third output component is ray incidence, as consumed by the acoustic shader.
It is not a normal expressed in the camera Cartesian basis.
"""
import numpy as np


def ray_surface_normals(ranges, azimuth_rad, elevation_rad, maximum_range_m):
    """Return float32 [elevation, azimuth, 3] normals in each ray's basis.

    The producer's float32 ranges/angles are represented explicitly. Nonfinite,
    nonpositive and out-of-range returns have zero normals. An isolated return
    supplies no normal. At most 25 neighbours enter each plane fit; temporary
    neighbour arrays are bounded by 512 centres per block.
    """
    raw, az, el, maximum = (np.asarray(v) for v in
                            (ranges, azimuth_rad, elevation_rad, maximum_range_m))
    if (raw.ndim != 2 or raw.dtype.kind not in 'fiu'
            or any(v.ndim != 1 or v.dtype.kind not in 'fiu' or not 1 <= len(v) <= 512
                   or not np.isfinite(v).all() for v in (az, el))
            or raw.shape != (len(el), len(az)) or raw.size > 262144
            or maximum.ndim != 0 or maximum.dtype.kind not in 'fiu'
            or not np.isfinite(maximum) or not 0 < maximum <= 200):
        raise ValueError('bounded_source_surface_grid_required')
    maximum_range_m = float(maximum)
    with np.errstate(over='ignore', invalid='ignore'):
        d = raw.astype(np.float32).astype(float)
        az, el = (v.astype(np.float32).astype(float) for v in (az, el))
    if any(not np.isfinite(v).all() or np.any(np.diff(v) <= 0) for v in (az, el)):
        raise ValueError('increasing_float32_source_angles_required')
    result = np.zeros((*raw.shape, 3), dtype=np.float32)
    if min(raw.shape) < 2:
        return result
    # std::cos(float)/std::sin(float) return float before the C++ helper
    # promotes their values to double for Cartesian tangent calculations.
    ca, sa, ce, se = (v.astype(np.float32).astype(float) for v in
                      (np.cos(az), np.sin(az), np.cos(el), np.sin(el)))
    rays = np.stack(np.broadcast_arrays(ce[:, None]*ca, ce[:, None]*sa,
                                        se[:, None]), axis=-1)
    valid = np.isfinite(d) & (d > 0) & (d <= maximum_range_m)
    points = np.where(valid[..., None], d[..., None], 0.) * rays
    tangent, supported = [], []
    # The negative side wins exact ties, matching the producer's iteration order.
    for axis in (1, 0):
        direction = np.zeros_like(points)
        shortest = np.full(raw.shape, np.inf)
        for side in (-1, 1):
            src = [slice(None), slice(None)]
            dst = [slice(None), slice(None)]
            src[axis] = slice(None, -1) if side < 0 else slice(1, None)
            dst[axis] = slice(1, None) if side < 0 else slice(None, -1)
            src, dst = tuple(src), tuple(dst)
            delta = (points[src] - points[dst]) * side
            squared = np.sum(delta*delta, axis=-1)
            take = valid[src] & valid[dst] & (squared > 1e-20) & (squared < shortest[dst])
            direction[dst] = np.where(take[..., None], delta, direction[dst])
            shortest[dst] = np.where(take, squared, shortest[dst])
        tangent.append(direction)
        supported.append(np.isfinite(shortest))
    normal = np.cross(*tangent)
    length = np.linalg.norm(normal, axis=-1)
    usable = valid & supported[0] & supported[1] & (length >= 1e-12)
    normal = np.divide(normal, length[..., None], out=np.zeros_like(normal),
                       where=usable[..., None])
    radius = np.deg2rad(1.5)

    def neighbourhoods(angles):
        targets = angles[:, None] + np.arange(-2, 3) * radius/2
        index = np.searchsorted(angles, targets)
        right = np.minimum(index, len(angles)-1)
        left = np.maximum(index-1, 0)
        take_left = (index == len(angles)) | ((index > 0)
                         & (targets-angles[left] < angles[right]-targets))
        index = np.where(take_left, left, right)
        distinct = np.ones_like(index, dtype=bool)
        distinct[:, 1:] = index[:, 1:] != index[:, :-1]
        return index, distinct

    rows, row_distinct = neighbourhoods(el)
    cols, col_distinct = neighbourhoods(az)
    centres = np.argwhere(usable)
    for start in range(0, len(centres), 512):
        r, c = centres[start:start+512].T
        ri, ci = rows[r, :, None], cols[c, None, :]
        delta = points[ri, ci] - points[r, c, None, None, :]
        reach = 3*d[r, c]*np.tan(radius)
        keep = (valid[ri, ci] & row_distinct[r, :, None] & col_distinct[c, None, :]
                & (np.sum(delta*delta, axis=-1) <= reach[:, None, None]**2))
        count = keep.sum(axis=(1, 2))
        mean = np.sum(np.where(keep[..., None], delta, 0.), axis=(1, 2))/count[:, None]
        q = np.where(keep[..., None], delta-mean[:, None, None, :], 0.)
        covariance = np.einsum('brci,brcj->bij', q, q)
        values, vectors = np.linalg.eigh(covariance)
        plane = ((count >= 6) & (values[:, 1] > 1e-12)
                 & (values[:, 0] <= .01*values[:, 1]))
        normal[r[plane], c[plane]] = vectors[plane, :, 0]
    incidence = np.sum(normal*rays, axis=-1)
    normal = np.where((incidence < 0)[..., None], -normal, normal)
    az_tangent = np.stack(np.broadcast_arrays(-sa, ca, np.zeros_like(sa)), axis=-1)
    el_tangent = np.stack(np.broadcast_arrays(-se[:, None]*ca, -se[:, None]*sa,
                                             ce[:, None]), axis=-1)
    result[..., 0] = np.sum(normal*az_tangent, axis=-1)
    result[..., 1] = np.sum(normal*el_tangent, axis=-1)
    result[..., 2] = np.clip(abs(incidence), 0., 1.)
    return result
