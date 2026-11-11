# SPDX-License-Identifier: Apache-2.0
# Cubemap mapping adapted from Gazebo Rendering, Copyright 2018 OSRF.
import numpy as np


def cubemap_rays(azimuth, elevation, size):
    """Adapt GZ Ogre2GpuRays 10.0.2 SampleCubemap / nearest filtering.

    Copyright 2018 Open Source Robotics Foundation, Apache-2.0. Source:
    https://github.com/gazebosim/gz-rendering/blob/gz-rendering10_10.0.2/ogre2/src/Ogre2GpuRays.cc
    The standard cube is Y-up; local sonar coordinates are X-forward/Z-up.
    Texel centres are used to reconstruct the first-pass camera rays.
    """
    az, el = np.asarray(azimuth), np.asarray(elevation)
    local = np.stack(np.broadcast_arrays(np.cos(el[:, None])*np.cos(az),
        np.cos(el[:, None])*np.sin(az), np.sin(el[:, None])), axis=-1)
    cube = local[..., [1, 2, 0]].copy()
    cube[..., 0] *= -1
    v = cube.reshape(-1, 3)
    axis = abs(v).argmax(axis=1)
    face = np.empty(len(v), dtype=np.int32)
    uv = np.empty((len(v), 2))
    for a in range(3):
        keep = axis == a
        x, y, z = v[keep].T
        if a == 2:
            face[keep] = np.where(z < 0, 5, 4)
            uv[keep] = np.column_stack((np.where(z < 0, -x, x), -y)) / (2*abs(z[:, None])) + .5
        elif a == 1:
            face[keep] = np.where(y < 0, 3, 2)
            uv[keep] = np.column_stack((x, np.where(y < 0, -z, z))) / (2*abs(y[:, None])) + .5
        else:
            face[keep] = np.where(x < 0, 1, 0)
            uv[keep] = np.column_stack((np.where(x < 0, z, -z), -y)) / (2*abs(x[:, None])) + .5
    # The producer stores UVs as RGBA32_FLOAT, then samples without filtering.
    uv = uv.astype(np.float32).astype(float)
    texels = np.clip(np.floor(uv*size).astype(int), 0, size-1)
    u, w = (2*(texels+.5)/size-1).T
    actual = np.empty_like(v)
    mapping = {0: np.column_stack((np.ones(len(v)), -w, -u)),
               1: np.column_stack((-np.ones(len(v)), -w, u)),
               2: np.column_stack((u, np.ones(len(v)), w)),
               3: np.column_stack((u, -np.ones(len(v)), -w)),
               4: np.column_stack((u, -w, np.ones(len(v)))),
               5: np.column_stack((-u, -w, -np.ones(len(v))))}
    for f, value in mapping.items():
        actual[face == f] = value[face == f]
    actual /= np.linalg.norm(actual, axis=1)[:, None]
    result = actual[:, [2, 0, 1]]
    result[:, 1] *= -1
    return result, np.column_stack((face, texels))
