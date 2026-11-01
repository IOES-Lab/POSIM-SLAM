# SPDX-License-Identifier: Apache-2.0
# Cubemap mapping adapted from Gazebo Rendering, Copyright 2018 OSRF.
import numpy as np
from .dvl_scale import rotation


def cubemap_rays(azimuth, elevation, size):
    """Adapt GZ Ogre2GpuRays 10.0.2 SampleCubemap / nearest filtering.

    Copyright 2018 Open Source Robotics Foundation, Apache-2.0. Source:
    https://github.com/gazebosim/gz-rendering/blob/gz-rendering10_10.0.2/ogre2/src/Ogre2GpuRays.cc
    The standard cube is Y-up; local sonar coordinates are X-forward/Z-up.
    Texel centres are used to reconstruct the first-pass camera rays.
    """
    az, el = np.asarray(azimuth), np.asarray(elevation)
    if (az.ndim!=1 or el.ndim!=1 or not 1<=len(az)<=2049 or not 1<=len(el)<=2049
            or len(az)*len(el)>262144 or az.dtype.kind not in 'fiu' or el.dtype.kind not in 'fiu'
            or not np.isfinite(az).all() or not np.isfinite(el).all()
            or np.any(abs(az)>np.pi) or np.any(abs(el)>np.pi/2)
            or type(size) is not int or not 128<=size<=1024 or size&(size-1)):
        raise ValueError('bounded_gazebo_cubemap_calibration_required')
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


def dvl_beam_rays(axes,apertures_rad,scan_spacing_m_at_1m,*,reference_to_sensor,
                  gz_sensors_version,gz_rendering_version):
    """Calibrate minimum-range sample sets for explicit Gazebo source versions.

    Axes are the original sensor-FLU calibration, without reference-frame
    round trips or rounding; returned directions use the DVL reference frame.
    reference_to_sensor maps that output frame into sensor FLU. The spacing
    is Gazebo's SDF resolution at unit distance, not range variance/accuracy.
    Nominal directions select cone membership; sampled directions intersect
    geometry. No sensor measurement, bias/noise calibration or XYZ is produced.
    """
    if (gz_sensors_version,gz_rendering_version)!=('10.1.1','10.0.2'):
        raise ValueError('qualified_gazebo_sampling_source_profile_required')
    a=np.asarray(axes);aperture=np.asarray(apertures_rad);r=rotation(reference_to_sensor)
    spacing=scan_spacing_m_at_1m
    if (a.ndim!=2 or a.shape[1:]!=(3,) or not 3<=len(a)<=16 or a.dtype.kind not in 'fiu'
            or aperture.shape!=(len(a),) or aperture.dtype.kind not in 'fiu'
            or not np.isfinite(a).all() or not np.isfinite(aperture).all()
            or not np.allclose(np.linalg.norm(a,axis=1),1.,rtol=0,atol=1e-8)
            or np.any(aperture<=0) or np.any(aperture>=np.pi)
            or isinstance(spacing,(bool,np.bool_)) or not np.isscalar(spacing)
            or not np.isfinite(spacing) or not 0<spacing<1):
        raise ValueError('bounded_dvl_sampling_calibration_required')
    beam_to_sensor=np.array([[0.,0.,1.],[0.,1.,0.],[-1.,0.,0.]])
    local=a@beam_to_sensor
    if np.any(local[:,0]<=0):raise ValueError('forward_dvl_beam_frame_required')
    az=np.arctan2(local[:,1],local[:,0]);el=np.arctan2(local[:,2],np.linalg.norm(local[:,:2],axis=1))
    half=aperture/2;extent=np.max(np.c_[abs(az)+half,abs(el)+half],axis=0)
    if np.any(extent>=np.pi/2):raise ValueError('bounded_dvl_sampling_field_required')
    dimensions=np.ceil(2*extent/spacing)
    if np.any(dimensions>2048):raise ValueError('bounded_dvl_sampling_workspace_required')
    dimensions=dimensions.astype(int);dimensions+=1-dimensions%2
    width,height=map(int,dimensions)
    if width<3 or height<3 or width*height>262144:
        raise ValueError('bounded_dvl_sampling_workspace_required')
    horizontal=np.linspace(-extent[0],extent[0],width)
    vertical=np.linspace(-extent[1],extent[1],height)
    nominal=np.stack(np.broadcast_arrays(np.cos(vertical[:,None])*np.cos(horizontal),
        np.cos(vertical[:,None])*np.sin(horizontal),np.sin(vertical[:,None])),axis=-1).reshape(-1,3)
    pixels=[int(dimensions[i]) if 2*extent[i]<np.pi/2 else int(np.pi/(4*extent[i])*dimensions[i]) for i in range(2)]
    cube_size=int(np.clip(2**int(np.ceil(np.log2(max(pixels)))),128,1024))
    sampled,texels=cubemap_rays(horizontal,vertical,cube_size)
    x,y=np.meshgrid(np.arange(width),np.arange(height));beams=[]
    for axis,azimuth,elevation,opening in zip(local,az,el,half):
        lo=((np.array([azimuth-opening,elevation-opening])+extent)/(2*extent/(dimensions-1))).astype(int)
        hi=((np.array([azimuth+opening,elevation+opening])+extent)/(2*extent/(dimensions-1))).astype(int)
        patch=((x>=lo[0])&(x<hi[0])&(y>=lo[1])&(y<hi[1])).reshape(-1)
        selected=np.flatnonzero(patch&(nominal@axis>np.cos(opening)))
        if not 1<=len(selected)<=512:raise ValueError('bounded_nonempty_dvl_beam_samples_required')
        beams.append(dict(nominal_directions=nominal[selected]@beam_to_sensor.T@r,
            ray_directions=sampled[selected]@beam_to_sensor.T@r,
            scan_indices=selected,texels=texels[selected]))
    return dict(beams=beams,width=width,height=height,cube_size=cube_size,
        scan_spacing_m_at_1m=float(spacing),source_versions=dict(gz_sensors=gz_sensors_version,gz_rendering=gz_rendering_version),
        source_exactness='source_referenced_hypothesis_from_supplied_sensor_axes',
        variance_inferred=False,map_qualified=False)
