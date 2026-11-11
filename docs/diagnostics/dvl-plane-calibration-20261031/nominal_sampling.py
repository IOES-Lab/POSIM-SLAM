# SPDX-License-Identifier: Apache-2.0
"""Unqualified gz-sensors10.1.1 grid interpretation; fails native controls.

The source uses an odd rectangular angular grid, half-open integer patches and
a strict cone-membership test. This independent interpretation does not qualify
the installed renderer's pixel positions, return branch or hardware beam model.
Geometry reference: OSRF gz-sensors, Apache-2.0, DopplerVelocityLog.cc 10.1.1.
"""
import math
import numpy as np


def sampled_beams(beams, resolution_m_at_1m):
    if (not isinstance(beams,(list,tuple)) or not 3<=len(beams)<=16
            or not np.isfinite(resolution_m_at_1m) or resolution_m_at_1m<=0):
        raise ValueError('bounded_explicit_dvl_sampling_profile_required')
    axes=np.asarray([b['axis'] for b in beams],dtype=float)
    aperture=np.asarray([b['aperture_rad'] for b in beams],dtype=float)
    if (axes.shape!=(len(beams),3) or not np.isfinite(axes).all()
            or not np.allclose(np.linalg.norm(axes,axis=1),1.,rtol=0,atol=1e-9)
            or not np.isfinite(aperture).all() or np.any(aperture<=0) or np.any(aperture>=math.pi)
            or len(set(b['id'] for b in beams))!=len(beams)):
        raise ValueError('unit_axes_and_unique_dvl_beam_ids_required')
    # R_y(pi/2) maps the acoustic x-down frame into the z-up sensor frame.
    acoustic_to_sensor=np.array([[0.,0.,1.],[0.,1.,0.],[-1.,0.,0.]])
    local=axes@acoustic_to_sensor
    az=np.arctan2(local[:,1],local[:,0]);el=np.arctan2(local[:,2],np.linalg.norm(local[:,:2],axis=1))
    extent=np.array([np.max(abs(az)+aperture/2),np.max(abs(el)+aperture/2)])
    if np.any(extent>=math.pi/2):raise ValueError('forward_bounded_dvl_scan_required')
    count=np.ceil(2*extent/resolution_m_at_1m).astype(int)
    count+=count%2==0
    if np.any(count<3) or np.prod(count)>262144:raise ValueError('dvl_sampling_workspace_limit')
    step=2*extent/(count-1)
    result=[]
    for i,beam in enumerate(beams):
        lower=np.array([az[i],el[i]])-aperture[i]/2
        upper=np.array([az[i],el[i]])+aperture[i]/2
        begin=np.trunc((lower+extent)/step).astype(int)
        stop=np.trunc((upper+extent)/step).astype(int)
        u,v=np.meshgrid(np.arange(begin[0],stop[0]),np.arange(begin[1],stop[1]))
        a=u.ravel()*step[0]-extent[0];e=v.ravel()*step[1]-extent[1]
        directions=np.column_stack((np.cos(e)*np.cos(a),np.cos(e)*np.sin(a),np.sin(e)))
        angle=np.arccos(np.clip(directions@local[i],-1.,1.))
        selected=angle<aperture[i]/2
        if not 1<=int(selected.sum())<=256:raise ValueError('dvl_sampled_ray_count_unavailable')
        result.append(dict(id=beam['id'],rays=directions[selected]@acoustic_to_sensor.T,
            indices=np.column_stack((u.ravel(),v.ravel()))[selected],
            membership_margin_rad=float(np.min(abs(angle-aperture[i]/2))),
            grid_count=count.copy(),grid_offset=-extent.copy(),grid_step=step.copy()))
    return result
