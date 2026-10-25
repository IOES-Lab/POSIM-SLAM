"""Conditional nonlinear heading profiles of an unknown local ground plane.

This diagnostic refits the plane at every trial relative yaw rate. Positions,
roll/pitch and range covariance stay fixed at caller-supplied estimates. It
accepts no map or truth and does not supply a pose correction or confidence
interval to the estimator. A global rigid gauge must not change its costs.
"""
import math
import numpy as np
from .dvl_scale import rotation, vector
from .inertial_filter import covariance
from .sonar_surface import ground_entry_band


def heading_profile(*, observations, states, plane_normal, plane_distance,
                    measurement_covariance, imu_to_sonar, sonar_to_imu,
                    vertical_fov, range_min, range_max, rates_rad_s):
    if not 2 <= len(observations) <= 8:
        raise ValueError('bounded_multiframe_ground_profile_required')
    stamps = [o['stamp_ns'] for o in observations]
    if any(type(t) is not int or t <= 0 for t in stamps) or any(b <= a for a,b in zip(stamps,stamps[1:])):
        raise ValueError('ordered_ground_profile_stamps_required')
    if stamps[-1]-stamps[0] > 2_000_000_000:
        raise ValueError('ground_profile_time_budget')
    rates = np.asarray(rates_rad_s, dtype=float)
    if (rates.ndim != 1 or not 1 <= len(rates) <= 181 or not np.isfinite(rates).all()
            or np.any(np.diff(rates) <= 0) or np.max(abs(rates)) > math.pi):
        raise ValueError('bounded_ordered_ground_profile_rates_required')
    n = vector(plane_normal); lever = vector(imu_to_sonar); mount = rotation(sonar_to_imu)
    if (not np.isclose(n@n,1,atol=1e-8,rtol=0) or n[2] <= math.cos(math.pi/6)
            or not np.isfinite([plane_distance,vertical_fov,range_min,range_max]).all() or not 0 < vertical_fov < math.pi
            or not 0 < range_min < range_max):
        raise ValueError('invalid_ground_profile_geometry')
    q0 = np.r_[-n[:2]/n[2], plane_distance/n[2]]
    local = []; measured = []; blocks = []; count = 0
    for o in observations:
        az = np.asarray(o['azimuth_rad'], dtype=float); r = np.asarray(o['range_m'], dtype=float)
        if (az.ndim != 1 or not 8 <= len(az) <= 128 or r.shape != az.shape
                or not np.isfinite(az).all() or not np.isfinite(r).all()
                or np.any(np.diff(az) <= 0) or np.any(abs(az) >= math.pi/2)
                or np.any(r < range_min) or np.any(r > range_max)
                or o['boundary_sign'] not in (-1,1)):
            raise ValueError('invalid_ground_profile_observation')
        state = states[o['stamp_ns']]
        rotation(state['attitude']); vector(state['position'])
        e = o['boundary_sign']*vertical_fov/2
        local.append(np.column_stack((np.cos(az)*math.cos(e),np.sin(az)*math.cos(e),np.full(len(az),math.sin(e)))))
        measured.extend(r); blocks.append(slice(count,count+len(r))); count += len(r)
    if count > 512:
        raise ValueError('ground_profile_row_budget')
    lower = np.linalg.cholesky(covariance(measurement_covariance,count))
    # One frozen whitening operator for all rates and plane iterations.
    whiten = np.linalg.solve(lower,np.eye(count)); measured = np.asarray(measured)
    rows = []
    for rate in rates:
        origins = []; rays = []; sensors = []
        for o,u in zip(observations,local):
            state = states[o['stamp_ns']]; angle = rate*(o['stamp_ns']-stamps[0])*1e-9
            c,s = math.cos(angle),math.sin(angle)
            r = np.array([[c,-s,0],[s,c,0],[0,0,1]]) @ state['attitude']
            origin = state['position'] + r@lever; sensor = r@mount
            origins.extend(np.tile(origin,(len(u),1))); rays.extend(u@sensor.T); sensors.append((origin,sensor))
        origins = np.asarray(origins); rays = np.asarray(rays)
        def evaluate(q):
            if not np.isfinite(q).all() or np.linalg.norm(q[:2]) > math.tan(math.pi/6):
                return None
            denominator = rays[:,2]-rays[:,:2]@q[:2]
            if np.any(abs(denominator)<1e-6):return None
            prediction = (q[2]+origins[:,:2]@q[:2]-origins[:,2])/denominator
            if np.any(prediction<range_min) or np.any(prediction>range_max):return None
            normal = np.r_[-q[:2],1.]; length = np.linalg.norm(normal); normal /= length
            for o,(origin,sensor),block in zip(observations,sensors,blocks):
                entry = ground_entry_band(np.asarray(o['azimuth_rad']),vertical_fov,
                                          sensor.T@normal,q[2]/length-normal@origin)[:,0]
                if not np.allclose(prediction[block],entry,atol=1e-7,rtol=0):return None
            jacobian = np.column_stack((origins[:,:2]+prediction[:,None]*rays[:,:2],np.ones(count)))/denominator[:,None]
            residual = whiten@(prediction-measured); jacobian = whiten@jacobian
            return residual,jacobian
        q = q0.copy(); value = evaluate(q); converged = False; iterations = 0
        if value is not None:
            for iterations in range(1,31):
                residual,jacobian = value
                step,_,rank,_ = np.linalg.lstsq(jacobian,-residual,rcond=1e-10)
                if rank != 3:break
                if np.linalg.norm(step)<1e-9:
                    converged = True; break
                accepted = False
                for scale in (1.,.5,.25,.125,.0625,.03125,.015625):
                    candidate = evaluate(q+scale*step)
                    if candidate is not None and candidate[0]@candidate[0] < residual@residual:
                        change = float(residual@residual-candidate[0]@candidate[0])
                        q += scale*step; value = candidate; accepted = True
                        if change < 1e-10*(1+residual@residual):converged = True
                        break
                if converged or not accepted:break
        row = dict(rate_rad_s=float(rate),valid=bool(value is not None),converged=converged,iterations=iterations)
        if value is not None:
            normal = np.r_[-q[:2],1.]; length = np.linalg.norm(normal)
            row.update(cost=float(.5*value[0]@value[0]),normal=(normal/length).tolist(),distance=float(q[2]/length))
        rows.append(row)
    return dict(scope=__doc__.strip(),frames=len(observations),ranges=count,
                duration_s=(stamps[-1]-stamps[0])*1e-9,profile=rows)
