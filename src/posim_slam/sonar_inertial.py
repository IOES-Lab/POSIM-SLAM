"""Bounded reference batch optimizer for associated sonar points and raw IMU.

Explicit initial state uncertainty, gravity, mounts, point seeds and sonar noise
are required. This is not a streaming frontend or a navigation-ready estimator.
Distinct tracks are assumed to have independent noise; each track may supply
full temporal covariance. Multimodal point association remains a caller duty.
"""
import copy
import math
import numpy as np
from .inertial import exp_so3
from .inertial_alignment import log_so3
from .inertial_filter import covariance
from .inertial_factor import pair_residual
from .sonar_joint import linearize_track
from .ping_joint import linearize_ping_patch
from .dvl_scale import rotation,vector


def optimize_window(*,states,tracks,imu_window,noise_density,gravity,
                    initial_state,initial_covariance,geometry,max_iterations=12,
                    ping_patches=()):
    """Optimize 2..4 poses and at most 16 point hypotheses using NumPy.

    State order is position, velocity, right attitude, gyro and accel bias (15).
    Initial-state covariance is a real supplied prior, not a gauge epsilon.
    Numeric Jacobians and bounded backtracking make this a reference solver.
    IMU integration is refreshed at accepted nonlinear states; numerical steps
    use its first-order bias correction. No damping is counted as information.
    Point seeds can select an ambiguous elevation branch; rank is local only.
    Optional Ping patches reference shared sonar track IDs and estimate one
    plane each. Physical planarity is counted once per point, not per return.
    Ping returns require independent calibrated range noise and exact state
    timestamps. Patch membership and reflection association are caller inputs.
    """
    if not 2<=len(states)<=4 or not 1<=len(tracks)<=16:
        raise ValueError('bounded_sonar_inertial_window_required')
    if type(max_iterations) is not int or not 1<=max_iterations<=30:
        raise ValueError('invalid_solver_iteration_limit')
    if sum(len(t['observations']) for t in tracks)>64:raise ValueError('too_many_sonar_observations')
    def checked(s):
        if not math.isfinite(s['stamp']) or s['stamp']<0:raise ValueError('invalid_state_time')
        return dict(stamp=float(s['stamp']),attitude=rotation(s['attitude']).copy(),
            **{key:vector(s[key]).copy() for key in ('position','velocity','gyro_bias','accel_bias')})
    current=[checked(s) for s in states];prior=checked(initial_state);g=vector(gravity)
    stamps=[int(round(s['stamp']*1e9)) for s in current]
    if (any(b<=a for a,b in zip(stamps,stamps[1:])) or prior['stamp']!=current[0]['stamp'] or
            any(not 0<b['stamp']-a['stamp']<=3 for a,b in zip(current,current[1:]))):
        raise ValueError('invalid_sonar_inertial_times')
    ids=[t['track_id'] for t in tracks]
    if any(type(i) is not int or i<0 for i in ids) or len(set(ids))!=len(ids):raise ValueError('distinct_track_ids_required')
    points=[vector(t['landmark']).copy() for t in tracks]
    if len(ping_patches)>4:raise ValueError('too_many_ping_patches')
    planes=[];support_indices=[];used_ids=set();used_times=set()
    for patch in ping_patches:
        support=patch['track_ids'];observations=patch['observations']
        if (not 3<=len(support)<=16 or any(type(i) is not int for i in support) or len(set(support))!=len(support) or
                any(i not in ids or i in used_ids for i in support)):
            raise ValueError('unique_shared_ping_support_required')
        used_ids.update(support);support_indices.append([ids.index(i) for i in support])
        times=[o['stamp_ns'] for o in observations]
        if (not 1<=len(times)<=4 or any(type(t) is not int or t not in stamps or t in used_times for t in times)
                or any(b<=a for a,b in zip(times,times[1:]))):
            raise ValueError('unique_synchronized_ping_returns_required')
        used_times.update(times)
        normal=vector(patch['plane_normal']).copy();distance=float(patch['plane_distance'])
        if abs(normal@normal-1)>1e-8 or not math.isfinite(distance):raise ValueError('invalid_ping_plane_seed')
        planes.append(dict(normal=normal,distance=distance))
    prior_l=np.linalg.cholesky(covariance(initial_covariance,15));count=len(current)
    plane_offset=15*count+3*len(points)
    dimension=plane_offset+3*len(planes)
    def integrate(s):
        return [imu_window.preintegrate(a['stamp'],b['stamp'],gyro_bias=a['gyro_bias'],
            accel_bias=a['accel_bias'],noise_density=noise_density) for a,b in zip(s,s[1:])]
    def residual(s,p,pl,pre):
        first=s[0]
        initial=np.r_[first['position']-prior['position'],first['velocity']-prior['velocity'],
            log_so3(prior['attitude'].T@first['attitude']),first['gyro_bias']-prior['gyro_bias'],
            first['accel_bias']-prior['accel_bias']]
        rows=[np.linalg.solve(prior_l,initial)]
        rows.extend(pair_residual(v,a,b,gravity=g) for v,a,b in zip(pre,s,s[1:]))
        state_map=dict(zip(stamps,s))
        for track,point in zip(tracks,p):
            rows.append(linearize_track(observations=track['observations'],states=state_map,
                landmark=point,measurement_covariance=track['measurement_covariance'],**geometry)['residual'])
        for patch,plane,indices in zip(ping_patches,pl,support_indices):
            for number,observation in enumerate(patch['observations']):
                state=state_map[observation['stamp_ns']]
                block=linearize_ping_patch(landmarks=np.array([p[i] for i in indices]),
                    position=state['position'],attitude=state['attitude'],
                    plane_normal=plane['normal'],plane_distance=plane['distance'],
                    measured_range=observation['range_m'],range_sigma=observation['range_sigma'],
                    surface_sigma=patch['surface_sigma'],max_support_distance=patch['max_support_distance'],
                    **patch['geometry'])
                rows.append(block['residual'] if number==0 else block['residual'][:1])
        return np.concatenate(rows)
    def apply(s,p,pl,delta):
        result=copy.deepcopy(s)
        for i,state in enumerate(result):
            d=delta[15*i:15*i+15]
            for key,offset in (('position',0),('velocity',3),('gyro_bias',9),('accel_bias',12)):state[key]+=d[offset:offset+3]
            state['attitude']=state['attitude']@exp_so3(d[6:9])
        new_planes=[]
        for i,plane in enumerate(pl):
            n=plane['normal'];u=np.cross(n,np.eye(3)[np.argmin(abs(n))]);u/=np.linalg.norm(u)
            basis=np.column_stack([u,np.cross(n,u)]);d=delta[plane_offset+3*i:plane_offset+3*i+3]
            new_planes.append(dict(normal=exp_so3(basis@d[:2])@n,distance=plane['distance']+d[2]))
        return result,[point+delta[15*count+3*i:15*count+3*i+3] for i,point in enumerate(p)],new_planes
    def linearize(s,p,pl,pre):
        r=residual(s,p,pl,pre);j=np.empty((len(r),dimension));epsilon=1e-6
        for k in range(dimension):
            d=np.zeros(dimension);d[k]=epsilon
            plus=apply(s,p,pl,d);minus=apply(s,p,pl,-d)
            j[:,k]=(residual(*plus,pre)-residual(*minus,pre))/(2*epsilon)
        return r,j
    initial_cost=None;costs=[];converged=False;reason='iteration_limit'
    for iteration in range(max_iterations):
        pre=integrate(current);r,j=linearize(current,points,planes,pre);cost=float(r@r)
        if initial_cost is None:initial_cost=cost
        costs.append(cost)
        delta=np.linalg.lstsq(j,-r,rcond=1e-10)[0]
        if np.linalg.norm(j.T@r,np.inf)<1e-6 or np.linalg.norm(delta)<1e-7:
            converged=True;reason='local_stationary_point';break
        accepted=False
        for scale in (1.,.5,.25,.125,.0625,.03125):
            candidate,new_points,new_planes=apply(current,points,planes,delta*scale)
            try:new_r=residual(candidate,new_points,new_planes,integrate(candidate))
            except ValueError:continue
            if new_r@new_r<cost:
                current,points,planes=candidate,new_points,new_planes;accepted=True;break
        if not accepted:reason='line_search_failed';break
    r,j=linearize(current,points,planes,integrate(current));singular=np.linalg.svd(j,compute_uv=False)
    rank=int(np.sum(singular>singular[0]*1e-8)) if len(singular) and singular[0]>0 else 0
    return dict(states=current,landmarks=points,planes=planes,track_ids=ids,initial_cost=initial_cost,
        final_cost=float(r@r),costs=costs,iterations=iteration+1,converged=converged,reason=reason,
        local_rank=rank,variables=dimension,residual_dimension=len(r),nullity=dimension-rank,navigation_ready=False,
        covariance_qualified=False,elevation_branch_qualified=False)
