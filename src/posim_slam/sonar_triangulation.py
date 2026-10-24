"""Conditional point hypotheses from range/azimuth tracks and estimated poses.

Poses are held fixed, so neither cost nor local rank qualifies joint uncertainty.
Multiple elevation seeds are retained; no fan-plane or seabed-height prior is
introduced. Finite seed sampling cannot prove uniqueness of a physical return.
"""
import math
import numpy as np
from .sonar_joint import linearize_track
from .constrained_step import feasible_step


def geometric_margins(point,observations,states,geometry):
    values=[];gradients=[];mount=np.asarray(geometry['sonar_to_imu'])
    for observation in observations:
        pose=states[observation['stamp_ns']];transform=mount.T@np.asarray(pose['attitude']).T
        local=transform@(point-pose['position'])-mount.T@geometry['imu_to_sonar']
        x,y,z=local;rho=math.hypot(x,y);distance=float(np.linalg.norm(local))
        az=math.atan2(y,x);el=math.atan2(z,rho)
        az_j=np.array([-y,x,0.])/rho**2
        el_j=np.array([-z*x/rho,-z*y/rho,rho])/distance**2
        radial=local/distance
        values.extend([x,geometry['horizontal_fov']/2-az,geometry['horizontal_fov']/2+az,
            geometry['vertical_fov']/2-el,geometry['vertical_fov']/2+el,
            distance-geometry['range_min'],geometry['range_max']-distance])
        gradients.extend(np.array([[1.,0.,0.],-az_j,az_j,-el_j,el_j,radial,-radial])@transform)
    return np.asarray(values),np.asarray(gradients)


def point_hypotheses(*,observations,states,measurement_covariance,geometry,
                     elevation_seeds=9,max_iterations=12):
    if not 2<=len(observations)<=16:raise ValueError('multiple_sonar_observations_required')
    if type(elevation_seeds) is not int or not 3<=elevation_seeds<=33:raise ValueError('bounded_elevation_seeds_required')
    if type(max_iterations) is not int or not 1<=max_iterations<=30:raise ValueError('bounded_iterations_required')
    first=observations[0];state=states[first['stamp_ns']]
    r=np.asarray(state['attitude']);mount=np.asarray(geometry['sonar_to_imu'])
    origin=np.asarray(state['position'])+r@np.asarray(geometry['imu_to_sonar'])
    hypotheses=[];invalid=0
    def evaluate(point):
        block=linearize_track(observations=observations,states=states,landmark=point,
            measurement_covariance=measurement_covariance,**geometry)
        return block['residual'],block['jacobian'][:,-3:]
    for elevation in np.linspace(-.499*geometry['vertical_fov'],.499*geometry['vertical_fov'],elevation_seeds):
        azimuth=first['azimuth_rad'];distance=first['range_m']
        ray=np.array([math.cos(elevation)*math.cos(azimuth),math.cos(elevation)*math.sin(azimuth),math.sin(elevation)])
        point=origin+r@mount@(distance*ray)
        try:residual,j=evaluate(point)
        except ValueError as error:
            if str(error) not in ('unsupported_sonar_point','sonar_point_outside_fov'):raise
            invalid+=1;continue
        initial_cost=float(residual@residual);reason='iteration_limit';invalid_trials=0;valid_trials=0
        for _ in range(max_iterations):
            margins,gradients=geometric_margins(point,observations,states,geometry)
            step_result=feasible_step(j,residual,np.maximum(margins,0.)*.99,gradients)
            delta=step_result['step']
            if np.linalg.norm(delta)<1e-7 and step_result['converged']:
                reason='constrained_stationary' if step_result['active_constraints'] else 'stationary';break
            accepted=False;invalid_in_step=0
            for step in (1.,.5,.25,.125,.0625,.03125):
                candidate=point+step*delta
                try:new_residual,new_j=evaluate(candidate)
                except ValueError as error:
                    if str(error) not in ('unsupported_sonar_point','sonar_point_outside_fov'):raise
                    invalid_trials+=1
                    invalid_in_step+=1
                    continue
                valid_trials+=1
                if new_residual@new_residual<residual@residual:
                    point,residual,j=candidate,new_residual,new_j;accepted=True;break
            if not accepted:
                reason='no_feasible_trial' if invalid_in_step==6 else 'line_search_failed'
                break
        singular=np.linalg.svd(j,compute_uv=False)
        rank=int(np.sum(singular>singular[0]*1e-8)) if singular[0]>0 else 0
        elevations=[];azimuths=[];distances=[]
        for observation in observations:
            pose=states[observation['stamp_ns']]
            local=mount.T@(np.asarray(pose['attitude']).T@(point-pose['position'])-geometry['imu_to_sonar'])
            elevations.append(math.atan2(local[2],math.hypot(local[0],local[1])))
            azimuths.append(math.atan2(local[1],local[0]));distances.append(float(np.linalg.norm(local)))
        hypotheses.append(dict(seed_elevation_rad=float(elevation),point=point.tolist(),
            initial_cost=initial_cost,cost=float(residual@residual),local_rank=rank,reason=reason,
            gradient_inf=float(np.linalg.norm(j.T@residual,np.inf)),
            gn_step_m=float(np.linalg.norm(np.linalg.lstsq(j,-residual,rcond=1e-10)[0])),
            minimum_elevation_margin_rad=float(geometry['vertical_fov']/2-max(map(abs,elevations))),
            minimum_azimuth_margin_rad=float(geometry['horizontal_fov']/2-max(map(abs,azimuths))),
            minimum_range_margin_m=float(min(min(distances)-geometry['range_min'],geometry['range_max']-max(distances))),
            invalid_trials=invalid_trials,valid_trials=valid_trials))
    return dict(hypotheses=sorted(hypotheses,key=lambda h:h['cost']),invalid_seeds=invalid,
        scope='Conditional fixed-pose point seeds; association and pose uncertainty unresolved',
        elevation_branch_qualified=False,navigation_ready=False)
