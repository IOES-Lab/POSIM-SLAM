"""Conditional point hypotheses from range/azimuth tracks and estimated poses.

Poses are held fixed, so neither cost nor local rank qualifies joint uncertainty.
Multiple elevation seeds are retained; no fan-plane or seabed-height prior is
introduced. Finite seed sampling cannot prove uniqueness of a physical return.
"""
import math
import numpy as np
from .sonar_joint import linearize_track


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
        initial_cost=float(residual@residual);reason='iteration_limit'
        for _ in range(max_iterations):
            delta=np.linalg.lstsq(j,-residual,rcond=1e-10)[0]
            if np.linalg.norm(delta)<1e-7:reason='stationary';break
            accepted=False
            for step in (1.,.5,.25,.125,.0625,.03125):
                candidate=point+step*delta
                try:new_residual,new_j=evaluate(candidate)
                except ValueError as error:
                    if str(error) not in ('unsupported_sonar_point','sonar_point_outside_fov'):raise
                    continue
                if new_residual@new_residual<residual@residual:
                    point,residual,j=candidate,new_residual,new_j;accepted=True;break
            if not accepted:reason='line_search_failed';break
        singular=np.linalg.svd(j,compute_uv=False)
        rank=int(np.sum(singular>singular[0]*1e-8)) if singular[0]>0 else 0
        hypotheses.append(dict(seed_elevation_rad=float(elevation),point=point.tolist(),
            initial_cost=initial_cost,cost=float(residual@residual),local_rank=rank,reason=reason))
    return dict(hypotheses=sorted(hypotheses,key=lambda h:h['cost']),invalid_seeds=invalid,
        scope='Conditional fixed-pose point seeds; association and pose uncertainty unresolved',
        elevation_branch_qualified=False,navigation_ready=False)
