"""Reproduce bounded perfect-pose controls with independent POSIM-SLAM inputs."""
from pathlib import Path
import json
import numpy as np
from posim_slam.sonar_triangulation import point_hypotheses, heldout_residuals
from posim_slam.sonar_joint import linearize_track

base=Path(__file__).resolve().parent
data=json.loads((base/'prepared-input.json').read_text())
expected=json.loads((base/'fit-summary.json').read_text())['results']
states={int(k):{name:np.asarray(value) for name,value in v.items()} for k,v in data['states'].items()}
geometry={k:np.asarray(v) if k in ['imu_to_sonar','sonar_to_imu'] else v for k,v in data['geometry'].items()}
sigma=np.asarray(data['settings']['noise_sigma']);comparisons=0
def distance(point):
    errors=[]
    for box in data['boxes']:
        delta=np.abs(point-np.asarray(box['center']))-box['half_size']
        errors.append(np.linalg.norm(np.maximum(delta,0)) if delta.max()>0 else np.min(-delta))
    return float(min(errors))
for reference in expected:
    variant=reference['variant'];window=reference['window']
    eligible=fitted=stationary=below=heldout_total=compatible=mode_count=0
    errors=[];major=[]
    for track in data['variants'][variant]:
        original=[dict(stamp_ns=o['stamp_ns'],range_m=o['range_azimuth'][0],azimuth_rad=o['range_azimuth'][1]) for o in track['observations']]
        count=len(original)
        if window=='two_training_views':indices=np.array([0,1])
        elif window=='eight_training_views':indices=np.arange(8)
        else:indices=np.unique(np.rint(np.linspace(0,count-2,min(16,count-1))).astype(int))
        if count<3 or indices[-1]>=count-1:continue
        eligible+=1;training=[original[i] for i in indices]
        heldout=[o for i,o in enumerate(original) if i not in set(indices)]
        covariance=np.diag(np.tile(sigma,len(training))**2)
        result=point_hypotheses(observations=training,states=states,
            measurement_covariance=covariance,geometry=geometry,elevation_seeds=9,max_iterations=30)
        modes=result['hypotheses'];mode_count+=len(modes)
        if not modes:continue
        fitted+=1;fit=modes[0];point=np.asarray(fit['point'])
        stationary+=fit['reason'] in ['stationary','constrained_stationary']
        block=linearize_track(observations=training,states=states,landmark=point,measurement_covariance=covariance,**geometry)
        singular=np.linalg.svd(block['jacobian'][:,-3:],compute_uv=False)
        if np.sum(singular>singular[0]*1e-8)==3:
            value=float(3/singular[-1]);major.append(value);below+=value<=.1
        check=heldout_residuals(point=point,observations=heldout,states=states,geometry=geometry,
                                fit_stamps_ns=[o['stamp_ns'] for o in training])
        heldout_total+=check['total']
        compatible+=sum(o['valid_geometry'] and (o['range_residual_m']/sigma[0])**2+(o['azimuth_residual_rad']/sigma[1])**2<=9. for o in check['observations'])
        errors.append(distance(point))
    observed=dict(eligible=eligible,fitted=fitted,no_feasible_seed=eligible-fitted,stationary_best=stationary,
                  optimistic_below_10cm=below,minimum_optimistic_three_sigma_m=min(major) if major else None,
                  heldout_candidates=heldout_total,heldout_compatible=compatible,all_modes_retained=mode_count,
                  surface_error_p50_p90_max_m=np.percentile(errors,[50,90,100]).tolist() if errors else None)
    for key,value in observed.items():
        other=reference[key]
        assert (value is None and other is None) or (value is not None and other is not None and np.allclose(value,other,rtol=1e-6,atol=1e-6)),(variant,window,key,value,other)
        comparisons+=1
print(f'PASS: {comparisons} prepared summary comparisons; every original chain retained; no native/SLAM update')
