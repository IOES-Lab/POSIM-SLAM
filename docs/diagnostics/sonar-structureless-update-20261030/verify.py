"""Reproduce conditional rows/updates and all assessed modes with public code."""
from pathlib import Path
import hashlib,json
import numpy as np
from structureless import owner_poses,point_rows,condition_finite_prior
ROOT=Path(__file__).resolve().parent
def load(name):return json.loads((ROOT/name).read_text())
for name,digest in load('sha256.json').items():
    assert hashlib.sha256((ROOT/name).read_bytes()).hexdigest()==digest,name
with np.load(ROOT/'pair-joint.npz') as archive:prior={k:archive[k].copy() for k in archive.files}
meta=load('prepared-input.json');cal=np.array(meta['sonar_to_imu']);lever=np.array(meta['imu_to_sonar'])
assessment=load('field-assessment.json');checks=0
def angle(r):return np.degrees(np.arccos(np.clip((np.trace(r)-1)/2,-1,1)))
def surface(points):
    align=meta['fixed_alignment'];world=points@np.array(align['rotation']).T+np.array(align['translation']);values=[]
    for box in meta['assessment_only_boxes']:
        delta=np.abs(world-np.array(box['center']))-np.array(box['half_size'])
        values.append(np.where(delta.max(axis=1)>0,np.linalg.norm(np.maximum(delta,0),axis=1),np.min(-delta,axis=1)))
    return np.min(values,axis=0)
truth=meta['assessment_only_sensor_truth'];true_r=np.array(truth[0]['rotation']).T@np.array(truth[1]['rotation'])
true_t=np.array(truth[0]['rotation']).T@(np.array(truth[1]['position'])-np.array(truth[0]['position']))
for group in load('field-update-summary.json')['results']:
    variant=group['variant'];support=load(variant+'-support.json')
    assert support['duplicate_original_keys']==0 and len(support['original_pixel_keys'])==2*group['selected']
    with np.load(ROOT/(variant+'-all-point-fits.npz')) as data:
        assert data['points_in_first'].shape==(group['observations'],9,3)
        assert data['valid'].shape==data['converged'].shape==data['cost'].shape==(group['observations'],9)
        assert int(data['selected'].sum())==group['selected']
    for mode in group['modes']:
        assert mode['status']=='conditioned'
        with np.load(ROOT/(variant+'-'+mode['mode']+'.npz')) as data:d={k:data[k].copy() for k in data.files}
        ids=d['selected_indices'];n=len(ids)
        rows=point_rows(references=prior['references'],delta=prior['delta'],points=d['initial_points'],
            measurements=np.stack([d['original_first'][ids],d['original_second'][ids]],axis=1),
            sonar_to_imu=cal,imu_to_sonar=lever,geometry=meta['geometry'],
            measurement_covariance=np.diag(np.tile(load('predeclared-control.json')['noise_sigma'],2*n)**2))
        update=condition_finite_prior(covariance=prior['covariance'],jacobian=rows['jacobian'],residual=rows['residual'],unbounded_basis=prior['unbounded_basis'])
        for name,computed in [('posterior_delta',prior['delta']+update['step'].reshape(2,6)),('posterior_covariance',update['covariance'])]:
            np.testing.assert_allclose(d[name],computed,rtol=1e-9,atol=2e-11);checks+=1
        latent=d['initial_points']+(rows['backsub_offset']+rows['backsub_matrix']@update['step']).reshape(-1,3)
        covariance=rows['backsub_matrix']@update['covariance']@rows['backsub_matrix'].T+rows['observable_noise_factor']@rows['observable_noise_factor'].T
        np.testing.assert_allclose(d['latent_points'],latent,atol=2e-8,rtol=1e-9)
        np.testing.assert_allclose(d['latent_covariance'],covariance,atol=2e-8,rtol=1e-9);checks+=2
        report=next(r for r in assessment['results'] if r['variant']==variant and r['mode']==mode['mode'])
        p,r=owner_poses(prior['references'],d['posterior_delta']);p+=np.einsum('nij,j->ni',r,lever);r=r@cal
        np.testing.assert_allclose(np.linalg.norm(r[0].T@(p[1]-p[0])-true_t),report['after']['relative_translation_error_m'],atol=1e-12)
        np.testing.assert_allclose(angle(true_r.T@r[0].T@r[1]),report['after']['relative_rotation_error_deg'],atol=1e-9);checks+=2
        for key,points in [('initial_surface',d['initial_points']),('latent_surface',latent),('reprofiled_surface',d['nonlinear_world_points'][ids])]:
            error=surface(points);np.testing.assert_allclose(error,report[key]['error_m'],rtol=1e-9,atol=2e-8);checks+=1
        sigma=[3*np.sqrt(np.linalg.eigvalsh(covariance[3*i:3*i+3,3*i:3*i+3]).max()) for i in range(n)]
        np.testing.assert_allclose(sigma,report['latent_position_three_sigma_major_m'],atol=2e-8,rtol=1e-9);checks+=1
        assert report['bounded_below_10cm']==0 and not report['qualified'] and update['navigation_prior_count']==1
ablation=load('dvl-controls.json');clocks=[]
for mode in ablation['results']:
    trace=np.genfromtxt(ROOT/('dvl-'+mode['mode']+'-aligned.csv'),delimiter=',',names=True);clocks.append(trace['stamp_ns'])
    estimated=np.column_stack([trace[k] for k in ['x','y','z']]);truth=np.column_stack([trace[k] for k in ['truth_x','truth_y','truth_z']])
    error=np.linalg.norm(estimated-truth,axis=1)
    np.testing.assert_allclose(error,trace['error_m'],atol=1e-12)
    np.testing.assert_allclose(np.sqrt(np.mean(error**2)),mode['trajectory']['position_rmse_m'],atol=1e-12);checks+=2
assert np.array_equal(*clocks) and len(clocks[0])==175
assert ablation['results'][0]['dvl_packets_supplied']==55 and ablation['results'][1]['dvl_packets_supplied']==0
assert not ablation['physical_dvl_failure_tested'] and not ablation['acoustic_factors_used']
print(f'PASS: {checks} conditional-update, full-point and identical-clock assessment comparisons; map remains unqualified')
