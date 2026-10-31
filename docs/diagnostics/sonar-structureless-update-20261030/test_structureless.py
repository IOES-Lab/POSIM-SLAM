import unittest
import numpy as np
from structureless import owner_poses,point_rows,condition_finite_prior
from posim_slam.inertial import exp_so3,left_jacobian_so3,skew

G=dict(range_min=.4,range_max=15.,horizontal_fov=2*np.pi/3,vertical_fov=np.pi/9)
def scene():
    references=np.array([np.r_[[-.4,.3,-1.8],exp_so3([.02,-.01,.04]).ravel()],
                         np.r_[[-.1,.34,-1.70],exp_so3([.03,-.02,.10]).ravel()]])
    delta=np.array([[.03,-.02,.01,.015,-.01,.02],[-.01,.01,-.02,-.01,.02,-.01]])
    position,rotation=owner_poses(references,delta);mount=exp_so3([.015,.35,-.01]);lever=np.array([.24,-.02,-.1])
    local=np.array([[x,y,z] for x,y,z in zip(np.linspace(3,7,8),[-.7,.5,.8,-.3,.7,-.8,.4,-.5],[.06,-.1,.08,.03,-.05,.08,-.07,.05])])
    points=position[0]+rotation[0]@lever+local@(rotation[0]@mount).T
    measured=predictions(references,delta,points,mount,lever)
    measured+=np.array([.003,.0002])
    sigma=np.tile([.025083611719310283,.008213314126139592],2*len(points))
    rng=np.random.default_rng(8);a=rng.normal(size=(len(sigma),len(sigma)))
    covariance=sigma[:,None]*(np.eye(len(sigma))+.2*a@a.T/len(sigma))*sigma[None,:]
    return dict(references=references,delta=delta,points=points,measurements=measured,
                sonar_to_imu=mount,imu_to_sonar=lever,geometry=G,measurement_covariance=covariance)

def predictions(references,delta,points,mount,lever):
    # Independent direct prediction for finite-difference checks.
    output=[]
    for v,d in zip(references,delta):
        r=exp_so3(d[3:])@v[3:12].reshape(3,3);p=v[:3]+d[:3]
        q=(points-p-r@lever)@(r@mount)
        output.append(np.column_stack([np.linalg.norm(q,axis=1),np.arctan2(q[:,1],q[:,0])]))
    return np.stack(output,axis=1)

class StructurelessTests(unittest.TestCase):
    def test_fixed_owner_jacobians_match_differences_with_mount_lever_and_nonzero_delta(self):
        args=scene();rows=point_rows(**args);numeric=[]
        for axis in range(12):
            direction=np.eye(12)[axis].reshape(2,6)*1e-6
            plus=predictions(args['references'],args['delta']+direction,args['points'],args['sonar_to_imu'],args['imu_to_sonar'])
            minus=predictions(args['references'],args['delta']-direction,args['points'],args['sonar_to_imu'],args['imu_to_sonar'])
            numeric.append(((plus-minus)/2e-6).reshape(-1))
        np.testing.assert_allclose(rows['raw_pose_jacobian'],np.stack(numeric,axis=1),atol=2e-8,rtol=1e-6)

    def test_all_six_global_gauge_directions_remain_unobserved(self):
        args=scene();rows=point_rows(**args);gauge=np.zeros((12,6))
        for i,(p,d) in enumerate(zip(rows['position'],args['delta'])):
            gauge[6*i:6*i+3,:3]=np.eye(3)
            gauge[6*i:6*i+3,3:]=-skew(p)
            gauge[6*i+3:6*i+6,3:]=np.linalg.inv(left_jacobian_so3(d[3:]))
        np.testing.assert_allclose(rows['jacobian']@gauge,0,atol=1e-10)

    def test_correlated_whitening_and_elimination_match_full_joint_solution_and_covariance(self):
        rows=point_rows(**scene());rng=np.random.default_rng(4);a=rng.normal(size=(12,12))
        prior=.0001*(a@a.T+np.eye(12))
        result=condition_finite_prior(covariance=prior,jacobian=rows['jacobian'],residual=rows['residual'],unbounded_basis=np.empty((12,0)))
        j=rows['joint_whitened_jacobian'];r=rows['joint_whitened_residual'];n=j.shape[1]
        root=np.zeros((12,n));root[:,:12]=np.linalg.solve(np.linalg.cholesky(prior),np.eye(12))
        full=np.vstack([root,j]);error=np.r_[np.zeros(12),r]
        expected=np.linalg.lstsq(full,-error,rcond=None)[0]
        _,singular,vt=np.linalg.svd(full,full_matrices=False)
        covariance=(vt.T/singular**2)@vt
        np.testing.assert_allclose(result['step'],expected[:12],atol=2e-11,rtol=1e-8)
        np.testing.assert_allclose(result['covariance'],covariance[:12,:12],atol=2e-11,rtol=1e-8)
        point_step=rows['backsub_offset']+rows['backsub_matrix']@result['step']
        point_covariance=rows['backsub_matrix']@result['covariance']@rows['backsub_matrix'].T+rows['observable_noise_factor']@rows['observable_noise_factor'].T
        np.testing.assert_allclose(point_step,expected[12:],atol=2e-10,rtol=1e-8)
        np.testing.assert_allclose(point_covariance,covariance[12:,12:],atol=2e-9,rtol=1e-8)
        self.assertEqual(result['navigation_prior_count'],1);self.assertEqual(result['point_prior_count'],0)

    def test_pose_correlation_is_preserved_with_scalar_closed_form_reference(self):
        prior=np.eye(12)*.01;prior[:6,6:]=np.eye(6)*.008;prior[6:,:6]=np.eye(6)*.008
        j=np.zeros((1,12));j[0,0]=-100.;j[0,6]=100.
        out=condition_finite_prior(covariance=prior,jacobian=j,residual=[1.],unbounded_basis=np.empty((12,0)))
        direction=j[0]/100;variance=.004;noise=.0001
        self.assertAlmostEqual(float(direction@out['covariance']@direction),variance*noise/(variance+noise),places=14)
        self.assertAlmostEqual(float(direction@out['step']),-.01*variance/(variance+noise),places=14)
        independent=condition_finite_prior(covariance=np.eye(12)*.01,jacobian=j,residual=[1.],unbounded_basis=np.empty((12,0)))
        self.assertGreater(np.linalg.norm(independent['step']-out['step']),1e-5)

    def test_pure_yaw_retains_height_nullspace_and_does_not_update_unmeasured_axes(self):
        reference=np.array([np.r_[np.zeros(3),np.eye(3).ravel()],np.r_[np.zeros(3),exp_so3([0,0,.08]).ravel()]])
        points=np.array([[x,y,0] for x in [3.,4.,5.,6.] for y in [-.7,.8]])
        measured=predictions(reference,np.zeros((2,6)),points,np.eye(3),np.zeros(3))
        rows=point_rows(references=reference,delta=np.zeros((2,6)),points=points,measurements=measured,
            sonar_to_imu=np.eye(3),imu_to_sonar=np.zeros(3),geometry=G,measurement_covariance=np.eye(4*len(points)))
        self.assertEqual(rows['nuisance_nullity'],len(points))
        axes=[2,3,4,8,9,10];np.testing.assert_allclose(rows['jacobian'][:,axes],0,atol=1e-12)
        out=condition_finite_prior(covariance=np.eye(12),jacobian=rows['jacobian'],residual=rows['residual'],unbounded_basis=np.empty((12,0)))
        np.testing.assert_array_equal(out['step'],np.zeros(12))
        np.testing.assert_allclose(out['covariance'][np.ix_(axes,axes)],np.eye(len(axes)),atol=1e-12)

    def test_empty_observation_leaves_prior_exactly_unchanged(self):
        prior=np.eye(12)*.03
        out=condition_finite_prior(covariance=prior,jacobian=np.empty((0,12)),residual=[],unbounded_basis=np.empty((12,0)))
        np.testing.assert_array_equal(out['step'],np.zeros(12));np.testing.assert_array_equal(out['covariance'],prior)

    def test_invalid_or_unbounded_uncertainty_is_rejected_without_jitter(self):
        args=scene();args['measurement_covariance'][:]=0
        with self.assertRaisesRegex(ValueError,'positive_definite_measurement_covariance_required'):point_rows(**args)
        with self.assertRaisesRegex(ValueError,'unbounded_prior_requires_information_form'):
            condition_finite_prior(covariance=np.eye(12),jacobian=np.zeros((1,12)),residual=[0],unbounded_basis=np.eye(12)[:,:1])
        bad=np.eye(12);bad[0,0]=-1
        with self.assertRaisesRegex(ValueError,'positive_semidefinite_navigation_covariance_required'):
            condition_finite_prior(covariance=bad,jacobian=np.zeros((1,12)),residual=[0],unbounded_basis=np.empty((12,0)))

if __name__=='__main__':unittest.main()
