import unittest
import copy
import numpy as np
from posim_slam.sonar_marginal import marginalize_window
from posim_slam.sonar_inertial import optimize_window
from test_sonar_inertial import fixture
import test_sonar_inertial as sonar_tests


class MarginalTests(unittest.TestCase):
    def sequential_fixture(self):
        full=sonar_tests.SonarInertialTests().delayed_ping_fixture()
        first=copy.copy(full);first.update(states=copy.deepcopy(full['states'][:2]),ping_patches=[])
        result=optimize_window(**first,retain_state_stamps_ns=[1_000_000_000])
        second=copy.deepcopy(full)
        second.update(states=[copy.deepcopy(result['states'][1]),copy.deepcopy(full['states'][2])],
            initial_state=None,initial_covariance=None,linearized_prior=result['linearized_prior'],
            independent_new_measurements=True)
        for track,point in zip(second['tracks'],result['landmarks']):
            track['observations']=[];track['measurement_covariance']=np.empty((0,0));track['landmark']=point.copy()
        return full,second

    def test_next_window_uses_historical_points_for_first_delayed_ping(self):
        full,second=self.sequential_fixture()
        second['states'][1]['position']+=np.array([.015,-.01,.02])
        second['ping_patches'][0]['plane_distance']-=.025
        result=optimize_window(**second,retain_state_stamps_ns=[3_000_000_000])
        expected=optimize_window(**full)
        self.assertLess(result['final_cost'],result['initial_cost']*1e-6)
        np.testing.assert_allclose(result['states'][-1]['position'],expected['states'][-1]['position'],atol=1e-4)
        self.assertAlmostEqual(result['planes'][0]['distance'],-.8,places=4)
        self.assertEqual(result['linearized_prior']['absorbed_measurements']['imu_intervals_ns'],
                         [(0,1_000_000_000),(1_000_000_000,3_000_000_000)])

    def test_reused_measurements_model_changes_and_extra_prior_rejected(self):
        full,second=self.sequential_fixture()
        for kind in ('sonar','imu','prior','noise','independence'):
            args=copy.deepcopy(second)
            if kind=='sonar':args['tracks'][0]['observations']=[full['tracks'][0]['observations'][1]]
            if kind=='imu':args['states']=copy.deepcopy(full['states'])
            if kind=='prior':args['initial_state']=copy.deepcopy(full['initial_state'])
            if kind=='noise':args['noise_density'][0]*=2
            if kind=='independence':args['independent_new_measurements']=False
            reason=dict(sonar='sonar_observation_already_absorbed',imu='imu_interval_already_absorbed',
                prior='initial_and_marginal_prior_are_exclusive',noise='marginal_prior_model_changed',
                independence='cross_window_measurement_independence_required')[kind]
            with self.subTest(kind=kind),self.assertRaisesRegex(ValueError,reason):optimize_window(**args)

    def args(self):
        return dict(states=fixture()['states'],landmarks=[np.array([4.,0.,1.])],
                    planes=[],track_ids=[7],retain_state_stamps_ns=[1_000_000_000])

    def test_reduced_cost_and_cross_covariance_equal_full_system(self):
        rng=np.random.default_rng(304);j=rng.normal(size=(55,33));r=rng.normal(size=55)
        reduced=marginalize_window(j,r,**self.args())
        a=reduced['jacobian'];b=reduced['residual']
        for _ in range(5):
            delta=rng.normal(size=18)
            old=np.linalg.lstsq(j[:,:15],-(r+j[:,15:]@delta),rcond=None)[0]
            self.assertAlmostEqual(np.linalg.norm(r+j@np.r_[old,delta])**2,
                                   np.linalg.norm(b+a@delta)**2,places=10)
        expected=np.linalg.inv(j.T@j)[15:,15:]
        np.testing.assert_allclose(np.linalg.inv(a.T@a),expected,atol=1e-12)
        self.assertGreater(np.linalg.norm(expected[:15,15:]),.001)

    def test_unobservable_point_direction_is_not_regularized(self):
        j=np.eye(33);j[-1,-1]=0
        prior=marginalize_window(j,np.zeros(33),**self.args())
        np.testing.assert_array_equal(prior['jacobian'][:,-1],0)
        self.assertEqual(np.linalg.matrix_rank(prior['jacobian']),17)

    def test_solver_exports_owned_measurements_and_copied_anchors(self):
        result=optimize_window(**fixture(),retain_state_stamps_ns=[1_000_000_000])
        prior=result['linearized_prior']
        self.assertEqual(prior['jacobian'].shape[1],21)
        self.assertEqual(prior['absorbed_measurements']['imu_intervals_ns'],[(0,1_000_000_000)])
        self.assertEqual(prior['track_ids'],[0,1])
        result['states'][1]['position'][:]=999
        self.assertLess(np.linalg.norm(prior['states'][0]['position']),1)

    def test_unknown_duplicate_or_all_state_retention_rejected(self):
        for keep in ([],[2],[0,0],[0,1_000_000_000],[1_000_000_000,0]):
            args=self.args();args['retain_state_stamps_ns']=keep
            with self.assertRaises(ValueError):marginalize_window(np.eye(33),np.zeros(33),**args)

    def test_ping_plane_tangent_and_absorbed_support_survive(self):
        result=optimize_window(**sonar_tests.SonarInertialTests().ping_fixture(),retain_state_stamps_ns=[1_000_000_000])
        prior=result['linearized_prior'];basis=prior['plane_rotation_bases'][0]
        self.assertEqual(prior['jacobian'].shape[1],30)
        np.testing.assert_allclose(basis.T@basis,np.eye(2))
        np.testing.assert_allclose(basis.T@prior['planes'][0]['normal'],[0,0])
        self.assertEqual(prior['absorbed_measurements']['ping'][0]['track_ids'],[0,1,2,3])


if __name__=='__main__':unittest.main()
