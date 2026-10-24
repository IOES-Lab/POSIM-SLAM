import unittest
import numpy as np
from posim_slam.sonar_marginal import marginalize_window
from posim_slam.sonar_inertial import optimize_window
from test_sonar_inertial import fixture
import test_sonar_inertial as sonar_tests


class MarginalTests(unittest.TestCase):
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
