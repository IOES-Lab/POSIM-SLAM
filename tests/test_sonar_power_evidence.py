import unittest
import numpy as np
from posim_slam.sonar_power_evidence import profile_power_evidence


class PowerEvidenceTests(unittest.TestCase):
    def test_gain_and_profile_gradient_match_independent_finite_difference(self):
        p = np.array([2., 8., 3., 7.])
        m = np.array([1., 3., 2., 4.])
        j = np.array([[-1., .3], [0., .4], [1., -.2], [2., -.5]])
        c = np.diag(m*m)
        a = profile_power_evidence(p, m, c, log_mean_jacobian=j)
        self.assertAlmostEqual(a['gain'], np.mean(p/m))
        for k in range(2):
            h = 1e-6
            scores = [profile_power_evidence(p, m*np.exp(sign*h*j[:, k]),
                       c*np.exp(sign*h*(j[:, k, None]+j[None, :, k])))['composite_log_score']
                      for sign in (1, -1)]
            self.assertAlmostEqual((scores[0]-scores[1])/(2*h), -a['score'][k], places=7)

    def test_duplicated_correlated_pixels_do_not_create_information(self):
        p = np.array([2., 4., 8.]); m = np.array([1., 2., 4.]); j = np.array([[-1.], [0.], [1.]])
        c = np.diag(m*m)
        original = profile_power_evidence(p, m, c, log_mean_jacobian=j)
        index = np.tile(np.arange(3), 8)
        duplicate = profile_power_evidence(p[index], m[index], c[np.ix_(index, index)],
                                             log_mean_jacobian=j[index])
        np.testing.assert_allclose(duplicate['conditional_identifiable_covariance'],
                                   original['conditional_identifiable_covariance'])
        independent = profile_power_evidence(p[index], m[index], np.diag(m[index]**2),
                                               log_mean_jacobian=j[index])
        np.testing.assert_allclose(independent['conditional_identifiable_covariance']*8,
                                   original['conditional_identifiable_covariance'])

    def test_common_gain_direction_stays_unobservable(self):
        j = np.column_stack((np.ones(4), [-1., 0., 1., 2.]))
        a = profile_power_evidence([2., 4., 6., 8.], [1., 2., 3., 4.],
                                   np.diag([1., 4., 9., 16.]), log_mean_jacobian=j)
        self.assertEqual(a['shape_rank'], 1)
        np.testing.assert_allclose(j@a['nullspace_basis'], np.tile(j[0]@a['nullspace_basis'], (4, 1)), atol=1e-14)
        self.assertEqual(a['conditional_identifiable_covariance'].shape, (1, 1))
        constant = profile_power_evidence([1.]*7, [1.]*7, np.eye(7),
                                            log_mean_jacobian=np.full((7, 2), .3))
        self.assertEqual(constant['shape_rank'], 0)

    def test_conditional_covariance_matches_independent_nonlinear_ratio_estimator(self):
        rng = np.random.default_rng(1837)
        frames = 128
        xi = (rng.normal(size=(5000, frames, 2))+1j*rng.normal(size=(5000, frames, 2)))/np.sqrt(2)
        z = xi.copy(); z[..., 1] = .8*xi[..., 0]+.6*xi[..., 1]
        averages = (abs(z)**2).mean(axis=1)
        estimate = .5*np.log(averages[:, 1]/averages[:, 0])
        result = profile_power_evidence([1., 1.], [1., 1.],
                    np.array([[1., .64], [.64, 1.]])/frames, log_mean_jacobian=[[-1.], [1.]])
        expected = .18/frames
        self.assertAlmostEqual(result['conditional_identifiable_covariance'][0, 0], expected)
        self.assertLess(abs(np.var(estimate, ddof=1)/expected-1), .08)

    def test_dynamic_power_range_and_zero_observation_remain_supported(self):
        m = np.array([1e-30, 1e30]); p = np.array([0., 2e30])
        a = profile_power_evidence(p, m, np.diag(m*m), log_mean_jacobian=[[-1.], [1.]])
        self.assertEqual(a['gain'], 1.)
        self.assertEqual(a['shape_rank'], 1)
        np.testing.assert_allclose(a['normalized_residual'], [-1., 1.])

    def test_zero_model_support_cannot_explain_a_return(self):
        with self.assertRaisesRegex(ValueError, 'outside_model_support'):
            profile_power_evidence([1., 1.], [0., 1.], [[0., 0.], [0., 1.]])
        a = profile_power_evidence([0., 2.], [0., 1.], [[0., 0.], [0., 1.]])
        self.assertEqual(a['zero_support_pixels'], 1)
        with self.assertRaisesRegex(ValueError, 'no_model_power_support'):
            profile_power_evidence([0.], [0.], [[0.]])

    def test_invalid_moments_do_not_receive_jitter(self):
        for c in ([[1., 2.], [2., 1.]], [[1., 0.], [1., 1.]], [[0., 0.], [0., 1.]]):
            with self.assertRaises(ValueError):profile_power_evidence([1., 1.], [1., 1.], c)
        with self.assertRaises(ValueError):profile_power_evidence([-1.], [1.], [[1.]])
        with self.assertRaises(ValueError):profile_power_evidence([0.], [1.], [[1.]])


if __name__ == '__main__':
    unittest.main()
