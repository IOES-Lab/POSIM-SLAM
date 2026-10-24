import unittest
import numpy as np
from test_sonar_inertial import fixture
from posim_slam.sonar_triangulation import point_hypotheses,geometric_margins,heldout_residuals


class TriangulationTests(unittest.TestCase):
    def test_unused_return_exposes_error_without_refitting(self):
        args=fixture();point=args['tracks'][0]['landmark'];pose=args['states'][1]
        observation=dict(args['tracks'][0]['observations'][1]);observation['range_m']+=.5
        result=heldout_residuals(point=point,observations=[observation],states={1_000_000_000:pose},
            geometry=args['geometry'],fit_stamps_ns=[0])
        self.assertAlmostEqual(result['observations'][0]['range_residual_m'],.5)
        with self.assertRaisesRegex(ValueError,'distinct_heldout_observations_required'):
            heldout_residuals(point=point,observations=[observation],states={1_000_000_000:pose},
                geometry=args['geometry'],fit_stamps_ns=[1_000_000_000])

    def test_heldout_point_outside_fov_is_counted(self):
        args=fixture();track=args['tracks'][0];point=track['landmark']+np.array([0.,0.,10.])
        result=heldout_residuals(point=point,observations=track['observations'],
            states={round(s['stamp']*1e9):s for s in args['states']},geometry=args['geometry'],fit_stamps_ns=[])
        self.assertEqual(result['total'],2);self.assertEqual(result['valid_geometry'],0)

    def test_geometric_constraint_jacobian_matches_finite_difference(self):
        args=fixture();track=args['tracks'][0];point=track['landmark']
        states={round(s['stamp']*1e9):s for s in args['states']}
        _,analytic=geometric_margins(point,track['observations'],states,args['geometry'])
        numerical=np.column_stack([(geometric_margins(point+np.eye(3)[i]*1e-6,track['observations'],states,args['geometry'])[0]
            -geometric_margins(point-np.eye(3)[i]*1e-6,track['observations'],states,args['geometry'])[0])/2e-6 for i in range(3)])
        np.testing.assert_allclose(analytic,numerical,atol=1e-8)

    def test_stationary_track_preserves_multiple_elevation_hypotheses(self):
        args=fixture(stationary=True);track=args['tracks'][0]
        result=point_hypotheses(observations=track['observations'],
            states={round(s['stamp']*1e9):s for s in args['states']},
            measurement_covariance=track['measurement_covariance'],geometry=args['geometry'])
        self.assertEqual(len(result['hypotheses']),9)
        self.assertTrue(all(h['local_rank']==2 for h in result['hypotheses']))
        self.assertGreater(np.ptp([h['point'][2] for h in result['hypotheses']]),2)
        self.assertFalse(result['elevation_branch_qualified'])
        self.assertTrue(all(h['minimum_elevation_margin_rad']>=0 for h in result['hypotheses']))

    def test_translating_poses_recover_a_known_point_seed(self):
        args=fixture();track=args['tracks'][0]
        result=point_hypotheses(observations=track['observations'],
            states={round(s['stamp']*1e9):s for s in args['states']},
            measurement_covariance=track['measurement_covariance'],geometry=args['geometry'])
        best=result['hypotheses'][0]
        self.assertLess(best['cost'],1e-8)
        np.testing.assert_allclose(best['point'],track['landmark'],atol=1e-4)

    def test_bad_noise_is_not_reported_as_a_geometric_seed_failure(self):
        args=fixture();track=args['tracks'][0]
        with self.assertRaises(ValueError):
            point_hypotheses(observations=track['observations'],
                states={round(s['stamp']*1e9):s for s in args['states']},
                measurement_covariance=np.zeros((4,4)),geometry=args['geometry'])


if __name__=='__main__':unittest.main()
