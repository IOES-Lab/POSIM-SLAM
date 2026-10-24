import unittest
import numpy as np
from test_sonar_inertial import fixture
from posim_slam.sonar_triangulation import point_hypotheses


class TriangulationTests(unittest.TestCase):
    def test_stationary_track_preserves_multiple_elevation_hypotheses(self):
        args=fixture(stationary=True);track=args['tracks'][0]
        result=point_hypotheses(observations=track['observations'],
            states={round(s['stamp']*1e9):s for s in args['states']},
            measurement_covariance=track['measurement_covariance'],geometry=args['geometry'])
        self.assertEqual(len(result['hypotheses']),9)
        self.assertTrue(all(h['local_rank']==2 for h in result['hypotheses']))
        self.assertGreater(np.ptp([h['point'][2] for h in result['hypotheses']]),2)
        self.assertFalse(result['elevation_branch_qualified'])

    def test_translating_poses_recover_a_known_point_seed(self):
        args=fixture();track=args['tracks'][0]
        result=point_hypotheses(observations=track['observations'],
            states={round(s['stamp']*1e9):s for s in args['states']},
            measurement_covariance=track['measurement_covariance'],geometry=args['geometry'])
        best=result['hypotheses'][0]
        self.assertLess(best['cost'],1e-8)
        np.testing.assert_allclose(best['point'],track['landmark'],atol=1e-4)


if __name__=='__main__':unittest.main()
