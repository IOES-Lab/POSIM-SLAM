import unittest
import numpy as np
from posim_slam.constrained_step import feasible_step


class StepTests(unittest.TestCase):
    def test_optimum_on_two_bounds(self):
        result=feasible_step(np.eye(3),[-3,-4,-.5],[1,2],[[-1,0,0],[0,-1,0]])
        np.testing.assert_allclose(result['step'],[1,2,.5],atol=1e-10)
        self.assertTrue(result['converged'])

    def test_rank_deficiency_adds_no_motion_in_unobserved_direction(self):
        result=feasible_step([[1,0,0]],[2],[1],[[1,0,0]])
        np.testing.assert_allclose(result['step'],[-1,0,0],atol=1e-10)

    def test_infeasible_origin_rejected(self):
        with self.assertRaises(ValueError):feasible_step(np.eye(3),[1,2,3],[-1],[[1,0,0]])


if __name__=='__main__':unittest.main()
