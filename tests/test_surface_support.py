import unittest
import numpy as np
from posim_slam.inertial import exp_so3
from posim_slam.surface_support import require_surface_support


class SurfaceSupportTests(unittest.TestCase):
    def test_rejects_box_overlap_outside_actual_triangle(self):
        # Bounding-box checks accept (0.8,0.8), though x+y>1 is unobserved.
        p=np.array([[0,0,2],[1,0,2],[0,1,2]])
        with self.assertRaisesRegex(ValueError,'does_not_cover'):
            require_surface_support(p,[[.8,.8,2]],[0,0,1],max_distance=1.)

    def test_rejects_hole_inside_hull(self):
        edge=np.linspace(-1,1,30)
        p=np.array([[x,y,2] for x in edge for y in edge if max(abs(x),abs(y))>.8])
        with self.assertRaisesRegex(ValueError,'support_gap'):
            require_surface_support(p,[[0,0,2]],[0,0,1],max_distance=.2)

    def test_support_invariant_to_rotation_translation_and_scale(self):
        p=np.array([[x,y,2] for x in np.linspace(-1,1,20) for y in np.linspace(-1,1,20)])
        q=np.array([[.15,.2,2],[-.5,.7,2]])
        r=exp_so3([.8,-.3,1.2]);shift=np.array([3.,4.,-2.])
        for scale in (.01,1.,100.):
            require_surface_support(scale*p@r.T+shift,scale*q@r.T+shift,
                                    r@np.array([0,0,1.]),max_distance=.1*scale)

    def test_invalid_and_degenerate_inputs(self):
        with self.assertRaisesRegex(ValueError,'degenerate'):
            require_surface_support([[0,0,1],[1,0,1],[2,0,1]],[[1,0,1]],[0,0,1],max_distance=1.)
        for limit in (0.,-1.,float('nan')):
            with self.assertRaises(ValueError):
                require_surface_support([[0,0,1],[1,0,1],[0,1,1]],[[0,0,1]],[0,0,1],max_distance=limit)


if __name__=='__main__':unittest.main()
