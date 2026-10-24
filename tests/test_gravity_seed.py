import unittest
import numpy as np
from posim_slam.inertial import ImuWindow,exp_so3
from posim_slam.gravity_seed import gravity_seed

class GravitySeedTests(unittest.TestCase):
    def window(self,acceleration=None):
        window=ImuWindow(max_samples=1000,max_gap=.02)
        omega=np.array([.3,-.2,.1]);bias=np.array([.01,-.02,.005]);accel_bias=np.array([.1,-.05,.02])
        world=np.array([0.,0.,9.80665])+(np.zeros(3) if acceleration is None else acceleration)
        for t in np.linspace(0,1,101):window.add(float(t),omega+bias,exp_so3(omega*t).T@world+accel_bias)
        return window,omega,bias,accel_bias
    def test_transport_to_endpoint_with_explicit_biases(self):
        window,omega,bg,ba=self.window()
        result=gravity_seed(window,0.,1.,gyro_bias=bg,accel_bias=ba)
        expected=exp_so3(omega).T@np.array([0.,0.,1.])
        np.testing.assert_allclose(result['direction_imu'],expected,atol=1e-12)
        self.assertFalse(result['observability_qualified'])
    def test_constant_acceleration_cannot_be_removed_or_qualified(self):
        window,omega,bg,ba=self.window(np.array([1.,0.,0.]))
        result=gravity_seed(window,0.,1.,gyro_bias=bg,accel_bias=ba)
        true_up=exp_so3(omega).T@np.array([0.,0.,1.])
        self.assertGreater(np.degrees(np.arccos(result['direction_imu']@true_up)),5.)
        self.assertFalse(result['velocity_estimated'])
        self.assertFalse(result['observability_qualified'])
    def test_missing_coverage_and_degenerate_force(self):
        window,_,bg,ba=self.window()
        with self.assertRaisesRegex(ValueError,'not_covered'):gravity_seed(window,0.,2.,gyro_bias=bg,accel_bias=ba)
        with self.assertRaisesRegex(ValueError,'duration'):gravity_seed(window,0.,.1,gyro_bias=bg,accel_bias=ba)
        empty=ImuWindow(max_samples=1000,max_gap=.02)
        for t in np.linspace(0,1,101):empty.add(float(t),[0,0,0],[0,0,0])
        with self.assertRaisesRegex(ValueError,'degenerate'):gravity_seed(empty,0.,1.,gyro_bias=[0,0,0],accel_bias=[0,0,0])
if __name__=='__main__':unittest.main()
