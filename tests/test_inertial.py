import math,sys,unittest
from pathlib import Path
import numpy as np
from posim_slam.inertial import ImuWindow,exp_so3

NOISE=[.002,.02,.0001,.001]


class InertialTests(unittest.TestCase):
    def window(self,gyro=(0,0,0),accel=(0,0,9.80665),duration=1.,rate=200):
        result=ImuWindow()
        for t in np.arange(int(duration*rate)+1)/rate:result.add(float(t),gyro,accel)
        return result

    def integrate(self,window,start=0.,end=1.,**changes):
        arguments=dict(gyro_bias=[0,0,0],accel_bias=[0,0,0],noise_density=NOISE)
        arguments.update(changes)
        return window.preintegrate(start,end,**arguments)

    def test_rest_retains_specific_force_and_requires_gravity_in_estimator(self):
        result=self.integrate(self.window())
        np.testing.assert_allclose(result['rotation'],np.eye(3),atol=1e-12)
        np.testing.assert_allclose(result['delta_velocity'],[0,0,9.80665],atol=1e-10)
        np.testing.assert_allclose(result['delta_position'],[0,0,4.903325],atol=1e-10)
        self.assertFalse(result['gravity_included'])
        # A downstream state predictor adds R0*delta + g*dt, once only.
        np.testing.assert_allclose(result['delta_velocity']+[0,0,-9.80665],[0,0,0],atol=1e-10)

    def test_known_acceleration_bias_and_interpolated_boundaries(self):
        result=self.integrate(self.window(accel=(2.1,-.2,9.80665)),.123,.877,
                              accel_bias=[.1,-.2,0])
        dt=.754
        np.testing.assert_allclose(result['delta_velocity'],[2*dt,0,9.80665*dt],atol=1e-10)
        np.testing.assert_allclose(result['delta_position'],[dt**2,0,.5*9.80665*dt**2],atol=1e-10)

    def test_3d_rotation_and_acceleration_in_initial_frame(self):
        rate=math.pi/2
        result=self.integrate(self.window(gyro=(0,0,rate),accel=(1,0,0),rate=1000))
        np.testing.assert_allclose(result['rotation'],exp_so3([0,0,rate]),atol=1e-10)
        np.testing.assert_allclose(result['delta_velocity'],[1/rate,1/rate,0],atol=1e-6)
        np.testing.assert_allclose(result['delta_position'],[1/rate**2,1/rate-1/rate**2,0],atol=1e-6)

    def test_covariance_psd_cross_terms_and_grows_without_observations(self):
        window=self.window(duration=2.)
        a=self.integrate(window);b=self.integrate(window,end=2.)
        for result in (a,b):
            cov=result['covariance']
            np.testing.assert_allclose(cov,cov.T,atol=1e-14)
            self.assertGreater(np.linalg.eigvalsh(cov).min(),0)
            self.assertNotEqual(float(cov[0,3]),0)
            self.assertLess(float(cov[6,9]),0)
        self.assertGreater(np.trace(b['covariance']),np.trace(a['covariance']))

    def test_bias_jacobian_matches_reintegration(self):
        window=self.window(gyro=(.1,-.07,.2),accel=(.3,.2,9.8),rate=1000)
        base=self.integrate(window);eps=1e-5
        for axis in range(6):
            bg=np.zeros(3);ba=np.zeros(3)
            (bg if axis<3 else ba)[axis%3]=eps
            moved=self.integrate(window,gyro_bias=bg,accel_bias=ba)
            relative=base['rotation'].T@moved['rotation']
            angle=np.array([relative[2,1]-relative[1,2],relative[0,2]-relative[2,0],relative[1,0]-relative[0,1]])*.5
            finite=np.r_[moved['delta_position']-base['delta_position'],moved['delta_velocity']-base['delta_velocity'],angle]/eps
            np.testing.assert_allclose(base['bias_jacobian'][:,axis],finite,rtol=.005,atol=.005)

    def test_rejects_gaps_clock_reversal_and_invalid_measurements(self):
        for t,g,a,error in ((1.1,[0,0,0],[0,0,9.8],'gap'),
                            (1.,[0,0,0],[0,0,9.8],'reversal'),
                            (1.005,[math.nan,0,0],[0,0,9.8],'vector'),
                            (1.005,[40,0,0],[0,0,9.8],'saturated')):
            window=self.window()
            with self.assertRaisesRegex(ValueError,error):window.add(t,g,a)
            self.assertFalse(window.samples)
            window.add(2.,[0,0,0],[0,0,9.8])
            with self.assertRaisesRegex(ValueError,'covered'):self.integrate(window)

    def test_no_extrapolation_or_invented_noise(self):
        window=self.window()
        for start,end in ((0,1.01),(-.01,1),(1,0),(0,4),(math.nan,1)):
            with self.assertRaises(ValueError):self.integrate(window,start,end)
        for noise in ([0,.1,.1,.1],[-1,1,1,1],[math.nan,1,1,1],[1,1,1]):
            with self.assertRaisesRegex(ValueError,'calibration'):self.integrate(window,noise_density=noise)

    def test_memory_bound_at_actual_1000_hz(self):
        window=self.window(duration=10.,rate=1000)
        self.assertEqual(len(window.samples),8192)
        with self.assertRaisesRegex(ValueError,'covered'):self.integrate(window)
        result=self.integrate(window,7.,10.)
        self.assertAlmostEqual(result['dt'],3.)


if __name__=='__main__':unittest.main()
