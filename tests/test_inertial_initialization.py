import math,unittest
import numpy as np
from posim_slam.inertial import ImuWindow,exp_so3
from posim_slam.inertial_initialization import stationary_initialize

NOISE=[.001,.02,.0001,.001]

def inputs(attitude=np.eye(3),velocity=(0,0,0),gyro=(.002,-.001,.003),bias=(0,0,0)):
    imu=ImuWindow()
    for t in np.arange(251)/100:imu.add(float(t),gyro,attitude.T@[0,0,9.80665]+bias)
    dvl=[(float(t),velocity,np.eye(3)*1e-5,True) for t in [0,1,2,3]]
    return imu,dvl

def initialize(imu,dvl,**changes):
    args=dict(dvl_to_imu=np.eye(3),imu_to_dvl=[.2,0,-.1],accel_bias=[0,0,0],
        accel_bias_covariance=np.eye(3)*.01,noise_density=NOISE,
        angular_stationarity_std=.001,linear_stationarity_std=.01)
    args.update(changes);return stationary_initialize(imu,0.,2.5,dvl,**args)

class InitializationTests(unittest.TestCase):
    def test_level_and_inverted_mount_need_no_orientation_input(self):
        for r in [np.eye(3),exp_so3([math.pi,0,0]),exp_so3([0,math.pi/2,0])]:
            imu,dvl=inputs(r);f,diagnostics=initialize(imu,dvl)
            np.testing.assert_allclose(f.attitude@(r.T@[0,0,1]),[0,0,1],atol=1e-12)
            np.testing.assert_allclose(f.attitude.T@f.attitude,np.eye(3),atol=1e-12)
            self.assertAlmostEqual(np.linalg.det(f.attitude),1)
            np.testing.assert_allclose(f.gyro_bias,[.002,-.001,.003],atol=1e-15)
            self.assertFalse(diagnostics['orientation_measurement_used'])
            self.assertEqual(diagnostics['initial_yaw'],'local_gauge')
            self.assertGreater(np.linalg.eigvalsh(f.covariance).min(),0)

    def test_tilted_frame_gravity_cancels_without_truth_heading(self):
        r=exp_so3([.3,-.2,1.]);imu,dvl=inputs(r);f,_=initialize(imu,dvl)
        w=ImuWindow()
        for t in np.arange(51)/100+2.5:w.add(float(t),[.002,-.001,.003],r.T@[0,0,9.80665])
        f.propagate(w,3.)
        np.testing.assert_allclose(f.position,0,atol=1e-12)
        np.testing.assert_allclose(f.velocity,0,atol=1e-12)

    def test_accel_bias_prior_and_tilt_correlation_are_preserved(self):
        ba=np.array([.1,-.05,.08]);imu,dvl=inputs(bias=ba);f,d=initialize(imu,dvl,accel_bias=ba)
        np.testing.assert_allclose(f.accel_bias,ba);np.testing.assert_allclose(f.covariance[12:15,12:15],np.eye(3)*.01)
        self.assertGreater(np.linalg.norm(f.covariance[6:9,12:15]),0)
        self.assertFalse(d['accel_bias_estimated'])

    def test_independent_dvl_rejects_constant_velocity_invisible_to_static_imu(self):
        imu,dvl=inputs(velocity=(.1,0,0))
        with self.assertRaisesRegex(ValueError,'dvl_not_stationary'):initialize(imu,dvl)
        imu,dvl=inputs();dvl[-1]=(3.,[0,0,0],np.eye(3)*1e-5,False)
        with self.assertRaisesRegex(ValueError,'bottom_lock'):initialize(imu,dvl)

    def test_low_dynamics_gates_do_not_assert_navigation_observability(self):
        # Slow constant rotation about gravity with zero velocity at the DVL
        # origin is ambiguous with gyro bias. Static acceptance is conditional.
        f,d=initialize(*inputs(gyro=(0,0,.01)))
        self.assertTrue(d['stationary_assumption'])
        self.assertFalse(d['navigation_ready'])
        self.assertAlmostEqual(f.gyro_bias[2],.01)

    def test_motion_and_wrong_gravity_reject(self):
        for gyro,bias in [([0,0,.2],[0,0,0]),([0,0,0],[0,0,1])]:
            with self.assertRaisesRegex(ValueError,'imu_not_stationary'):initialize(*inputs(gyro=gyro,bias=bias))
        imu,dvl=inputs()
        for i,(t,g,a) in enumerate(imu.samples):imu.samples[i]=(t,g,a+[.3*math.sin(t*10),0,0])
        with self.assertRaisesRegex(ValueError,'imu_not_stationary'):initialize(imu,dvl)

    def test_real_coverage_covariance_and_calibration_required(self):
        imu,dvl=inputs()
        for changes in [dict(accel_bias_covariance=np.zeros((3,3))),dict(noise_density=[0]*4),dict(angular_stationarity_std=0)]:
            with self.assertRaises(ValueError):initialize(imu,dvl,**changes)
        for samples in [dvl[1:],dvl[:-1],[(0.,[0,0,0],np.eye(3)*1e-5,True),(2.,[0,0,0],np.eye(3)*1e-5,True),(3.,[0,0,0],np.eye(3)*1e-5,True)]]:
            with self.assertRaises(ValueError):initialize(imu,samples)
        dvl[0]=(0.,[0,0,0],np.zeros((3,3)),True)
        with self.assertRaisesRegex(ValueError,'positive_covariance'):initialize(imu,dvl)

if __name__=='__main__':unittest.main()
