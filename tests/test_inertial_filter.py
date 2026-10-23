import math,unittest
import numpy as np
from posim_slam.inertial import ImuWindow,exp_so3
from posim_slam.inertial_filter import InertialFilter

NOISE=[.002,.02,.0001,.001]
def estimator(**changes):
    args=dict(stamp=0.,position=[0,0,0],velocity=[0,0,0],attitude=np.eye(3),
              gyro_bias=[0,0,0],accel_bias=[0,0,0],state_covariance=np.eye(15)*.01,noise_density=NOISE)
    args.update(changes);return InertialFilter(**args)
def window(gyro=(0,0,0),force=(0,0,9.80665),duration=1):
    w=ImuWindow()
    for t in np.arange(round(duration*200)+1)/200:w.add(float(t),gyro,force)
    return w

def dvl(f,velocity,**changes):
    args=dict(stamp=f.stamp,gyro=[0,0,0],dvl_to_imu=np.eye(3),imu_to_dvl=[0,0,0],gyro_covariance=np.eye(3)*1e-5)
    args.update(changes);return f.update_dvl(velocity,np.eye(3)*.001,**args)

class FilterTests(unittest.TestCase):
    def test_gravity_once_and_mount_rotation(self):
        r=exp_so3([math.pi,0,0]);f=estimator(attitude=r)
        f.propagate(window(force=(0,0,-9.80665)),1.)
        np.testing.assert_allclose(f.position,0,atol=1e-12)
        np.testing.assert_allclose(f.velocity,0,atol=1e-12)
        self.assertGreater(np.trace(f.covariance),.15)

    def test_acceleration_and_rotation_have_analytic_solution(self):
        f=estimator(gravity=[0,0,0]);f.propagate(window(gyro=(0,0,math.pi/2),force=(1,0,0)),1.)
        a=math.pi/2
        np.testing.assert_allclose(f.velocity,[1/a,1/a,0],atol=2e-6)
        np.testing.assert_allclose(f.position,[1/a**2,1/a-1/a**2,0],atol=3e-6)
        np.testing.assert_allclose(f.attitude,exp_so3([0,0,a]),atol=1e-12)

    def test_dvl_lever_arm_and_mount_prevent_fake_translation(self):
        f=estimator();r=exp_so3([0,0,.8]);lever=np.array([.4,-.2,.1]);omega=np.array([0,0,.5])
        result=dvl(f,r.T@np.cross(omega,lever),gyro=omega,dvl_to_imu=r,imu_to_dvl=lever)
        self.assertTrue(result['accepted']);self.assertAlmostEqual(result['nis'],0)
        np.testing.assert_allclose(f.velocity,0,atol=1e-15)

    def test_dvl_outlier_no_state_change_and_duplicate_rejected(self):
        f=estimator();before=f.covariance.copy()
        result=dvl(f,[100,0,0]);self.assertFalse(result['accepted'])
        np.testing.assert_array_equal(f.covariance,before);np.testing.assert_array_equal(f.velocity,0)
        with self.assertRaisesRegex(ValueError,'already_consumed'):dvl(f,[0,0,0])
        with self.assertRaisesRegex(ValueError,'time_mismatch'):dvl(estimator(),[0,0,0],stamp=.1)

    def test_vertical_accel_bias_is_updated_by_independent_velocity_observations(self):
        f=estimator(state_covariance=np.diag([.01]*6+[1e-8]*3+[.01]*6))
        w=window(force=(0,0,9.92665),duration=3)
        for t in np.arange(1,31)/10:
            f.propagate(w,float(t));self.assertTrue(dvl(f,[0,0,0])['accepted'])
        self.assertAlmostEqual(f.accel_bias[2],.12,delta=.008)
        self.assertLess(np.linalg.norm(f.velocity),.005)
        self.assertGreater(f.updates['dvl'],20)
        self.assertGreater(np.linalg.eigvalsh(f.covariance).min(),0)

    def test_clone_covariance_and_relative_zero_residual(self):
        f=estimator(velocity=[.4,0,0]);f.clone_pose();anchor=f.covariance.copy()
        self.assertEqual(anchor.shape,(21,21));np.testing.assert_allclose(anchor[:3,15:18],np.eye(3)*.01)
        f.propagate(window(),1.)
        result=f.update_relative_planar([.4,0],0,np.eye(3)*.001,stamp=1.,sonar_to_imu=np.eye(3),imu_to_sonar=[.2,0,0])
        self.assertTrue(result['accepted']);self.assertLess(result['nis'],1e-20)
        self.assertGreaterEqual(np.linalg.eigvalsh(f.covariance).min(),-1e-12)
        f.clone_pose();self.assertEqual(f.covariance.shape,(21,21))

    def test_planar_update_rejects_tilt_and_unresolved_normal_motion(self):
        for velocity,gyro in [([0,0,.3],[0,0,0]),([0,0,0],[.3,0,0])]:
            f=estimator(velocity=velocity);f.clone_pose();f.propagate(window(gyro=gyro),1.)
            with self.assertRaisesRegex(ValueError,'planar_motion_not_supported'):
                f.update_relative_planar([0,0],0,np.eye(3),stamp=1.,sonar_to_imu=np.eye(3),imu_to_sonar=[0,0,0])

    def test_depth_uses_calibrated_sensor_height_and_offset(self):
        r=exp_so3([.2,-.1,0]);lever=np.array([.2,.1,.3]);f=estimator(attitude=r,position=[0,0,-3])
        z=float((f.position+r@lever)[2]);result=f.update_depth(z,.001,stamp=0.,imu_to_pressure=lever)
        self.assertTrue(result['accepted']);self.assertEqual(result['nis'],0)
        self.assertLess(f.covariance[2,2],.01)
        f=estimator(position=[0,0,-2.9]);result=f.update_depth(-3,.001,stamp=0.,imu_to_pressure=[0,0,0])
        self.assertTrue(result['accepted']);self.assertLess(abs(f.position[2]+3),.01)

    def test_three_consecutive_innovation_rejections_invalidate_segment(self):
        f=estimator();w=window()
        for i in range(3):
            f.propagate(w,(i+1)*.1);result=dvl(f,[100,0,0])
            self.assertFalse(result['accepted'])
            self.assertEqual(f.valid,i<2)
        self.assertEqual(f.invalid_reason,'dvl_innovation_rejected')
        with self.assertRaisesRegex(ValueError,'segment_invalid'):f.clone_pose()
        with self.assertRaisesRegex(ValueError,'segment_invalid'):dvl(f,[0,0,0])

    def test_relative_motion_does_not_observe_absolute_position_gauge(self):
        f=estimator(velocity=[.4,0,0]);f.clone_pose();f.propagate(window(),1.)
        f.update_relative_planar([.4,0],0,np.eye(3)*1e-5,stamp=1.,sonar_to_imu=np.eye(3),imu_to_sonar=[0,0,0])
        # Relative data cannot erase the 1 cm² common translation uncertainty.
        self.assertGreaterEqual(f.covariance[0,0],.01-1e-12)
        self.assertAlmostEqual(f.covariance[15,15],.01,delta=1e-12)

    def test_invalid_calibration_and_missing_imu_do_not_invent_state(self):
        for changes in [dict(state_covariance=np.zeros((15,15))),dict(noise_density=[0]*4),dict(attitude=np.zeros((3,3)))]:
            with self.assertRaises(ValueError):estimator(**changes)
        f=estimator();before=f.covariance.copy()
        with self.assertRaisesRegex(ValueError,'not_covered'):f.propagate(window(duration=.5),1.)
        self.assertEqual(f.stamp,0);np.testing.assert_array_equal(f.covariance,before)
        self.assertFalse(f.valid)
        with self.assertRaisesRegex(ValueError,'segment_invalid'):f.propagate(window(),1.)

if __name__=='__main__':unittest.main()
