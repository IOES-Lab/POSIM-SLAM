import unittest
import numpy as np
from posim_slam.inertial import exp_so3,ImuWindow
from posim_slam.inertial_session import InertialDvlSession


def session():
    return InertialDvlSession(dict(dvl_to_imu=np.eye(3),imu_to_dvl=[0,0,0],
        gyro_bias=[0,0,0],gyro_bias_covariance=np.zeros((3,3)),
        accel_bias=[0,0,0],accel_bias_covariance=np.zeros((3,3)),
        noise_density=[.001,.02,.0001,.001]))


def feed(s,start=0,end=15):
    for i in range(round(start*100),round(end*100)+1):
        t=i/100;roll=.1*np.sin(t);r=exp_so3([roll,0,0])
        # DVL can arrive before its IMU endpoint; it must wait for coverage.
        if i%100==0:s.dvl(t,r.T@np.array([.3,0,0]),np.eye(3)*1e-4,True,t)
        s.imu(t,[.1*np.cos(t),0,0],r.T@np.array([0,0,9.80665]),t)


class StreamingTests(unittest.TestCase):
    def test_moving_initialization_and_endpoint_order(self):
        s=session();feed(s)
        value=s.snapshot()
        self.assertEqual(value['state'],'tracking');self.assertEqual(value['stamp'],15)
        self.assertGreaterEqual(s.accepted,2)
        self.assertLess(np.linalg.norm(s.filter.position-[.9,0,0]),.02)
        self.assertFalse(value['navigation_ready'])

    def test_lock_loss_clears_pose_and_recovers_in_new_segment(self):
        s=session();feed(s);old=s.segment
        s.dvl(16,[0,0,0],np.eye(3),False,16)
        self.assertIsNone(s.snapshot()['position']);self.assertGreater(s.segment,old)
        feed(s,17,31)
        self.assertEqual(s.state,'tracking');self.assertGreater(s.segment,old)

    def test_weak_bias_prior_does_not_promote_a_small_residual_to_tracking(self):
        s=session();s.calibration['gyro_bias_covariance']=np.eye(3)*.01**2
        feed(s)
        self.assertEqual(s.reason,'initial_uncertainty_too_large')
        self.assertIsNone(s.filter)
        self.assertGreater(s.diagnostics['attitude_three_sigma_deg'],3)

    def test_clock_gap_and_wall_staleness_clear_state(self):
        s=session();feed(s)
        s.tick(17.1);self.assertEqual(s.reason,'imu_stale');self.assertIsNone(s.filter)
        segment=s.segment;s.tick(100);self.assertEqual(s.segment,segment)
        s=session();feed(s);s.imu(15.1,[0,0,0],[0,0,9.8],15.1)
        self.assertEqual(s.reason,'imu_time_gap');self.assertIsNone(s.filter)

    def test_high_rate_window_and_capacity_are_bounded(self):
        w=ImuWindow(max_samples=32768)
        for i in range(33000):w.add(i/1000,[0,0,0],[0,0,9.8])
        self.assertEqual(len(w.samples),32768)
        self.assertGreater(w.samples[-1][0]-w.samples[0][0],30)
        for bad in (0,65537,True):
            with self.assertRaises(ValueError):ImuWindow(max_samples=bad)


if __name__=='__main__':unittest.main()
