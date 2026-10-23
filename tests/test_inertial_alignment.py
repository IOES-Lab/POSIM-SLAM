import unittest
import numpy as np
from posim_slam.inertial import ImuWindow
from posim_slam.inertial_alignment import moving_initialize


def inputs(inverted=False):
 w=ImuWindow()
 for t in np.arange(801)/100:w.add(float(t),[.002,-.001,.003],[.03,-.02,(-9.80665 if inverted else 9.80665)+.04])
 return w,[(float(t),[.3,0,0],np.eye(3)*1e-4,True) for t in range(9)]

def align(w,dvl,**changes):
 args=dict(dvl_to_imu=np.eye(3),imu_to_dvl=[.2,0,-.1],gyro_bias=[0,0,0],gyro_bias_covariance=np.eye(3)*.01**2,
  accel_bias=[0,0,0],accel_bias_covariance=np.eye(3)*.1**2,noise_density=[.001,.02,.0001,.001])
 args.update(changes);return moving_initialize(w,dvl,**args)

class AlignmentTests(unittest.TestCase):
 def test_translation_and_inverted_mount_do_not_require_stationarity(self):
  for inverted in (False,True):
   f,d=align(*inputs(inverted))
   self.assertFalse(d['stationary_required']);self.assertFalse(d['navigation_ready'])
   self.assertLess(np.linalg.norm(f.velocity-[.3,0,0]),.01)
   self.assertEqual(f.stamp,8.)
   self.assertGreater(np.linalg.eigvalsh(f.covariance).min(),0)
   self.assertGreater(np.linalg.norm(f.covariance[6:9,12:15]),0)
   self.assertLess(abs(f.gyro_bias[2]),.002) # unexcited yaw bias stays near its calibration prior

 def test_missing_lock_gaps_and_unknown_covariance_reject(self):
  w,dvl=inputs()
  for sample in [(3.,[.3,0,0],np.eye(3)*1e-4,False),(3.,[.3,0,0],np.zeros((3,3)),True),(2.,[.3,0,0],np.eye(3)*1e-4,True)]:
   changed=dvl.copy();changed[3]=sample
   with self.assertRaises(ValueError):align(w,changed)
  with self.assertRaisesRegex(ValueError,'samples_required'):align(w,dvl[:4])
  with self.assertRaisesRegex(ValueError,'not_covered'):align(ImuWindow(),dvl)

 def test_disagreeing_dvl_outlier_is_not_absorbed_as_initial_motion(self):
  w,dvl=inputs();dvl[4]=(4.,[10,0,0],np.eye(3)*1e-4,True)
  with self.assertRaisesRegex(ValueError,'moving_alignment_(residual_rejected|iteration_limit|not_converged)'):align(w,dvl)

if __name__=='__main__':unittest.main()
