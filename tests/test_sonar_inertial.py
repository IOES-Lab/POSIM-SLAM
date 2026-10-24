import copy
import unittest
import numpy as np
from posim_slam.inertial import ImuWindow
from posim_slam.sonar_inertial import optimize_window


def fixture(stationary=False):
    velocity=np.array([0.,0.,0.] if stationary else [.4,.1,.15])
    states=[dict(stamp=float(t),position=velocity*t,velocity=velocity.copy(),
        attitude=np.eye(3),gyro_bias=np.zeros(3),accel_bias=np.zeros(3)) for t in (0,1)]
    imu=ImuWindow()
    for i in range(201):imu.add(i*.005,[0,0,0],[0,0,9.80665])
    points=[np.array(p,dtype=float) for p in ([4,.6,.5],[5,-.7,-.6])]
    tracks=[]
    for i,point in enumerate(points):
        observations=[]
        for s in states:
            q=point-s['position']
            observations.append(dict(stamp_ns=round(s['stamp']*1e9),
                range_m=float(np.linalg.norm(q)),azimuth_rad=float(np.arctan2(q[1],q[0]))))
        tracks.append(dict(track_id=i,landmark=point.copy(),observations=observations,
            measurement_covariance=np.diag([.01**2,.002**2]*2)))
    return dict(states=states,tracks=tracks,imu_window=imu,
        noise_density=[.002,.02,.0001,.001],gravity=[0,0,-9.80665],
        initial_state=copy.deepcopy(states[0]),initial_covariance=np.eye(15)*.001,
        geometry=dict(imu_to_sonar=np.zeros(3),sonar_to_imu=np.eye(3),
            range_min=.1,range_max=10.,horizontal_fov=2.,vertical_fov=.8))


class SonarInertialTests(unittest.TestCase):
    def test_joint_solve_reduces_pose_and_point_error(self):
        args=fixture();truth=copy.deepcopy(args['states']);points=[t['landmark'].copy() for t in args['tracks']]
        args['states'][1]['position']+=np.array([.03,-.02,.02])
        args['states'][1]['velocity']+=np.array([.02,.01,-.01])
        for t in args['tracks']:t['landmark']+=np.array([.04,.02,.03])
        result=optimize_window(**args)
        self.assertLess(result['final_cost'],result['initial_cost']*1e-6)
        self.assertLess(np.linalg.norm(result['states'][1]['position']-truth[1]['position']),1e-4)
        for actual,expected in zip(result['landmarks'],points):
            self.assertLess(np.linalg.norm(actual-expected),.01)
        self.assertFalse(result['navigation_ready'])
        self.assertFalse(result['elevation_branch_qualified'])

    def test_stationary_tracks_retain_unobserved_elevation(self):
        result=optimize_window(**fixture(stationary=True))
        self.assertGreaterEqual(result['nullity'],2)
        self.assertLess(result['final_cost'],1e-12)
        self.assertFalse(result['covariance_qualified'])

    def test_missing_state_and_unknown_prior_noise_rejected(self):
        args=fixture()
        for change in (dict(initial_covariance=np.zeros((15,15))),
                       dict(states=[args['states'][0]]*2)):
            with self.subTest(change=change),self.assertRaises(ValueError):
                optimize_window(**dict(args,**change))


if __name__=='__main__':unittest.main()
