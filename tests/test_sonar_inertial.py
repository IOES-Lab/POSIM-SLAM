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
    def delayed_ping_fixture(self):
        args=fixture();velocity=np.array([1.2,0.,0.])
        args['states']=[dict(stamp=t,position=velocity*t,velocity=velocity.copy(),
            attitude=np.eye(3),gyro_bias=np.zeros(3),accel_bias=np.zeros(3)) for t in (0.,1.,3.)]
        args['initial_state']=copy.deepcopy(args['states'][0])
        args['imu_window']=ImuWindow()
        for i in range(601):args['imu_window'].add(i*.005,[0,0,0],[0,0,9.80665])
        args['geometry']['vertical_fov']=1.1
        args['tracks']=[]
        for i,p in enumerate(([3.,-.4,-.8],[4.5,-.4,-.8],[3.,.4,-.8],[4.5,.4,-.8])):
            point=np.array(p);observations=[]
            # The sonar observes the patch before the downward Ping reaches it.
            for state in args['states'][:2]:
                q=point-state['position']
                observations.append(dict(stamp_ns=round(state['stamp']*1e9),
                    range_m=float(np.linalg.norm(q)),azimuth_rad=float(np.arctan2(q[1],q[0]))))
            args['tracks'].append(dict(track_id=i,landmark=point,observations=observations,
                measurement_covariance=np.diag([.01**2,.002**2]*2)))
        args['ping_patches']=[dict(track_ids=list(range(4)),plane_normal=np.array([0.,0.,1.]),
            plane_distance=-.8,surface_sigma=.01,max_support_distance=.9,
            observations=[dict(stamp_ns=3_000_000_000,range_m=.8,range_sigma=.01)],
            geometry=dict(imu_to_ping=np.zeros(3),ping_to_imu=np.eye(3),
                rays=np.array([[0.,0.,-1.]]),range_bias=0.,range_min=.1,range_max=10.))]
        return args

    def test_delayed_downward_ping_shares_historical_sonar_points(self):
        args=self.delayed_ping_fixture()
        for track in args['tracks']:track['landmark']+=np.array([.01,0.,-.025])
        args['ping_patches'][0]['plane_distance']-=.025
        args['states'][2]['position']+=np.array([.01,0.,.015])
        result=optimize_window(**args)
        self.assertLess(result['final_cost'],result['initial_cost']*1e-6)
        self.assertAlmostEqual(result['planes'][0]['distance'],-.8,places=4)
        # No invented sonar measurement at the Ping-only state, and no duplicate
        # surface rows: prior + two IMU edges + eight sonar returns + Ping + plane.
        self.assertEqual(result['residual_dimension'],15+30+16+1+4)
        self.assertFalse(result['navigation_ready'])

    def test_delayed_ping_still_requires_spatial_support(self):
        args=self.delayed_ping_fixture()
        args['ping_patches'][0]['observations'][0]['stamp_ns']=0
        with self.assertRaisesRegex(ValueError,'surface_does_not_cover_ping'):
            optimize_window(**args)

    def ping_fixture(self):
        args=fixture();tracks=[]
        for i,p in enumerate(([3.,-1.,-.8],[6.,-1.,-.8],[3.,1.,-.8],[6.,1.,-.8])):
            point=np.array(p);observations=[]
            for state in args['states']:
                q=point-state['position']
                observations.append(dict(stamp_ns=round(state['stamp']*1e9),
                    range_m=float(np.linalg.norm(q)),azimuth_rad=float(np.arctan2(q[1],q[0]))))
            tracks.append(dict(track_id=i,landmark=point,observations=observations,
                measurement_covariance=np.diag([.01**2,.002**2]*2)))
        ray=np.array([1.,0.,-.2]);ray/=np.linalg.norm(ray)
        patch=dict(track_ids=list(range(4)),plane_normal=np.array([0.,0.,1.]),plane_distance=-.8,
            surface_sigma=.01,max_support_distance=2.,observations=[dict(
                stamp_ns=round(s['stamp']*1e9),range_m=(-.8-s['position'][2])/ray[2],range_sigma=.01)
                for s in args['states']],geometry=dict(imu_to_ping=np.zeros(3),ping_to_imu=np.eye(3),
                    rays=np.array([ray]),range_bias=0.,range_min=.1,range_max=10.))
        args.update(tracks=tracks,ping_patches=[patch]);return args

    def test_ping_plane_and_sonar_points_are_joint_unknowns(self):
        args=self.ping_fixture()
        args['ping_patches'][0]['plane_distance']-=.03
        for track in args['tracks']:track['landmark']+=np.array([.02,.01,-.03])
        args['states'][1]['position']+=np.array([.01,-.01,.01])
        result=optimize_window(**args)
        self.assertLess(result['final_cost'],result['initial_cost']*1e-6)
        self.assertAlmostEqual(result['planes'][0]['distance'],-.8,places=4)
        np.testing.assert_allclose(result['planes'][0]['normal'],[0,0,1],atol=1e-4)
        self.assertEqual(result['variables'],45)
        # 15 prior + 15 IMU + 16 sonar + 2 Ping + 4 surface rows.
        self.assertEqual(result['residual_dimension'],52)
        self.assertFalse(result['navigation_ready'])

    def test_ping_rejects_duplicate_returns_and_overlapping_support(self):
        for duplicate in ('observations','patch'):
            args=self.ping_fixture();patch=args['ping_patches'][0]
            if duplicate=='observations':patch['observations']*=2
            else:args['ping_patches'].append(copy.deepcopy(patch))
            with self.subTest(duplicate=duplicate),self.assertRaises(ValueError):optimize_window(**args)

    def test_ping_rejects_unobserved_footprint(self):
        args=self.ping_fixture()
        args['ping_patches'][0]['geometry']['imu_to_ping']=np.array([10.,0.,0.])
        with self.assertRaisesRegex(ValueError,'surface_does_not_cover_ping'):optimize_window(**args)

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
