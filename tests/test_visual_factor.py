import copy
import unittest
import numpy as np
from posim_slam.visual_factor import linearize_visual_track
from posim_slam.inertial import exp_so3
from posim_slam.sonar_inertial import optimize_window
from test_sonar_inertial import fixture


def visual_fixture():
    args=fixture(stationary=True)
    geometry=dict(imu_to_camera=np.zeros(3),camera_to_imu=np.array([[0.,0.,1.],[-1.,0.,0.],[0.,-1.,0.]]),
        intrinsics=[300.,300.,320.,240.],image_size=[640,480],near=.1,far=10.)
    args['visual_geometry']=geometry
    for track in args['tracks']:
        observations=[]
        for state in args['states']:
            q=geometry['camera_to_imu'].T@(track['landmark']-state['position'])
            observations.append(dict(stamp_ns=round(state['stamp']*1e9),pixel=(300*q[:2]/q[2]+[320,240]).tolist()))
        track['visual_observations']=observations;track['visual_measurement_covariance']=np.eye(4)*.5**2
    return args


class VisualFactorTests(unittest.TestCase):
    def test_visual_pose_point_jacobian(self):
        args=visual_fixture();track=args['tracks'][0];states={round(s['stamp']*1e9):s for s in args['states']}
        inputs=dict(observations=track['visual_observations'],measurement_covariance=track['visual_measurement_covariance'],**args['visual_geometry'])
        block=linearize_visual_track(states=states,landmark=track['landmark'],**inputs)
        numerical=np.empty_like(block['jacobian'])
        for k in range(15):
            values=[]
            for sign in (-1,1):
                altered=copy.deepcopy(states);point=track['landmark'].copy();d=sign*1e-6
                if k>=12:point[k-12]+=d
                else:
                    state=altered[(k//6)*1_000_000_000];axis=k%6
                    if axis<3:state['position'][axis]+=d
                    else:state['attitude']=state['attitude']@exp_so3(np.eye(3)[axis-3]*d)
                values.append(linearize_visual_track(states=altered,landmark=point,**inputs)['residual'])
            numerical[:,k]=(values[1]-values[0])/2e-6
        np.testing.assert_allclose(block['jacobian'],numerical,atol=1e-7)

    def test_joint_pixels_resolve_stationary_sonar_height_in_known_association_case(self):
        args=visual_fixture();expected=[t['landmark'].copy() for t in args['tracks']]
        for track in args['tracks']:track['landmark']+=np.array([.03,.02,.04])
        result=optimize_window(**args,retain_state_stamps_ns=[1_000_000_000])
        self.assertLess(result['final_cost'],result['initial_cost']*1e-6)
        self.assertEqual(result['nullity'],0)
        np.testing.assert_allclose(result['landmarks'],expected,atol=1e-5)
        self.assertEqual(result['linearized_prior']['absorbed_measurements']['visual'][0]['stamps_ns'],[0,1_000_000_000])
        self.assertFalse(result['navigation_ready'])

    def test_visual_only_landmark_can_share_joint_pose(self):
        args=visual_fixture();args['tracks'][0]['observations']=[]
        result=optimize_window(**args)
        self.assertLess(result['final_cost'],1e-12)
        # No stationary parallax for this visual-only point; depth stays unknown.
        self.assertGreaterEqual(result['nullity'],1)

    def test_history_rejects_reused_pixels_and_changed_camera(self):
        args=visual_fixture();result=optimize_window(**args,retain_state_stamps_ns=[1_000_000_000])
        args.update(initial_state=None,initial_covariance=None,linearized_prior=result['linearized_prior'],
            independent_new_measurements=True)
        start=copy.deepcopy(result['states'][-1]);end=copy.deepcopy(start);end['stamp']=2.
        args['states']=[start,end]
        for track in args['tracks']:
            track['observations']=[];track['visual_observations']=track['visual_observations'][1:]
        with self.assertRaisesRegex(ValueError,'visual_observation_already_absorbed'):optimize_window(**args)
        args['visual_geometry']['intrinsics'][0]+=1
        with self.assertRaisesRegex(ValueError,'marginal_visual_model_changed'):optimize_window(**args)


if __name__=='__main__':unittest.main()
