import copy
import json
import math
import os
import subprocess
import unittest
import numpy as np
from posim_slam.inertial import exp_so3
from posim_slam.ping_joint import eliminate_patch
from posim_slam.sonar_ground_joint import linearize_ground_patch


def fixture(flat=False):
    normal = np.array([0., 0., 1.]) if flat else np.array([.1, -.08, 1.])
    normal /= np.linalg.norm(normal)
    mount = exp_so3([0., .4, 0.])
    lever = np.array([.13, -.06, .08])
    states = {i+1: dict(position=np.array([.2*i, .07*(i % 2), .03*i]),
                       attitude=np.eye(3) if flat else exp_so3([.01*i, -.015*i, .02*i])) for i in range(4)}
    az = np.linspace(-.6, .6, 9); elevation = -.2
    rays = np.column_stack([np.cos(az)*math.cos(elevation), np.sin(az)*math.cos(elevation),
                            np.full(len(az), math.sin(elevation))])
    observations = []
    for stamp, state in states.items():
        origin = state['position']+state['attitude']@lever
        world_rays = rays@mount.T@state['attitude'].T
        ranges = (-2.-normal@origin)/(world_rays@normal)
        observations.append(dict(stamp_ns=stamp, beam_ids=list(range(len(az))), azimuth_rad=az,
                                 range_m=ranges, boundary_sign=-1))
    count = sum(len(o['range_m']) for o in observations)
    noise = np.eye(count)*.03**2 + np.ones((count, count))*.01**2
    return dict(observations=observations, states=states, plane_normal=normal, plane_distance=-2.,
                imu_to_sonar=lever, sonar_to_imu=mount, vertical_fov=.4,
                range_min=.2, range_max=10., measurement_covariance=noise)


class GroundJointTests(unittest.TestCase):
    def test_derivatives_with_nontrivial_mount_and_correlated_noise(self):
        a=fixture();base=linearize_ground_patch(**a);epsilon=1e-6;columns=[]
        for k in range(base['jacobian'].shape[1]):
            values=[]
            for sign in (-1, 1):
                b=copy.deepcopy(a)
                if k<24:
                    state=b['states'][k//6+1];delta=np.eye(3)[k%3]*sign*epsilon
                    if k%6<3:state['position']+=delta
                    else:state['attitude']=state['attitude']@exp_so3(delta)
                elif k<26:b['plane_normal']=exp_so3(base['normal_basis'][:,k-24]*sign*epsilon)@a['plane_normal']
                else:b['plane_distance']+=sign*epsilon
                values.append(linearize_ground_patch(**b)['residual'])
            columns.append((values[1]-values[0])/(2*epsilon))
        np.testing.assert_allclose(np.array(columns).T,base['jacobian'],atol=2e-7)

    def test_flat_nullspaces_and_single_view_give_no_pose_prior(self):
        a=fixture(flat=True);f=linearize_ground_patch(**a)
        reduced=eliminate_patch(f['jacobian'],f['residual'],retained_columns=24)
        self.assertEqual(reduced['nuisance_rank'],3)
        self.assertEqual(np.linalg.matrix_rank(reduced['jacobian'],tol=1e-7),9)
        for i in range(4):
            np.testing.assert_allclose(reduced['jacobian'][:,[6*i,6*i+1,6*i+5]],0.,atol=1e-11)
        b=dict(a,observations=a['observations'][:1],measurement_covariance=a['measurement_covariance'][:9,:9])
        one=linearize_ground_patch(**b)
        np.testing.assert_allclose(eliminate_patch(one['jacobian'],one['residual'],retained_columns=6)['jacobian'],0.,atol=1e-11)

    def test_world_gauges_and_dense_correlated_elimination(self):
        a=fixture();f=linearize_ground_patch(**a);j=f['jacobian'];r=f['residual']
        reduced=eliminate_patch(j,r,retained_columns=24)
        for axis in np.eye(3):
            translation=np.tile(np.r_[axis,np.zeros(3)],4)
            turn=np.concatenate([np.r_[np.cross(axis,s['position']),s['attitude'].T@axis] for s in a['states'].values()])
            for gauge in (translation,turn):np.testing.assert_allclose(reduced['jacobian']@gauge,0.,atol=1e-10)
        plain=linearize_ground_patch(**dict(a,measurement_covariance=np.eye(36)))
        expected=plain['jacobian'].T@np.linalg.solve(a['measurement_covariance'],plain['jacobian'])
        np.testing.assert_allclose(j.T@j,expected,atol=1e-9)
        delta=np.linspace(-.01,.02,24)
        nuisance=np.linalg.lstsq(j[:,-3:],-(r+j[:,:24]@delta),rcond=None)[0]
        self.assertAlmostEqual(np.linalg.norm(r+j[:,:24]@delta+j[:,-3:]@nuisance)**2,
            np.linalg.norm(reduced['residual']+reduced['jacobian']@delta)**2,places=9)

    def test_rejects_false_identity_covariance_and_aperture_branch(self):
        for kind in ('duplicate_beam','duplicate_time','missing_pose','covariance','wrong_branch','inside_fan'):
            a=fixture()
            if kind=='duplicate_beam':a['observations'][0]['beam_ids'][1]=0
            elif kind=='duplicate_time':a['observations'][1]['stamp_ns']=1
            elif kind=='missing_pose':a['states'].pop(1)
            elif kind=='covariance':a['measurement_covariance'][:]=0
            elif kind=='wrong_branch':a['observations'][0]['boundary_sign']=1
            else:
                a['sonar_to_imu']=exp_so3([0.,math.pi/2,0.]);a['plane_normal']=np.array([0.,0.,1.])
            with self.subTest(kind=kind),self.assertRaises(ValueError):linearize_ground_patch(**a)


@unittest.skipUnless(os.environ.get('POSIM_SLAM_GROUND_PROBE'),'compiled native ground probe required')
class NativeGroundTests(unittest.TestCase):
    def native(self,a,**extras):
        payload=json.dumps(dict(a,**extras),default=lambda v:v.tolist())
        result=subprocess.run([os.environ['POSIM_SLAM_GROUND_PROBE']],input=payload,text=True,
                              capture_output=True,timeout=30)
        if result.returncode:raise RuntimeError(result.stderr)
        return json.loads(result.stdout)

    def test_native_matches_reference_and_retirement(self):
        a=fixture();a['observations'][1]['range_m']+=.01
        py=linearize_ground_patch(**a);change=np.eye(27)
        for i,s in enumerate(a['states'].values()):change[6*i+3:6*i+6,6*i+3:6*i+6]=s['attitude'].T
        j=py['jacobian']@change;r=py['residual'];native=self.native(a,prior_sqrt=np.eye(24)*2)
        np.testing.assert_allclose(native['jacobian'],j,atol=1e-10)
        np.testing.assert_allclose(np.array(native['residual']).ravel(),r,atol=1e-10)
        reduced=eliminate_patch(j,r,retained_columns=24)
        np.testing.assert_allclose(native['reduced_H'],reduced['jacobian'].T@reduced['jacobian'],atol=1e-8)
        # Independently eliminate first pose + shared plane in a single dense
        # solve; the native marginal path must retain all correlated frame rows.
        whole=np.vstack([np.c_[2*np.eye(24),np.zeros((24,3))],j]);res=np.r_[np.zeros(24),r]
        permutation=list(range(6,24))+list(range(6))+list(range(24,27))
        marginal=eliminate_patch(whole[:,permutation],res,retained_columns=18)
        np.testing.assert_allclose(native['marginal_H'],marginal['jacobian'].T@marginal['jacobian'],atol=2e-8)
        np.testing.assert_allclose(np.array(native['marginal_g']).ravel(),marginal['jacobian'].T@marginal['residual'],atol=1e-9)
        self.assertEqual(native['retired_stamps'],[1,2,3,4])

    def test_native_flat_plane_keeps_unobservable_directions(self):
        a=fixture(flat=True);native=self.native(a)
        h=np.array(native['reduced_H'])
        self.assertEqual(np.linalg.matrix_rank(h,tol=1e-6),9)
        for i in range(4):np.testing.assert_allclose(h[:,[6*i,6*i+1,6*i+5]],0.,atol=1e-9)
        a['observations']=a['observations'][:1]
        a['measurement_covariance']=a['measurement_covariance'][:9,:9]
        np.testing.assert_allclose(self.native(a)['reduced_H'],0.,atol=1e-9)

    def test_joint_optimizer_corrects_relative_height_and_tilt(self):
        a=fixture(flat=True);truth=copy.deepcopy(a['states'])
        for i in range(2,5):
            a['states'][i]['position'][2]+=.1*math.sin(i)
            a['states'][i]['attitude']=exp_so3([.025*math.cos(i),.025*math.sin(i),0.])
        # An explicit synthetic pose prior fixes the first pose gauge. No known
        # floor is constrained; all range rows and the plane are solved jointly.
        prior=np.eye(24)*5.;prior[:6,:6]*=2000
        result=self.native(a,prior_sqrt=prior)
        before=[];after=[]
        for i in range(2,5):
            before.append(abs(a['states'][i]['position'][2]-truth[i]['position'][2]))
            p=np.array(result['states'][str(i)]['position']).ravel()
            after.append(abs(p[2]-truth[i]['position'][2]))
            np.testing.assert_allclose(p[:2],truth[i]['position'][:2],atol=1e-10)
            self.assertLess(np.linalg.norm(np.array(result['states'][str(i)]['attitude'])-np.eye(3)),
                            .75*np.linalg.norm(a['states'][i]['attitude']-np.eye(3)))
        # Finite range noise and the explicit pose prior trade height against
        # pitch. Test improvement, not an unjustified millimetre-level target.
        self.assertLess(max(after),.6*max(before));self.assertGreater(max(before),.07)
        self.assertGreater(result['optimization']['accepted'],0)
        self.assertLess(result['optimization']['final_cost'],result['optimization']['initial_cost']*.01)
        print(json.dumps(dict(scope='synthetic joint pose/plane with explicit pose prior; no IMU or field acceptance',
            maximum_height_error_before_m=max(before),maximum_height_error_after_m=max(after),
            optimization=result['optimization'])))

    def test_native_rejects_duplicate_beams_bad_noise_and_wrong_branch(self):
        for kind in ('duplicate_beam','noise','branch'):
            a=fixture()
            if kind=='duplicate_beam':a['observations'][0]['beam_ids'][1]=0
            elif kind=='noise':a['measurement_covariance'][:]=0
            else:a['observations'][0]['boundary_sign']=1
            with self.subTest(kind=kind),self.assertRaises(RuntimeError):self.native(a)


if __name__=='__main__':unittest.main()
