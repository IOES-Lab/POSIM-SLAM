"""Real Basalt multi-surface elimination, trial updates and retirement controls."""
import copy,json,os,subprocess,unittest
import numpy as np
from posim_slam.ground_surface_return import plane_returns
from posim_slam.sonar_plane_joint import linearize_plane_patch
from posim_slam.inertial import skew,exp_so3
from test_sonar_plane_joint import fixture


def multi_fixture():
    a=fixture();second=np.array([1.,-.25,.18]);second/=np.linalg.norm(second)
    planes=[dict(id=0,normal=a['plane_normal'],distance=4.),dict(id=1,normal=second,distance=5.)]
    observations=[]
    for o in a['observations']:
        s=a['states'][o['stamp_ns']];origin=s['position']+s['attitude']@a['imu_to_sonar'];returns=[]
        for plane in planes:
            prediction=plane_returns(origin,o['rays']@a['sonar_to_imu'].T@s['attitude'].T,plane['normal'],plane['distance'],range_min=.1,range_max=10.)
            returns.extend(dict(source_id=4*plane['id']+k,plane_id=plane['id'],range_m=float(prediction['range_m'][k]),rays=o['rays'][k]) for k in range(4))
        observations.append(dict(stamp_ns=o['stamp_ns'],returns=returns))
    return {k:v for k,v in a.items() if k not in ('observations','plane_normal','plane_distance','measurement_covariance')}|dict(
        observations=observations,planes=planes,measurement_covariance=np.eye(24)*.025**2+np.ones((24,24))*.01**2)


def python_rows(a):
    j=np.zeros((24,24));residual=np.zeros(24)
    for plane in a['planes']:
        observations=[];indices=[]
        for frame in a['observations']:
            entries=[(i,row) for i,row in enumerate(frame['returns']) if row['plane_id']==plane['id']]
            observations.append(dict(stamp_ns=frame['stamp_ns'],source_ids=[r['source_id'] for _,r in entries],
                range_m=[r['range_m'] for _,r in entries],rays=np.array([r['rays'] for _,r in entries])))
            indices.extend((frame['stamp_ns']-1)*8+i for i,_ in entries)
        kw={k:v for k,v in a.items() if k not in ('observations','planes','measurement_covariance','prior_sqrt','check_derivatives','check_install')}
        f=linearize_plane_patch(**kw,observations=observations,plane_normal=plane['normal'],plane_distance=plane['distance'],measurement_covariance=np.eye(12))
        local=f['jacobian'].copy()
        for stamp,state in a['states'].items():local[:,6*(stamp-1)+3:6*stamp]=local[:,6*(stamp-1)+3:6*stamp]@state['attitude'].T
        basis=f['normal_basis'];conversion=basis.T@(-skew(plane['normal']))@basis
        j[np.ix_(indices,range(18))]=local[:,:18]
        cols=[18+3*plane['id'],19+3*plane['id']]
        j[np.ix_(indices,cols)]=local[:,-3:-1]@conversion
        j[indices,20+3*plane['id']]=local[:,-1];residual[indices]=f['residual']
    lower=np.linalg.cholesky(a['measurement_covariance']);return np.linalg.solve(lower,j),np.linalg.solve(lower,residual)


@unittest.skipUnless(os.environ.get('POSIM_SLAM_GROUND_PROBE'),'compiled native probe required')
class MultiPlaneNativeTests(unittest.TestCase):
    def native(self,a):
        command=[os.environ['POSIM_SLAM_GROUND_PROBE']]
        result=subprocess.run(command,input=json.dumps(a,default=lambda v:v.tolist()),text=True,capture_output=True,timeout=30)
        if result.returncode:raise ValueError(result.stderr)
        return json.loads(result.stdout)

    def test_shared_frame_cross_surface_covariance_matches_python(self):
        a=multi_fixture();a['check_derivatives']=True;r=self.native(a);j,e=python_rows(a)
        np.testing.assert_allclose(r['jacobian'],j,atol=1e-10)
        np.testing.assert_allclose(np.array(r['residual']).ravel(),e,atol=1e-10)
        self.assertLess(r['maximum_derivative_error'],2e-6)
        self.assertEqual(r['nuisance_rank'],6)
        projection=np.eye(24)-j[:,18:]@np.linalg.pinv(j[:,18:])
        np.testing.assert_allclose(r['reduced_H'],j[:,:18].T@projection@j[:,:18],atol=1e-8)

    def test_retirement_keeps_both_planes_and_full_correlations(self):
        a=multi_fixture();a['prior_sqrt']=np.eye(18)*20;a['check_install']=True;r=self.native(a);j,_=python_rows(a)
        h=j.T@j;h[:18,:18]+=np.eye(18)*400
        keep=list(range(6,18));remove=list(range(6))+list(range(18,24))
        expected=h[np.ix_(keep,keep)]-h[np.ix_(keep,remove)]@np.linalg.solve(h[np.ix_(remove,remove)],h[np.ix_(remove,keep)])
        np.testing.assert_allclose(r['marginal_H'],expected,atol=1e-7)
        self.assertEqual(r['retired_planes'],[0,1]);self.assertEqual(r['retired_stamps'],[1,2,3])
        self.assertEqual(r['installation'],dict(remaining_planes=0,remaining_groups=0,remaining_poses=2,replay_rejected=True))

    def test_accumulated_pose_and_plane_coordinates_remain_consistent(self):
        a=multi_fixture();a['check_derivatives']=True
        for s in a['states'].values():s['delta']=[.01,-.02,.005,.03,-.025,.015]
        for p in a['planes']:p['delta']=[.025,-.02,.03]
        r=self.native(a);self.assertLess(r['maximum_derivative_error'],2e-6)

    def test_optimizer_updates_each_plane_through_existing_trial_path(self):
        a=multi_fixture();a['prior_sqrt']=np.eye(18)*100
        for p in a['planes']:p['distance']+=.08
        r=self.native(a);self.assertGreater(r['optimization']['accepted'],0)
        self.assertLess(r['optimization']['final_cost'],r['optimization']['initial_cost']*.01)
        for p in a['planes']:self.assertLess(abs(r['planes'][str(p['id'])]['distance']-(p['distance']-.08)),.005)

    def test_duplicate_source_and_tied_rays_rejected(self):
        a=multi_fixture();a['observations'][0]['returns'][4]['source_id']=0
        with self.assertRaisesRegex(ValueError,'invalid_sonar_plane_return'):self.native(a)
        a=multi_fixture();a['observations'][1]['returns'][0]['plane_id']=1
        with self.assertRaisesRegex(ValueError,'source_plane_changed'):self.native(a)
        a=multi_fixture()
        for frame in a['observations']:
            for r in frame['returns']:r['rays']=np.repeat(r['rays'],2,axis=0)
        with self.assertRaisesRegex(ValueError,'branch_ambiguous'):self.native(a)

if __name__=='__main__':unittest.main()
