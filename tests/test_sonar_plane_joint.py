import copy
import unittest
import numpy as np
from posim_slam.inertial import exp_so3
from posim_slam.ground_surface_return import plane_returns
from posim_slam.sonar_plane_joint import linearize_plane_patch,marginal_plane_information
from posim_slam.ping_joint import eliminate_patch


def fixture():
    n=np.array([1.,.12,.08]);n/=np.linalg.norm(n)
    lever=np.array([.12,-.05,.07]);mount=exp_so3([.02,-.04,.03])
    az=np.array([-.3,-.1,.13,.35]);elev=np.array([-.12,.03,.17])
    rays=np.array([np.column_stack((np.cos(elev)*np.cos(a),np.cos(elev)*np.sin(a),np.sin(elev))) for a in az])
    states={i+1:dict(position=np.array([i*.12,i*i*.03,.01*i]),attitude=exp_so3([.01*i,.02*i,-.015*i])) for i in range(3)}
    obs=[]
    for stamp,s in states.items():
        pred=plane_returns(s['position']+s['attitude']@lever,rays@mount.T@s['attitude'].T,n,4.,range_min=.1,range_max=10.)
        obs.append(dict(stamp_ns=stamp,source_ids=list(range(4)),rays=rays.copy(),range_m=pred['range_m']))
    return dict(observations=obs,states=states,plane_normal=n,plane_distance=4.,
                imu_to_sonar=lever,sonar_to_imu=mount,range_min=.1,range_max=10.,
                measurement_covariance=np.eye(12)*.025**2+np.ones((12,12))*.01**2)


class PlaneJointTests(unittest.TestCase):
    def test_full_pose_and_normal_derivatives_with_rotated_lever(self):
        a=fixture();base=linearize_plane_patch(**a);eps=1e-6;cols=[]
        for k in range(21):
            values=[]
            for sign in (-1,1):
                b=copy.deepcopy(a)
                if k<18:
                    state=b['states'][k//6+1];delta=np.eye(3)[k%3]*sign*eps
                    if k%6<3:state['position']+=delta
                    else:state['attitude']=state['attitude']@exp_so3(delta)
                elif k<20:
                    n=a['plane_normal']+base['normal_basis'][:,k-18]*sign*eps;b['plane_normal']=n/np.linalg.norm(n)
                else:b['plane_distance']+=sign*eps
                values.append(linearize_plane_patch(**b)['residual'])
            cols.append((values[1]-values[0])/(2*eps))
        np.testing.assert_allclose(np.array(cols).T,base['jacobian'],atol=3e-7)

    def test_unknown_plane_does_not_create_absolute_pose_information(self):
        a=fixture();f=linearize_plane_patch(**a)
        reduced=eliminate_patch(f['jacobian'],f['residual'],retained_columns=18)['jacobian']
        for axis in np.eye(3):
            shift=np.tile(np.r_[axis,np.zeros(3)],3)
            turn=np.concatenate([np.r_[np.cross(axis,s['position']),s['attitude'].T@axis] for s in a['states'].values()])
            np.testing.assert_allclose(reduced@shift,0.,atol=1e-10)
            np.testing.assert_allclose(reduced@turn,0.,atol=1e-10)

    def test_full_correlated_pose_marginal_matches_joint_inverse(self):
        f=linearize_plane_patch(**fixture());j=f['jacobian'];rng=np.random.default_rng(52)
        a=rng.normal(size=(18,18))*.002;p=a@a.T+np.eye(18)*.0001
        result=marginal_plane_information(j,p)
        full=j.T@j;full[:18,:18]+=np.linalg.inv(p)
        np.testing.assert_allclose(result['plane_covariance'],np.linalg.inv(full)[-3:,-3:],rtol=1e-8,atol=1e-10)
        fixed=marginal_plane_information(j,np.zeros((18,18)))
        self.assertGreaterEqual(np.linalg.eigvalsh(result['plane_covariance']-fixed['plane_covariance']).min(),-1e-10)
        diagonal=marginal_plane_information(j,np.diag(np.diag(p)))
        self.assertGreater(np.linalg.norm(result['plane_covariance']-diagonal['plane_covariance']),1e-6)

    def test_support_rejects_extrapolation_and_sparse_interior_holes(self):
        a=fixture();f=linearize_plane_patch(**a);n=a['plane_normal'];basis=f['normal_basis'];center=f['intersections'].mean(axis=0)
        a['support_points']=np.array([center+basis@v for v in [(-2,-2),(2,-2),(2,2),(-2,2)]])
        a['max_support_distance']=4.;linearize_plane_patch(**a)
        with self.assertRaisesRegex(ValueError,'support_gap'):linearize_plane_patch(**dict(a,max_support_distance=.05))
        shifted=a['support_points']+basis[:,0]*10
        with self.assertRaisesRegex(ValueError,'does_not_cover'):linearize_plane_patch(**dict(a,support_points=shifted))

    def test_rank_deficient_plane_does_not_get_finite_covariance(self):
        j=np.zeros((3,9));j[:,-1]=1.
        result=marginal_plane_information(j,np.eye(6))
        self.assertFalse(result['full_rank']);self.assertIsNone(result['plane_covariance'])

    def test_rejects_identity_ties_and_invalid_covariance(self):
        for kind in ['identity','tie','time','covariance','support']:
            a=fixture()
            if kind=='identity':a['observations'][0]['source_ids'][1]=0
            elif kind=='tie':
                for o in a['observations']:o['rays']=np.repeat(o['rays'],2,axis=1)
            elif kind=='time':a['observations'][1]['stamp_ns']=1
            elif kind=='covariance':a['measurement_covariance'][:]=0
            else:a['max_support_distance']=1.
            with self.subTest(kind=kind),self.assertRaises(ValueError):linearize_plane_patch(**a)
        j=linearize_plane_patch(**fixture())['jacobian'];p=np.eye(18);p[0,0]=-1
        with self.assertRaises(ValueError):marginal_plane_information(j,p)

if __name__=='__main__':unittest.main()
