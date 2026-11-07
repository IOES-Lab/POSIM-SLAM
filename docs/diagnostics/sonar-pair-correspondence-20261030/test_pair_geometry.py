from pathlib import Path
import sys
import unittest
import numpy as np
here=Path(__file__).resolve().parent
for ancestor in here.parents:
    source=next((p/'src' for p in (ancestor,ancestor/'libraries/posim-slam')
                 if (p/'src/posim_slam').is_dir()),None)
    if source is not None:break
else:raise RuntimeError('public_posim_slam_source_required')
sys.path[:0]=[str(source),str(here)]
from posim_slam.inertial import exp_so3
from posim_slam.sonar_factor import point_return_factor
from pair_geometry import fit_pair,evaluate,SIGMA

G=dict(range_min=.4,range_max=15.,horizontal_fov=2*np.pi/3,vertical_fov=np.pi/9)
def polar(points):
    return np.column_stack([np.linalg.norm(points,axis=1),np.arctan2(points[:,1],points[:,0])])

class PairGeometryTests(unittest.TestCase):
    def test_noiseless_translated_views_recover_points_without_a_point_prior(self):
        points=np.array([[3.,-.5,-.15],[4.,.2,.2],[5.,.4,-.3],[6.,-.3,.2]])
        r=exp_so3([.01,-.01,.1]);t=np.array([.2,.05,.1])
        result=fit_pair(polar(points),polar((points-t)@r),r,t,geometry=G,iterations=24)
        best=result['cost'].argmin(axis=1)
        np.testing.assert_allclose(result['points'][np.arange(len(points)),best],points,atol=1e-6)
        self.assertTrue(np.all(result['cost'][np.arange(len(points)),best]<1e-12))

    def test_pure_yaw_retains_the_unresolved_elevation_family(self):
        points=np.array([[5.,.3,0.]])
        r=exp_so3([0,0,.1]);t=np.zeros(3)
        result=fit_pair(polar(points),polar(points@r),r,t,geometry=G)
        self.assertTrue(result['valid'].all() and result['converged'].all())
        self.assertTrue((result['rank']==2).all())
        self.assertLess(result['cost'].max(),1e-20)
        self.assertGreater(np.ptp(result['points'][0,:,2]),1.7)

    def test_impossible_range_correspondence_is_not_explained_by_height(self):
        result=fit_pair([[5.,0]],[[12.,0]],np.eye(3),[.2,0,0],geometry=G,iterations=24)
        self.assertGreater(result['cost'].min(),1000)

    def test_boundary_optimum_uses_feasible_stationarity_without_a_height_prior(self):
        result=fit_pair([[5.,0.]],[[4.5,0.]],np.eye(3),[0.,0.,.2],geometry=G)
        mode=int(result['cost'][0].argmin())
        self.assertTrue(result['converged'][0,mode] and result['constrained_stationary'][0,mode])
        self.assertGreater(result['gradient_inf'][0,mode],1.)
        # Independent scalar minimization at the analytically active +10° fan.
        def cost(radius):
            second=np.sqrt(radius**2-.4*radius*np.sin(np.pi/18)+.04)
            return ((radius-5.)**2+(second-4.5)**2)/SIGMA[0]**2
        low,high=4.,6.
        for _ in range(90):
            a=low+(high-low)/3;b=high-(high-low)/3
            if cost(a)<cost(b):high=b
            else:low=a
        self.assertAlmostEqual(result['cost'][0,mode],cost((low+high)/2),places=5)

    def test_prediction_jacobians_match_independent_finite_differences(self):
        point=np.array([[[5.,.3,.2]]]);r=exp_so3([.01,-.01,.05]);t=np.array([.2,.1,.05])
        measurements=np.stack([polar(point[:,0]),polar((point[:,0]-t)@r)],axis=1)
        residual,jacobian,_,_=evaluate(point,measurements,r,t,G)
        numerical=[]
        for axis in range(3):
            delta=np.eye(3)[axis]*1e-6
            numerical.append((evaluate(point+delta,measurements,r,t,G)[0]-evaluate(point-delta,measurements,r,t,G)[0])/(2e-6))
        np.testing.assert_allclose(jacobian,np.stack(numerical,axis=-1),atol=1e-6,rtol=0)
        factor=point_return_factor(position=t,attitude=r,imu_to_sonar=np.zeros(3),sonar_to_imu=np.eye(3),
            landmark=point[0,0],measured_range=measurements[0,1,0],measured_azimuth=measurements[0,1,1],**G)
        np.testing.assert_allclose(jacobian[0,0,2:],factor['landmark_jacobian']/SIGMA[:,None],atol=1e-10)

if __name__=='__main__':unittest.main()
