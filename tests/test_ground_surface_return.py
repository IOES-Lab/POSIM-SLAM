import unittest
import numpy as np
from posim_slam.ground_surface_return import surface_returns,plane_returns


def fan(azimuth,elevation):
    a,e=np.meshgrid(azimuth,elevation,indexing='ij')
    return np.stack((np.cos(a)*np.cos(e),np.sin(a)*np.cos(e),np.sin(e)),axis=-1)


class GroundSurfaceReturnTests(unittest.TestCase):
    def test_vertical_face_selects_interior_ray_and_is_sign_invariant(self):
        rays=fan([-.2,0.,.3],[-.2,0.,.2]);kwargs=dict(range_min=.1,range_max=10.)
        result=plane_returns([0,0,0],rays,[1,0,0],4.,**kwargs)
        np.testing.assert_allclose(result['range_m'],4/np.cos([-.2,0.,.3]))
        np.testing.assert_array_equal(result['ray_indices'],[1,1,1])
        reverse=plane_returns([0,0,0],rays,[-1,0,0],-4.,**kwargs)
        np.testing.assert_allclose(result['points'],reverse['points'])
        np.testing.assert_allclose(result['points'][:,0],4.)

    def test_general_plane_matches_height_plane_and_derivatives(self):
        rays=fan([-.2,.1,.35],[-.7,-.5,-.3]);o=np.array([.2,-.1,.15]);c=np.array([-2.,.15,-.1])
        normal=np.array([-c[1],-c[2],1.]);scale=np.linalg.norm(normal);normal/=scale;distance=c[0]/scale
        kwargs=dict(range_min=.1,range_max=10.);result=plane_returns(o,rays,normal,distance,**kwargs)
        np.testing.assert_allclose(result['range_m'],surface_returns(o,rays,c,**kwargs)['range_m'])
        for i in range(3):
            step=np.eye(3)[i]*1e-6
            difference=(plane_returns(o+step,rays,normal,distance,**kwargs)['range_m']-plane_returns(o-step,rays,normal,distance,**kwargs)['range_m'])/2e-6
            np.testing.assert_allclose(difference,result['origin_jacobian'][:,i],atol=1e-8)
        tangent=np.cross(normal,[0,1,0]);epsilon=1e-6
        plus=normal+epsilon*tangent;plus/=np.linalg.norm(plus)
        minus=normal-epsilon*tangent;minus/=np.linalg.norm(minus)
        difference=(plane_returns(o,rays,plus,distance,**kwargs)['range_m']-plane_returns(o,rays,minus,distance,**kwargs)['range_m'])/2e-6
        np.testing.assert_allclose(difference,result['normal_jacobian']@tangent,atol=1e-8)

    def test_plane_parallel_behind_and_blind_returns_invalid(self):
        rays=fan([0.],[0.]);kwargs=dict(range_min=.2,range_max=10.)
        for normal,distance in (([0,0,1],2),([1,0,0],-2),([1,0,0],.1)):
            result=plane_returns([0,0,0],rays,normal,distance,**kwargs)
            self.assertFalse(result['valid'][0]);self.assertTrue(np.isnan(result['points']).all())

    def test_flat_ground_uses_closest_authored_ray(self):
        rays=fan([-.3,0.,.3],[-.6,-.4,-.2])
        r=surface_returns([0,0,0],rays,[-2,0,0],range_min=.1,range_max=10.)
        np.testing.assert_allclose(r['range_m'],2/np.sin(.6))
        np.testing.assert_array_equal(r['ray_indices'],[0,0,0])
        np.testing.assert_allclose(r['points'][:,2],-2.)
        self.assertFalse(r['map_qualified'])

    def test_curvature_can_make_first_return_inside_vertical_fan(self):
        elevation=np.linspace(-1.1,-.3,256);rays=fan([0.],elevation)
        r=surface_returns([0,0,0],rays,[-2,1,0,.1,0,0],range_min=.1,range_max=10.)
        self.assertTrue(r['valid'][0]);self.assertTrue(0<r['ray_indices'][0]<255)
        # Independent dense surface parameterization, not the ray-root formula.
        x=np.linspace(0,4,400001);z=-2+x+.1*x*x;a=np.arctan2(z,x)
        valid=(a>=elevation[0])&(a<=elevation[-1]);distance=np.hypot(x,z)
        self.assertAlmostEqual(r['range_m'][0],distance[valid].min(),delta=2e-5)

    def test_near_occluder_is_not_replaced_by_second_root(self):
        ray=fan([0.],[0.]);c=[8,-6,0,1,0,0]  # roots at x=2 and x=4
        r=surface_returns([0,0,0],ray,c,range_min=.1,range_max=10.)
        self.assertAlmostEqual(r['range_m'][0],2.)
        invalid=surface_returns([0,0,0],ray,c,range_min=3.,range_max=10.)
        self.assertFalse(invalid['valid'][0]);self.assertTrue(np.isnan(invalid['range_m'][0]))

    def test_origin_and_coefficient_jacobians_match_finite_difference(self):
        rays=fan([-.2,.1,.35],[-.7,-.5,-.3]);o=np.array([.2,-.1,.15]);c=np.array([-2,.15,-.1,.04,.01,.02])
        kwargs=dict(range_min=.1,range_max=10.)
        result=surface_returns(o,rays,c,**kwargs);self.assertTrue(result['valid'].all())
        for source,key in ((o,'origin_jacobian'),(c,'coefficient_jacobian')):
            numerical=[]
            for i in range(len(source)):
                step=np.eye(len(source))[i]*1e-6
                if len(source)==3:
                    plus=surface_returns(o+step,rays,c,**kwargs);minus=surface_returns(o-step,rays,c,**kwargs)
                else:
                    plus=surface_returns(o,rays,c+step,**kwargs);minus=surface_returns(o,rays,c-step,**kwargs)
                np.testing.assert_array_equal(plus['ray_indices'],minus['ray_indices'])
                numerical.append((plus['range_m']-minus['range_m'])/2e-6)
            np.testing.assert_allclose(result[key],np.array(numerical).T,atol=1e-8)

    def test_direction_jacobian_on_unit_ray_rotation(self):
        rays=fan([-.2,.1,.35],[-.7,-.5,-.3]);c=[-2,.15,-.1,.04,.01,.02]
        kwargs=dict(range_min=.1,range_max=10.)
        result=surface_returns([.2,-.1,.15],rays,c,**kwargs);epsilon=1e-6
        rotation=np.array([[np.cos(epsilon),-np.sin(epsilon),0],
                           [np.sin(epsilon),np.cos(epsilon),0],[0,0,1.]])
        plus=surface_returns([.2,-.1,.15],rays@rotation.T,c,**kwargs)
        minus=surface_returns([.2,-.1,.15],rays@rotation,c,**kwargs)
        selected=rays[np.arange(len(rays)),result['ray_indices']]
        differential=np.cross([0,0,1.],selected)
        analytic=np.sum(result['direction_jacobian']*differential,axis=1)
        np.testing.assert_allclose(analytic,(plus['range_m']-minus['range_m'])/(2*epsilon),atol=1e-8)

    def test_tangent_missing_and_invalid_geometry_do_not_emit_observations(self):
        ray=fan([0.],[0.])
        for coefficients in ([4,-4,0,1,0,0],[-2,0,0]):
            result=surface_returns([0,0,0],ray,coefficients,range_min=.1,range_max=10.)
            self.assertFalse(result['valid'][0]);self.assertTrue(np.isnan(result['coefficient_jacobian']).all())
        with self.assertRaises(ValueError):surface_returns([0,0,0],ray*2,[-2,0,0],range_min=.1,range_max=10.)
        with self.assertRaises(ValueError):surface_returns([0,0,0],np.tile(ray,(129,1,1)),[-2,0,0],range_min=.1,range_max=10.)


if __name__=='__main__':unittest.main()
