# SPDX-License-Identifier: Apache-2.0
"""Independent geometric and correlated-uncertainty controls."""
import math
import unittest
import numpy as np
from shadow_geometry import shadow_edge,plane_ray


def skew(v):
    x,y,z=v
    return np.array([[0.,-z,y],[z,0.,-x],[-y,x,0.]])


def exp(v):
    angle=np.linalg.norm(v)
    if angle==0:return np.eye(3)
    k=skew(v/angle)
    return np.eye(3)+math.sin(angle)*k+(1-math.cos(angle))*(k@k)


class ShadowGeometry(unittest.TestCase):
    def geometry(self):
        # A tilted sensor observing an independently constructed edge and floor.
        rotation=exp(np.array([.12,.75,.08]))
        n=rotation.T@np.array([0.,0.,-1.])
        edge=rotation.T@np.array([2.2,.35,-1.75])
        h=.45;altitude=2.2
        end=edge*altitude/(altitude-h)
        return dict(edge_range_m=np.linalg.norm(edge),shadow_end_range_m=np.linalg.norm(end),
                    azimuth_rad=math.atan2(edge[1],edge[0]),plane_normal=n,
                    plane_distance_m=altitude,vertical_fov_rad=math.radians(30)),edge,end,h

    def test_forward_occlusion_geometry(self):
        kw,edge,end,height=self.geometry()
        result=shadow_edge(**kw)
        self.assertTrue(result['available'])
        np.testing.assert_allclose(result['point'],edge,atol=2e-14)
        np.testing.assert_allclose(result['shadow_endpoint'],end,atol=2e-14)
        self.assertAlmostEqual(result['height_m'],height,places=13)
        self.assertFalse(result['covariance_available'])
        self.assertFalse(result['map_qualified'])

    def test_full_jacobian_and_global_rotation(self):
        kw,_,_,_=self.geometry();base=shadow_edge(**kw)
        eps=1e-6
        names=['edge_range_m','shadow_end_range_m','azimuth_rad','plane_distance_m']
        columns=[];heights=[]
        for index in range(7):
            plus=dict(kw);minus=dict(kw)
            if index<4:
                plus[names[index]]+=eps;minus[names[index]]-=eps
            else:
                delta=np.eye(3)[index-4]*eps
                plus['plane_normal']=exp(delta)@kw['plane_normal']
                minus['plane_normal']=exp(-delta)@kw['plane_normal']
            a=shadow_edge(**plus);b=shadow_edge(**minus)
            columns.append((a['point']-b['point'])/(2*eps))
            heights.append((a['height_m']-b['height_m'])/(2*eps))
        np.testing.assert_allclose(np.array(columns).T,base['point_jacobian'],atol=2e-8)
        np.testing.assert_allclose(heights,base['height_jacobian'],atol=2e-9)
        # Rotation about the plane normal leaves that same plane unchanged.
        np.testing.assert_allclose(base['point_jacobian'][:,4:]@kw['plane_normal'],0.,atol=1e-15)

    def test_shared_correlated_errors(self):
        kw,_,_,_=self.geometry();rng=np.random.default_rng(14)
        a=rng.normal(size=(7,7));cov=a@a.T*1e-8
        result=shadow_edge(**kw,measurement_covariance=cov)
        draws=rng.multivariate_normal(np.zeros(7),cov,size=100000)
        empirical=np.cov((draws@result['point_jacobian'].T).T)
        np.testing.assert_allclose(empirical,result['point_covariance'],rtol=.025,atol=2e-9)
        independent=shadow_edge(**kw,measurement_covariance=np.diag(np.diag(cov)))
        self.assertGreater(np.max(abs(independent['point_covariance']-result['point_covariance'])),1e-9)

    def test_two_branches_are_unresolved(self):
        result=shadow_edge(edge_range_m=1.,shadow_end_range_m=2.,azimuth_rad=0.,
                 plane_normal=[1.,0.,0.],plane_distance_m=1.98,vertical_fov_rad=.6)
        self.assertFalse(result['available']);self.assertEqual(result['reason'],'ambiguous_ground_intersection')

    def test_missing_or_tangent_intersection(self):
        common=dict(edge_range_m=1.,shadow_end_range_m=2.,azimuth_rad=0.,
                    plane_normal=[1.,0.,0.],vertical_fov_rad=.4)
        self.assertEqual(shadow_edge(**common,plane_distance_m=2.1)['reason'],'no_ground_intersection')
        self.assertEqual(shadow_edge(**common,plane_distance_m=2.)['reason'],'tangent_ground_intersection')

    def test_reject_invalid_syntax_without_inventing_uncertainty(self):
        kw,_,_,_=self.geometry()
        for edit in [dict(edge_range_m=0.),dict(shadow_end_range_m=kw['edge_range_m']),
                     dict(plane_normal=[0,0,-2]),dict(plane_distance_m=float('nan')),
                     dict(measurement_covariance=-np.eye(7)),dict(measurement_covariance=-np.eye(7)*1e-14),
                     dict(measurement_covariance=np.eye(6))]:
            with self.assertRaises(ValueError):shadow_edge(**dict(kw,**edit))


if __name__=='__main__':
    unittest.main()
