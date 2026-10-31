# SPDX-License-Identifier: Apache-2.0
"""Known geometry, rank, frame and full correlated-error controls."""
import unittest
import numpy as np
from posim_slam.range_direction import calibrate_return_directions


class DirectionCalibration(unittest.TestCase):
    def fixture(self):
        n=np.array([[0.,0.,-1.],[.2,0.,-1.],[0.,.2,-1.],[-.15,.1,-1.],[.08,-.12,-1.]])
        n/=np.linalg.norm(n,axis=1)[:,None]
        rays=np.array([[-.35,.35,-.87],[.35,.35,-.87]])
        rays/=np.linalg.norm(rays,axis=1)[:,None]
        h=np.array([.75,1.3,2.,3.,4.])
        return n,h,h[:,None]/(n@rays.T),rays

    def test_known_geometry_and_unknown_uncertainty(self):
        n,h,r,u=self.fixture();out=calibrate_return_directions(plane_normals=n,plane_distances_m=h,measured_ranges_m=r)
        np.testing.assert_allclose(out['rays'],u,atol=2e-15)
        self.assertTrue(out['available']);self.assertFalse(out['covariance_available'])
        self.assertFalse(out['physical_branch_verified']);self.assertFalse(out['map_qualified'])
        self.assertNotIn('joint_ray_covariance',out)

    def test_repeated_plane_cannot_supply_missing_excitation(self):
        n,h,r,u=self.fixture();n[:]=n[0];r=h[:,None]/(n@u.T)
        out=calibrate_return_directions(plane_normals=n,plane_distances_m=h,measured_ranges_m=r)
        self.assertFalse(out['available']);self.assertEqual(out['normal_rank'],1)

    def test_full_cross_beam_covariance_and_derivatives(self):
        n,h,r,u=self.fixture();rng=np.random.default_rng(7)
        m=len(n)*len(u);a=rng.normal(size=(m,m));cov=(a@a.T+np.eye(m))*1e-8
        args=dict(plane_normals=n,plane_distances_m=h,measured_ranges_m=r,
            inverse_measurement_covariance=cov,uncertainty_source='explicit correlated synthetic control')
        out=calibrate_return_directions(**args)
        self.assertGreater(np.linalg.norm(out['joint_ray_covariance'][:3,3:]),1e-10)
        y=(h[:,None]/r).T.reshape(-1);columns=[];eps=1e-7
        for i in range(m):
            delta=np.eye(m)[i]*eps
            plus=h[:,None]/(y+delta).reshape(len(u),len(n)).T
            minus=h[:,None]/(y-delta).reshape(len(u),len(n)).T
            x=calibrate_return_directions(**dict(args,measured_ranges_m=plus))['rays'].reshape(-1)
            z=calibrate_return_directions(**dict(args,measured_ranges_m=minus))['rays'].reshape(-1)
            columns.append((x-z)/(2*eps))
        np.testing.assert_allclose(np.array(columns).T,out['inverse_measurement_jacobian'],atol=2e-8)
        for i,ray in enumerate(out['rays']):
            np.testing.assert_allclose(ray@out['joint_ray_covariance'][3*i:3*i+3,:],0,atol=2e-18)
        # A global change of calibration coordinates rotates every mean/cross block.
        q,_=np.linalg.qr(rng.normal(size=(3,3)))
        if np.linalg.det(q)<0:q[:,0]*=-1
        moved=calibrate_return_directions(**dict(args,plane_normals=n@q.T))
        np.testing.assert_allclose(moved['rays'],out['rays']@q.T,atol=2e-15)
        transform=np.kron(np.eye(len(u)),q)
        np.testing.assert_allclose(moved['joint_ray_covariance'],transform@out['joint_ray_covariance']@transform.T,atol=2e-18)

    def test_invalid_inputs_and_inconsistent_model_are_explicit(self):
        n,h,r,u=self.fixture();args=dict(plane_normals=n,plane_distances_m=h,measured_ranges_m=r)
        for edit in [dict(plane_distances_m=-h),dict(measured_ranges_m=r*float('nan')),
                     dict(inverse_measurement_covariance=-np.eye(r.size),uncertainty_source='invalid'),
                     dict(inverse_measurement_covariance=np.eye(r.size))]:
            with self.assertRaises(ValueError):calibrate_return_directions(**dict(args,**edit))
        out=calibrate_return_directions(**dict(args,measured_ranges_m=r/2))
        self.assertGreater(np.linalg.norm(out['inverse_range_residuals']),.1)
        self.assertFalse(out['physical_branch_verified'])


if __name__=='__main__':unittest.main()
