import unittest
import numpy as np
from posim_slam.ping_joint import split_patch_information,eliminate_patch


class JointInformationSplitTests(unittest.TestCase):
    def test_pointwise_norm_and_conditional_geometry_for_observed_and_free_directions(self):
        rng=np.random.default_rng(912)
        for rows,cols,keep in ((40,18,12),(7,18,12),(12,9,0),(9,9,9),(0,9,6),(3,0,0)):
            for deficient in (False,True):
                j=rng.normal(size=(rows,cols));r=rng.normal(size=rows)
                if deficient and cols-keep>=2:j[:,-1]=j[:,-2]
                out=split_patch_information(j,r,retained_columns=keep)
                for _ in range(3):
                    x=rng.normal(size=cols)
                    original=np.linalg.norm(j@x+r)**2
                    split=np.linalg.norm(out['jacobian']@x[:keep]+out['residual'])**2
                    split+=np.linalg.norm(out['conditional_jacobian']@x+out['conditional_residual'])**2
                    self.assertAlmostEqual(original,split,places=9)
                rank=out['nuisance_rank'];b=out['conditional_jacobian'][:,keep:]
                np.testing.assert_allclose(b@out['observable_noise_factor'],np.eye(rank),atol=2e-12)
                np.testing.assert_allclose(b@out['unobservable_basis'],0,atol=2e-12)
                mode=out['backsub_offset']+out['backsub_matrix']@np.ones(keep)
                np.testing.assert_allclose(out['conditional_jacobian']@np.r_[np.ones(keep),mode]+out['conditional_residual'],0,atol=2e-12)

    def test_shared_conditional_noise_matches_dense_conditional_covariance(self):
        rng=np.random.default_rng(118);j=rng.normal(size=(50,24));r=rng.normal(size=50)
        out=split_patch_information(j,r,retained_columns=12);noise=out['observable_noise_factor']
        direct=np.linalg.inv(j[:,12:].T@j[:,12:])
        np.testing.assert_allclose(noise@noise.T,direct,atol=2e-14)
        self.assertGreater(np.linalg.norm(direct[:6,6:]),.01)
        # Treating the two old poses independently loses real cross terms.
        independent=np.zeros((12,12));independent[:6,:6]=direct[:6,:6];independent[6:,6:]=direct[6:,6:]
        self.assertGreater(np.linalg.norm(direct-independent),.01)

    def test_free_coordinates_never_receive_finite_zero_variance(self):
        out=split_patch_information(np.zeros((2,12)),np.array([1.,2.]),retained_columns=6)
        self.assertEqual(out['nuisance_nullity'],6)
        np.testing.assert_allclose(out['unobservable_basis']@out['unobservable_basis'].T,np.eye(6),atol=1e-14)
        self.assertEqual(out['observable_noise_factor'].shape,(6,0))

    def test_legacy_contract_and_bad_inputs_remain_explicit(self):
        with self.assertRaises(ValueError):eliminate_patch(np.eye(3),np.zeros(3),retained_columns=3)
        for keep in (-1,4,True):
            with self.assertRaises(ValueError):split_patch_information(np.eye(3),np.zeros(3),retained_columns=keep)
        with self.assertRaises(ValueError):split_patch_information([[float('nan')]],np.zeros(1),retained_columns=0)


if __name__=='__main__':unittest.main()
