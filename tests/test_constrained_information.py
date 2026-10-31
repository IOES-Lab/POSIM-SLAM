"""Exact affine support against dense KKT controls, never large weights."""
import unittest
import numpy as np
from posim_slam.constrained_information import constraint_support,split_constrained_information


def gaussian(j,b,c,d):
    n=j.shape[1];h=j.T@j;g=j.T@b
    kkt=np.block([[h,c.T],[c,np.zeros((len(c),len(c)))]])
    mode=np.linalg.solve(kkt,np.r_[-g,d])[:n]
    covariance=np.linalg.inv(kkt)[:n,:n]
    return mode,covariance


class ConstrainedInformationTests(unittest.TestCase):
    def test_mixed_calibration_constraints_match_dense_mode_and_cross_covariance(self):
        rng=np.random.default_rng(982)
        for kept in (0,6,10,16):
            j=rng.normal(size=(35,16));b=rng.normal(size=35)
            c=rng.normal(size=(4,16));d=rng.normal(size=4)
            if kept in (6,10):c[0,kept:]=0  # A surviving root equality.
            result=split_constrained_information(j,b,retained_columns=kept,constraint_jacobian=c,constraint_target=d)
            root_mode,root_cov=gaussian(result['jacobian'],result['residual'],result['constraint_jacobian'],result['constraint_target'])
            m=result['backsub_matrix'];q=result['backsub_offset'];l=result['observable_noise_factor']
            mode=np.r_[root_mode,m@root_mode+q]
            cov=np.block([[root_cov,root_cov@m.T],[m@root_cov,m@root_cov@m.T+l@l.T]])
            expected_mean,expected_cov=gaussian(j,b,c,d)
            np.testing.assert_allclose(mode,expected_mean,atol=2e-12)
            np.testing.assert_allclose(cov,expected_cov,atol=2e-12)
            np.testing.assert_allclose(c@mode,d,atol=2e-12)
            np.testing.assert_allclose(c@cov,0,atol=2e-12)

    def test_bias_rows_keep_deterministic_and_unbounded_modes_distinct(self):
        j=np.zeros((3,12));j[0,0]=1;j[1,7]=2;j[2,9]=3
        c=np.zeros((2,12));c[0,6]=1;c[1,8]=1
        out=split_constrained_information(j,np.zeros(3),retained_columns=6,constraint_jacobian=c,constraint_target=[.2,-.3])
        self.assertEqual(out['deterministic_basis'].shape,(6,2))
        self.assertEqual(out['unobservable_basis'].shape,(6,2))
        np.testing.assert_allclose(c[:,6:]@out['observable_noise_factor'],0,atol=1e-14)
        np.testing.assert_allclose(c[:,6:]@out['unobservable_basis'],0,atol=1e-14)
        np.testing.assert_allclose(c[:,6:]@out['backsub_offset'],[.2,-.3],atol=1e-14)

    def test_cost_and_conditional_density_preserve_residual_constants_on_support(self):
        rng=np.random.default_rng(983);j=rng.normal(size=(30,11));b=rng.normal(size=30)
        c=np.zeros((3,11));c[:,5:8]=np.eye(3);d=np.array([.1,.2,-.4])
        out=split_constrained_information(j,b,retained_columns=5,constraint_jacobian=c,constraint_target=d)
        for _ in range(7):
            x=rng.normal(size=5);noise=rng.normal(size=3)
            gone=out['backsub_matrix']@x+out['backsub_offset']+out['observable_noise_factor']@noise
            np.testing.assert_allclose(c@np.r_[x,gone],d,atol=1e-13)
            np.testing.assert_allclose(np.linalg.norm(j@np.r_[x,gone]+b)**2,
                np.linalg.norm(out['jacobian']@x+out['residual'])**2+noise@noise,atol=1e-10)

    def test_empty_soft_system_and_exactly_known_all_coordinates(self):
        out=split_constrained_information(np.zeros((0,4)),[],retained_columns=2,
            constraint_jacobian=np.eye(4),constraint_target=np.arange(4))
        self.assertEqual(out['observable_noise_factor'].shape,(2,0))
        self.assertEqual(out['unobservable_basis'].shape,(2,0))
        np.testing.assert_allclose(out['constraint_jacobian']@np.arange(2),out['constraint_target'])
        np.testing.assert_allclose(out['backsub_offset'],[2,3])

    def test_inconsistent_dependent_constraints_and_nonfinite_inputs_reject(self):
        for c,d in (([[1,0],[1,0]],[0,1]),([[0,0]],[.1]),([[float('nan'),0]],[0])):
            with self.assertRaises(ValueError):constraint_support(c,d,2)


if __name__=='__main__':unittest.main()
