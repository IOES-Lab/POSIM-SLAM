import json
import os
import subprocess
import unittest
import numpy as np
from posim_slam.ping_joint import split_patch_information


@unittest.skipUnless(os.environ.get('POSIM_SLAM_JOINT_INFORMATION_PROBE'),'compiled conditional information probe required')
class JointInformationNativeTests(unittest.TestCase):
    def test_full_rank_rank_deficient_and_endpoint_splits(self):
        rng=np.random.default_rng(918)
        for rows,cols,keep in ((40,18,12),(7,18,12),(12,9,0),(9,9,9),(0,9,6),(3,0,0)):
            for deficient in (False,True):
                j=rng.normal(size=(rows,cols));r=rng.normal(size=rows)
                if deficient and cols-keep>=2:j[:,-1]=j[:,-2]
                source=split_patch_information(j,r,retained_columns=keep)
                payload=dict(jacobian=j.tolist(),residual=r.tolist(),columns=cols,retained=keep)
                p=subprocess.run([os.environ['POSIM_SLAM_JOINT_INFORMATION_PROBE']],input=json.dumps(payload),text=True,capture_output=True,check=True,timeout=15)
                actual=json.loads(p.stdout);rank=source['nuisance_rank'];nuisance=cols-keep
                matrix=lambda name,shape:np.array(actual[name]).reshape(shape)
                a=matrix('jacobian',(rows-rank,keep));b=np.array(actual['residual'])
                c=matrix('conditional_jacobian',(rank,cols));d=np.array(actual['conditional_residual'])
                np.testing.assert_allclose(a.T@a,source['jacobian'].T@source['jacobian'],atol=2e-11)
                np.testing.assert_allclose(a.T@b,source['jacobian'].T@source['residual'],atol=2e-11)
                np.testing.assert_allclose(c.T@c,source['conditional_jacobian'].T@source['conditional_jacobian'],atol=2e-11)
                noise=matrix('noise',(nuisance,rank));free=matrix('unbounded',(nuisance,nuisance-rank))
                np.testing.assert_allclose(noise@noise.T,source['observable_noise_factor']@source['observable_noise_factor'].T,atol=2e-12)
                np.testing.assert_allclose(free@free.T,source['unobservable_basis']@source['unobservable_basis'].T,atol=2e-12)
                np.testing.assert_allclose(matrix('regression',(nuisance,keep)),source['backsub_matrix'],atol=2e-11)
                self.assertEqual(actual['nullity'],source['nuisance_nullity'])


if __name__=='__main__':unittest.main()
