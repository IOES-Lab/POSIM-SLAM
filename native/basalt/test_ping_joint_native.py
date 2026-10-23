"""Cross-check a compiled Eigen probe against the independent NumPy reference."""
import os
import subprocess
import unittest
import numpy as np
from posim_slam.ping_joint import eliminate_patch,linearize_ping_patch
from posim_slam.ping_scale import PING_DIRECTIONS


@unittest.skipUnless(os.environ.get('POSIM_SLAM_PING_PROBE'),'compiled native Ping probe required')
class NativeJointTests(unittest.TestCase):
    def compare(self,j,r,retained):
        rows,cols=j.shape
        text=f'{rows} {cols} {retained}\n'+'\n'.join(' '.join(map(str,[*row,value])) for row,value in zip(j,r))+'\n'
        result=subprocess.run([os.environ['POSIM_SLAM_PING_PROBE']],input=text,text=True,
                              capture_output=True,timeout=30,check=True)
        actual=np.fromstring(result.stdout,sep=' ');ref=eliminate_patch(j,r,retained_columns=retained)
        expected=np.r_[ref['nuisance_rank'],ref['nuisance_nullity'],
            (ref['jacobian'].T@ref['jacobian']).ravel(),ref['jacobian'].T@ref['residual'],
            ref['residual']@ref['residual'],ref['backsub_matrix'].ravel(),ref['backsub_offset']]
        np.testing.assert_allclose(actual,expected,rtol=2e-8,atol=2e-9)

    def test_full_rank_and_unobservable_systems(self):
        rng=np.random.default_rng(75)
        for rows,cols,retained in ((32,15,6),(8,15,6),(20,10,3)):
            j=rng.normal(size=(rows,cols));r=rng.normal(size=rows)
            self.compare(j,r,retained)
            j[:,-1]=j[:,-2];self.compare(j,r,retained)
        self.compare(np.zeros((8,15)),np.ones(8),6)

    def test_real_ping_block_with_joint_landmark_rows(self):
        normal=np.array([.1,.2,1.]);normal/=np.linalg.norm(normal)
        xy=np.array([(x,y) for x in np.linspace(-.6,.6,8) for y in np.linspace(-.6,.6,8)])
        landmarks=np.column_stack([xy,(2.-xy@normal[:2])/normal[2]])
        b=linearize_ping_patch(landmarks=landmarks,range_sigma=.02,surface_sigma=.01,
            max_support_distance=.15,position=np.zeros(3),attitude=np.eye(3),
            imu_to_ping=np.array([.02,.03,.1]),ping_to_imu=np.eye(3),rays=PING_DIRECTIONS,
            plane_normal=normal,plane_distance=2.,measured_range=2.,range_bias=.1,range_min=.2,range_max=50.)
        self.compare(b['jacobian'],b['residual'],6)
        size=b['jacobian'].shape[1];prior=np.eye(size)[list(range(6))+list(range(9,size))]*5
        self.compare(np.vstack([prior,b['jacobian']]),np.r_[np.zeros(len(prior)),b['residual']],6)

    def test_bad_dimensions_fail(self):
        result=subprocess.run([os.environ['POSIM_SLAM_PING_PROBE']],input='5000 2 1\n',
                              text=True,capture_output=True,timeout=5)
        self.assertNotEqual(result.returncode,0)


if __name__=='__main__':unittest.main()
