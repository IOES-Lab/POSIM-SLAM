import unittest
import numpy as np
from posim_slam.inertial import ImuWindow,exp_so3
from posim_slam.inertial_factor import pair_residual


def state(t,p=(0,0,0),v=(0,0,0),r=None,bg=(0,0,0),ba=(0,0,0)):
    return dict(stamp=t,position=np.array(p,dtype=float),velocity=np.array(v,dtype=float),
        attitude=np.eye(3) if r is None else r,gyro_bias=np.array(bg,dtype=float),accel_bias=np.array(ba,dtype=float))


def integral(force,gyro=(0,0,0),bg=(0,0,0),ba=(0,0,0)):
    w=ImuWindow()
    for i in range(201):w.add(i*.005,gyro,force)
    return w.preintegrate(0.,1.,gyro_bias=bg,accel_bias=ba,noise_density=[.002,.02,.0001,.001])


class ImuFactorTests(unittest.TestCase):
    def test_analytic_acceleration_with_nonzero_initial_velocity_and_gravity(self):
        pre=integral([1.,0.,9.80665]);a=state(0,v=(.4,.2,0));b=state(1,p=(.9,.2,0),v=(1.4,.2,0))
        np.testing.assert_allclose(pair_residual(pre,a,b,gravity=[0,0,-9.80665]),0.,atol=1e-9)

    def test_rotation_and_calibrated_bias(self):
        bg=[.01,-.02,.03];ba=[.1,-.2,.05]
        pre=integral(ba,gyro=np.array(bg)+[0,0,.4],bg=bg,ba=ba)
        np.testing.assert_allclose(pair_residual(pre,state(0,bg=bg,ba=ba),
            state(1,r=exp_so3([0,0,.4]),bg=bg,ba=ba),gravity=[0,0,0]),0.,atol=1e-9)

    def test_rigid_map_gauge_invariance(self):
        pre=integral([1.,0.,9.80665]);a=state(0,v=(.4,.2,0));b=state(1,p=(.92,.2,0),v=(1.4,.2,0))
        g=np.array([0,0,-9.80665]);q=exp_so3([.3,-.4,.7]);shift=np.array([3.,-2.,4.])
        def transform(s):return dict(s,position=q@s['position']+shift,velocity=q@s['velocity'],attitude=q@s['attitude'])
        np.testing.assert_allclose(pair_residual(pre,a,b,gravity=g),pair_residual(pre,transform(a),transform(b),gravity=q@g),atol=1e-9)

    def test_full_covariance_is_used(self):
        pre=integral([0,0,9.80665]);a=state(0);b=state(1,p=(.01,0,0),v=(.02,0,0))
        raw=pair_residual(dict(pre,covariance=np.eye(15)),a,b,gravity=[0,0,-9.80665])
        white=pair_residual(pre,a,b,gravity=[0,0,-9.80665])
        self.assertAlmostEqual(white@white,raw@np.linalg.solve(pre['covariance'],raw),places=8)

    def test_interval_and_noise_validation(self):
        pre=integral([0,0,9.80665])
        for changed in (dict(pre,end=2),dict(pre,gravity_included=True),dict(pre,covariance=np.zeros((15,15)))):
            with self.assertRaises(ValueError):pair_residual(changed,state(0),state(1),gravity=[0,0,-9.80665])
        with self.assertRaises(ValueError):pair_residual(pre,state(.1),state(1),gravity=[0,0,-9.80665])


if __name__=='__main__':unittest.main()
