"""Independent analytic checks for gauge/scale and rotating sensor origins."""
import importlib.util
from pathlib import Path
import unittest
import numpy as np

spec = importlib.util.spec_from_file_location('vio_eval', Path(__file__).resolve().parents[2] / 'tools/evaluate_vio.py')
evaluation = importlib.util.module_from_spec(spec)
spec.loader.exec_module(evaluation)


class EvaluationTests(unittest.TestCase):
    def trajectory(self):
        names = ('stamp_ns', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw')
        truth = np.zeros(51, dtype=[(n, 'f8') for n in names])
        mount = {'translation': [.2, .1, -.1], 'rotation': [1., 0., 0., 0.]}
        gauge = evaluation.matrix([0, 0, np.sin(.4), np.cos(.4)])
        shift = np.array([5., -2., 1.])
        states = np.zeros(11, dtype=truth.dtype)
        for i in range(51):
            t = i * .05
            q = np.array([0, 0, np.sin(.1*t), np.cos(.1*t)])
            for name, value in zip(names, [1e9+i*50_000_000, t, 0, 0, *q]):
                truth[name][i] = value
            if i % 4:
                continue
            r = evaluation.matrix(q)
            p = np.array([t, 0, 0]) + r @ np.array(mount['translation'])
            # Rz(.2*t-.8) * Rx(pi), expressed as an xyzw quaternion.
            estimated_q = [np.cos(.1*t-.4), np.sin(.1*t-.4), 0, 0]
            estimated_p = gauge.T @ (p-shift)
            for name, value in zip(names, [truth['stamp_ns'][i], *estimated_p, *estimated_q]):
                states[name][i//4] = value
            if i == 40:
                break
        # Finish truth independently after the last selected estimate.
        for i in range(41, 51):
            t=i*.05;q=[0,0,np.sin(.1*t),np.cos(.1*t)]
            for name,value in zip(names,[1e9+i*50_000_000,t,0,0,*q]):truth[name][i]=value
        return states, truth, mount

    def test_initial_gauge_and_rotating_lever_arm_have_zero_error(self):
        states, truth, mount = self.trajectory()
        result, _ = evaluation.evaluate(states, truth, mount)
        self.assertLess(result['position_rmse_m'], 1e-12)
        self.assertLess(result['rotation_rmse_rad'], 1e-7)
        self.assertGreater(result['rpe_1s_pairs'], 0)

    def test_scale_drift_is_not_hidden_by_trajectory_fit(self):
        states, truth, mount = self.trajectory()
        for name in ('x', 'y', 'z'):states[name]*=2
        result, _ = evaluation.evaluate(states, truth, mount)
        self.assertGreater(result['position_rmse_m'], .5)
        self.assertGreater(result['rpe_1s_position_rmse_m'], .5)

    def test_initial_gravity_tilt_error_is_not_aligned_away(self):
        states,truth,mount=self.trajectory()
        for row in states:
            x,y,z,w=[row[n] for n in ('qx','qy','qz','qw')]
            a,b=np.sin(.05),np.cos(.05)
            for name,value in zip(('qx','qy','qz','qw'),(b*x+a*w,b*y-a*z,b*z+a*y,b*w-a*x)):
                row[name]=value
        result,_=evaluation.evaluate(states,truth,mount)
        self.assertGreater(result['rotation_rmse_rad'],.05)

    def test_truth_gaps_and_antipodal_quaternions(self):
        states, truth, mount = self.trajectory()
        with self.assertRaisesRegex(ValueError, 'truth_not_bracketing_or_gap'):
            evaluation.evaluate(states, truth[::3], mount)
        a = np.array([0, 0, np.sin(.2), np.cos(.2)])
        self.assertTrue(np.allclose(evaluation.slerp(a, -a, .5), a))


if __name__ == '__main__':
    unittest.main()
