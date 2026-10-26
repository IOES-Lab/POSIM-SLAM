import unittest
import numpy as np
from posim_slam.inertial import exp_so3, ImuWindow
from posim_slam.visual_inertial_seed import bearing_imu_seed


class VisualInertialSeedTests(unittest.TestCase):
    def scene(self, moving=True):
        t = np.linspace(0, 3, 16)
        velocity = np.array([.45, -.12, .07])
        gravity = np.array([.4, -.6, -9.78])
        rotations = np.array([exp_so3(np.array([.05, -.04, .08])*s) for s in t])
        positions = t[:, None]*velocity
        if moving:
            positions += np.column_stack((.12*(1-np.cos(2*t)),
                                          .08*(np.sin(3*t)-3*t),
                                          .04*(1-np.cos(4*t))))
        else:
            rotations[:] = np.eye(3)
        mount = exp_so3([.1, -.2, .15]);lever = np.array([.14, -.06, .08])
        points = np.random.default_rng(13).uniform([-2, -2, 4], [2, 2, 8], (24, 3))
        tracks = {}
        for key, point in enumerate(points):
            observations = {}
            for i in range(len(t)):
                ray = mount.T @ rotations[i].T @ (point-positions[i]-rotations[i] @ lever)
                observations[i] = ray/np.linalg.norm(ray)
            tracks[key] = observations
        alpha = positions-t[:, None]*velocity-.5*t[:, None]**2*gravity
        # No world alignment is provided to the seed: first IMU defines gauge.
        return (t, rotations, alpha, tracks), dict(camera_to_imu=mount, camera_origin_imu=lever), velocity, gravity

    def test_metric_velocity_and_gravity_with_camera_lever_and_rotation(self):
        args, kwargs, velocity, gravity = self.scene()
        result = bearing_imu_seed(*args, **kwargs)
        np.testing.assert_allclose(result['velocity_initial'], velocity, atol=1e-9)
        np.testing.assert_allclose(result['gravity_initial'], gravity, atol=1e-9)
        self.assertEqual(result['positive_depth_observations'], result['observations'])
        self.assertLess(result['reduced_residual_rms_m'], 1e-12)
        self.assertFalse(result['observability_qualified'])
        self.assertFalse(result['covariance_estimated'])

    def test_constant_velocity_does_not_supply_metric_scale(self):
        args, kwargs, _, _ = self.scene(moving=False)
        with self.assertRaisesRegex(ValueError, 'unobservable'):
            bearing_imu_seed(*args, **kwargs)

    def test_metric_recovery_from_raw_imu_integrals(self):
        args, kwargs, velocity, gravity = self.scene()
        t, rotations, _, tracks = args
        window = ImuWindow(max_samples=4096)
        omega = np.array([.05, -.04, .08])
        for s in np.linspace(0, 3, 1501):
            acceleration = np.array([.48*np.cos(2*s), -.72*np.sin(3*s), .64*np.cos(4*s)])
            window.add(float(s), omega, exp_so3(omega*s).T @ (acceleration-gravity))
        r, alpha, dv = [np.eye(3)], [np.zeros(3)], np.zeros(3)
        for start, end in zip(t[:-1], t[1:]):
            delta = window.preintegrate(float(start), float(end), gyro_bias=[0, 0, 0],
                accel_bias=[0, 0, 0], noise_density=[.001, .02, .0001, .001])
            alpha.append(alpha[-1]+dv*(end-start)+r[-1] @ delta['delta_position'])
            dv += r[-1] @ delta['delta_velocity']
            r.append(r[-1] @ delta['rotation'])
        result = bearing_imu_seed(t, r, alpha, tracks, **kwargs)
        np.testing.assert_allclose(result['velocity_initial'], velocity, atol=2e-5)
        np.testing.assert_allclose(result['gravity_initial'], gravity, atol=2e-5)

    def test_missing_views_and_invalid_bearings_rejected(self):
        args, kwargs, _, _ = self.scene()
        args[3][0] = {0: np.array([0., 0., 1.])}
        with self.assertRaisesRegex(ValueError, 'track_length'):
            bearing_imu_seed(*args, **kwargs)
        args, kwargs, _, _ = self.scene()
        args[3][0][0] *= 2
        with self.assertRaisesRegex(ValueError, 'unit_bearing'):
            bearing_imu_seed(*args, **kwargs)

    def test_stationary_rays_rejected_without_fake_depth(self):
        args, kwargs, _, _ = self.scene(moving=False)
        for observations in args[3].values():
            for i in observations:observations[i] = observations[0]
        with self.assertRaisesRegex(ValueError, 'no_parallax'):
            bearing_imu_seed(*args, **kwargs)


if __name__ == '__main__':unittest.main()
