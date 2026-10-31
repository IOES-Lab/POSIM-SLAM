import unittest
import numpy as np
from posim_slam.inertial import exp_so3, ImuWindow
from posim_slam.visual_inertial_seed import bearing_imu_seed, tracked_imu_seed


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
        frames = []
        for i, s in enumerate(t):
            frames.append(dict(type='startup_tracks', stamp_ns=1_000_000_000+round(s*1e9),
                width=256, height=256, tracks=[dict(id=ident,
                    pixel=(ray[i][:2]/ray[i][2]*80+128).tolist(), point=[999,999,999])
                    for ident,ray in tracks.items()]))
        shifted = ImuWindow(max_samples=4096)
        for stamp, gyro, accel in window.samples:shifted.add(stamp+1.,gyro,accel)
        options=dict(intrinsics=[80,80,128,128],gyro_bias=[0,0,0],accel_bias=[0,0,0],
                     noise_density=[.001,.02,.0001,.001],**kwargs)
        endpoint = tracked_imu_seed(frames,shifted,**options)
        acceleration_velocity = np.array([.24*np.sin(6),.24*(np.cos(9)-1),.16*np.sin(12)])
        np.testing.assert_allclose(endpoint['velocity_imu'],rotations[-1].T @
                                   (velocity+acceleration_velocity),atol=3e-5)
        np.testing.assert_allclose(endpoint['up_direction_imu'],
                                   -rotations[-1].T @ gravity/np.linalg.norm(gravity),atol=1e-5)
        self.assertEqual(endpoint['last_measurement_ns'],frames[-1]['stamp_ns'])
        robust = tracked_imu_seed(frames,shifted,**options,
            gravity_magnitude=float(np.linalg.norm(gravity)),pixel_sigma=.5)
        np.testing.assert_allclose(robust['velocity_imu'],rotations[-1].T @
                                   (velocity+acceleration_velocity),atol=4e-5)
        self.assertEqual(robust['seed_method'],'bearing_imu_ransac_fixed_gravity')
        self.assertEqual(robust['inlier_tracks'],24)
        frames[1]['tracks'].append(frames[1]['tracks'][0])
        with self.assertRaisesRegex(ValueError,'pixel_identity'):
            tracked_imu_seed(frames,shifted,**options)

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

    def test_robust_seed_rejects_moving_tracks_without_losing_metric_motion(self):
        args, kwargs, velocity, gravity = self.scene()
        focal = np.array([320., 300.]);rng = np.random.default_rng(42)
        for ident, observations in args[3].items():
            for i, ray in observations.items():
                pixel = ray[:2]/ray[2]*focal+rng.normal(0, .08, 2)
                if ident < 9:
                    pixel += np.array([9*np.sin(i*.8+ident), 7*np.cos(i*.6+ident)])
                ray = np.r_[pixel/focal, 1.]
                observations[i] = ray/np.linalg.norm(ray)
        options = dict(gravity_magnitude=float(np.linalg.norm(gravity)), pixel_sigma=.5,
                       focal_lengths=focal)
        result = bearing_imu_seed(*args, **kwargs, **options)
        repeated = bearing_imu_seed(*args, **kwargs, **options)
        np.testing.assert_allclose(result['velocity_initial'], velocity, atol=.015)
        np.testing.assert_allclose(result['gravity_initial'], gravity, atol=.01)
        np.testing.assert_array_equal(result['velocity_initial'], repeated['velocity_initial'])
        self.assertAlmostEqual(np.linalg.norm(result['gravity_initial']), options['gravity_magnitude'])
        self.assertGreaterEqual(result['inlier_tracks'], 14)
        self.assertGreaterEqual(result['rejected_tracks'], 8)
        self.assertFalse(result['covariance_estimated'])
        self.assertFalse(result['observability_qualified'])

    def test_known_gravity_does_not_invent_constant_velocity_scale(self):
        args, kwargs, _, gravity = self.scene(moving=False)
        with self.assertRaisesRegex(ValueError, 'unobservable'):
            bearing_imu_seed(*args, **kwargs, gravity_magnitude=float(np.linalg.norm(gravity)),
                             pixel_sigma=.5, focal_lengths=[300,300])

    def test_robust_seed_requires_consensus_and_explicit_calibration(self):
        args, kwargs, _, gravity = self.scene()
        options = dict(gravity_magnitude=float(np.linalg.norm(gravity)), pixel_sigma=.5,
                       focal_lengths=[300,300])
        for field, value in (('gravity_magnitude', True), ('gravity_magnitude', -1),
                             ('pixel_sigma', 0), ('pixel_sigma', float('nan')),
                             ('focal_lengths', [0,300]), ('focal_lengths', None)):
            with self.subTest(field=field, value=value), self.assertRaisesRegex(ValueError, 'robust_calibration'):
                bearing_imu_seed(*args, **kwargs, **dict(options, **{field:value}))
        rng = np.random.default_rng(17)
        for observations in args[3].values():
            for i in observations:
                ray = np.r_[rng.uniform(-.5,.5,2),1.]
                observations[i] = ray/np.linalg.norm(ray)
        with self.assertRaisesRegex(ValueError, 'no_consensus'):
            bearing_imu_seed(*args, **kwargs, **options)


if __name__ == '__main__':unittest.main()
