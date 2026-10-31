"""Numerical controls for equal-length batching of measured seed tracks."""
import unittest
from unittest.mock import patch
import numpy as np
from posim_slam.inertial import exp_so3
from posim_slam.visual_seed_consensus import _track_groups, _reprojection_errors
from posim_slam.visual_inertial_seed import bearing_imu_seed


def scalar_errors(groups, origins, solution, mount, focal, radius, count):
    errors = np.full(count, radius)
    for ids, inverses, states, targets, indices, pixels, rotations in groups:
        for j, ident in enumerate(ids):
            point = inverses[j] @ (targets[j]-states[j] @ solution)
            camera = np.einsum('nji,nj->ni', rotations[j], point-origins[indices[j]]) @ mount
            if np.any(camera[:, 2] <= 0):
                continue
            residual = (camera[:, :2]/camera[:, 2, None]-pixels[j])*focal
            if np.isfinite(residual).all():
                errors[ident] = float(np.median(np.linalg.norm(residual, axis=1)))
    return errors


class SeedReprojectionBatchTests(unittest.TestCase):
    def test_mixed_track_lengths_and_behind_camera_keep_scalar_scores(self):
        rng = np.random.default_rng(37)
        rotations = np.array([exp_so3(rng.normal(0, .04, 3)) for _ in range(32)])
        origins = rng.normal(0, .1, (32, 3));mount = exp_so3([.1, -.2, .03])
        parts = []
        # Exercise odd/even medians, uneven visibility and all supported lengths.
        for i in range(256):
            length = 3+i%30;indices = np.sort(rng.choice(32, length, replace=False))
            pinv = rng.normal(0, .1, (3, 2*length))
            state = rng.normal(0, .1, (2*length, 6));rhs = rng.normal(0, .1, 2*length)
            parts.append((pinv, state, rhs, indices, rng.normal(0, .2, (length, 2))))
        groups = _track_groups(parts, rotations)
        for _ in range(8):
            solution = rng.normal(0, .1, 6)
            options = (groups, origins, solution, mount, np.array([300., 320.]), 1.5, len(parts))
            expected = scalar_errors(*options);actual = _reprojection_errors(*options)
            np.testing.assert_allclose(actual, expected, atol=1e-10, rtol=1e-12)
            np.testing.assert_array_equal(actual < 1.5, expected < 1.5)
            self.assertTrue(np.any(actual == 1.5))

    def test_full_consensus_preserves_mixed_visibility_outlier_decisions(self):
        rng = np.random.default_rng(13);t = np.linspace(0, 3, 16)
        velocity = np.array([.45, -.12, .07]);gravity = np.array([.4, -.6, -9.78])
        rotations = np.array([exp_so3(np.array([.05, -.04, .08])*s) for s in t])
        positions = t[:, None]*velocity+np.column_stack((.12*(1-np.cos(2*t)),
            .08*(np.sin(3*t)-3*t), .04*(1-np.cos(4*t))))
        mount = exp_so3([.1, -.2, .15]);lever = np.array([.14, -.06, .08]);focal = np.array([320., 300.])
        tracks = {}
        for ident, point in enumerate(rng.uniform([-2,-2,4], [2,2,8], (64,3))):
            observations = {}
            for i in range(16):
                if ident%3 == 0 and i%4 == 0:
                    continue
                ray = mount.T @ rotations[i].T @ (point-positions[i]-rotations[i] @ lever)
                pixel = ray[:2]/ray[2]*focal+rng.normal(0, .08, 2)
                if ident < 20:
                    pixel += [9*np.sin(i*.8+ident), 7*np.cos(i*.6+ident)]
                ray = np.r_[pixel/focal, 1.];observations[i] = ray/np.linalg.norm(ray)
            tracks[ident] = observations
        alpha = positions-t[:, None]*velocity-.5*t[:, None]**2*gravity
        args = (t, rotations, alpha, tracks)
        options = dict(camera_to_imu=mount, camera_origin_imu=lever,
            gravity_magnitude=float(np.linalg.norm(gravity)), pixel_sigma=.5, focal_lengths=focal)
        with patch('posim_slam.visual_seed_consensus._reprojection_errors', scalar_errors):
            expected = bearing_imu_seed(*args, **options)
        actual = bearing_imu_seed(*args, **options)
        for key in ('inlier_tracks', 'rejected_tracks', 'ransac_hypotheses'):
            self.assertEqual(actual[key], expected[key])
        for key in ('velocity_initial', 'gravity_initial'):
            np.testing.assert_allclose(actual[key], expected[key], atol=1e-10, rtol=1e-10)
        self.assertFalse(actual['observability_qualified'])
        self.assertFalse(actual['covariance_estimated'])


if __name__ == '__main__':unittest.main()
