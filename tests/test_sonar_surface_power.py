import unittest

import numpy as np

from posim_slam.sonar_surface_power import SurfacePowerModel, TriangleSurface


def rectangle(x, half_y=3., half_z=3.):
    a, b, c, d = np.array([[x, -half_y, -half_z], [x, half_y, -half_z],
                          [x, half_y, half_z], [x, -half_y, half_z]])
    return np.array([[a, b, c], [a, c, d]])


class SurfacePowerTests(unittest.TestCase):
    def model(self, **kwargs):
        return SurfacePowerModel(np.linspace(-.2, .2, 7), np.linspace(-.1, .1, 5),
                                 np.arange(80) * .1, maximum_range_m=7.9, **kwargs)

    def test_first_hit_analytic_plane_miss_parallel_and_maximum_boundary(self):
        scene = TriangleSurface(rectangle(4., half_y=1., half_z=1.))
        d = np.array([[1., 0, 0], [1., .2, 0], [1., 1., 0], [0, 1., 0]])
        d /= np.linalg.norm(d, axis=1)[:, None]
        ranges, ids = scene.first_hits(np.zeros(3), d, 4.1)
        np.testing.assert_allclose(ranges[:2], [4., 4. * np.sqrt(1.04)])
        self.assertTrue(np.isinf(ranges[2:]).all())
        self.assertTrue(np.all(ids[2:] == -1))
        self.assertEqual(scene.first_hits(np.zeros(3), d[:1], 4.)[0][0], 4.)

    def test_bvh_matches_independent_axis_aligned_box_intervals(self):
        # All six faces of [2,4] x [-1,1] x [-1,1], forcing BVH subdivision.
        front, back = rectangle(2., 1., 1.), rectangle(4., 1., 1.)
        triangles = [front, back]
        for axis in (1, 2):
            for side in (-1., 1.):
                other = 3 - axis
                corners = []
                for x, v in ((2., -1.), (4., -1.), (4., 1.), (2., 1.)):
                    p = np.zeros(3); p[0] = x; p[axis] = side; p[other] = v
                    corners.append(p)
                a, b, c, d = corners
                triangles.append(np.array([[a, b, c], [a, c, d]]))
        # Duplicate geometric subdivisions trigger nodes without changing geometry.
        scene = TriangleSurface(np.concatenate(triangles * 2))
        rng = np.random.default_rng(42)
        rays = rng.normal(size=(200, 3)); rays /= np.linalg.norm(rays, axis=1)[:, None]
        near = np.minimum(np.array([2., -1., -1.]) / rays,
                          np.array([4., 1., 1.]) / rays).max(axis=1)
        far = np.maximum(np.array([2., -1., -1.]) / rays,
                         np.array([4., 1., 1.]) / rays).min(axis=1)
        expected = np.where((far >= near) & (near > 0) & (near <= 10), near, np.inf)
        np.testing.assert_allclose(scene.first_hits(np.zeros(3), rays, 10)[0], expected)

    def test_zero_reflectivity_surface_still_occludes_the_rear(self):
        model = self.model()
        scene = TriangleSurface(np.concatenate([rectangle(3.), rectangle(5.)]))
        observation = model.observation(scene, np.zeros(3), np.eye(3))
        self.assertLess(observation.hit_triangles.max(), 2)
        np.testing.assert_array_equal(observation.forward([0., 0., 1., 1.]),
                                      np.zeros(model.shape))
        rear = model.observation(TriangleSurface(rectangle(5.)), np.zeros(3), np.eye(3))
        self.assertGreater(rear.forward([1., 1.]).sum(), 0)

    def test_partial_occlusion_retains_visible_rear_rays(self):
        model = self.model()
        scene = TriangleSurface(np.concatenate([rectangle(3., .15, .15), rectangle(5.)]))
        observation = model.observation(scene, np.zeros(3), np.eye(3))
        self.assertTrue(np.any(observation.hit_triangles < 2))
        self.assertTrue(np.any(observation.hit_triangles >= 2))
        self.assertGreater(observation.forward([0., 0., 1., 1.]).sum(), 0)

    def test_rigid_transform_and_winding_leave_power_unchanged(self):
        model = self.model(); t = rectangle(3.15)
        theta = .43
        rotation = np.array([[np.cos(theta), -np.sin(theta), 0],
                             [np.sin(theta), np.cos(theta), 0], [0, 0, 1.]])
        shift = np.array([2., -3., .7])
        original = model.observation(TriangleSurface(t), np.zeros(3), np.eye(3))
        moved = model.observation(TriangleSurface(t[:, ::-1] @ rotation.T + shift), shift, rotation)
        np.testing.assert_allclose(original.forward([.001, .001]),
                                   moved.forward([.001, .001]), rtol=1e-12, atol=1e-8)

    def test_source_power_matches_independent_frequency_sum(self):
        model = self.model(); plane = 3.15
        observation = model.observation(TriangleSurface(rectangle(plane)), np.zeros(3), np.eye(3))
        az, el = np.linspace(-.2, .2, 7), np.linspace(-.1, .1, 5)
        f = np.arange(80)
        window = .54 - .46 * np.cos(2 * np.pi * (f + 1) / 80)
        window /= np.linalg.norm(window)
        curvature = ((2 * np.pi / 80) ** 2 * 2
                     * (window.sum() * (window @ (f * f)) - (window @ f) ** 2) / 80)
        amplitude = abs(np.sinc(.884 / (az[1] - az[0]) * np.sin(az[:, None] - az)))
        beam_power = amplitude ** 2 / np.sum(amplitude ** 2)
        expected = np.zeros(model.shape)
        absolute_bound = np.zeros(model.shape[1])
        for i, a in enumerate(az):
            for e in el:
                cosine = np.cos(a) * np.cos(e)
                distance = plane / cosine
                spectrum = window * np.exp(2j * np.pi * f * distance / .1 / 80)
                response = abs(np.fft.fft(spectrum)) ** 2
                physical = (1e10 * .001 * 2 * (az[1] - az[0]) * (el[1] - el[0])
                            * distance ** -3 * np.exp(-4 * .0354 * np.log(10) / 20 * distance)
                            * cosine ** 2 * (7500 / 80) ** 2)
                expected += physical * response[:, None] * beam_power[:, i]
                absolute_bound += physical * 80 * curvature / (8 * 8 ** 2) * beam_power[:, i]
        # Known-window interpolation at eight sub-bins is bounded, not exact DFT.
        error = abs(observation.forward([.001, .001]) - expected)
        self.assertTrue(np.all(error <= absolute_bound[None, :]))

    def test_grouped_adjoint_and_reflectivity_finite_difference(self):
        model = self.model(range_group=4, beam_group=3)
        scene = TriangleSurface(np.concatenate([rectangle(3., .15, .15), rectangle(5.)]))
        observation = model.observation(scene, np.zeros(3), np.eye(3))
        rng = np.random.default_rng(4); x = rng.normal(size=4); y = rng.normal(size=model.shape)
        np.testing.assert_allclose(np.sum(observation.forward(x) * y),
                                   x @ observation.adjoint(y), rtol=1e-12)
        direction = rng.normal(size=4); epsilon = 1e-5
        delta = (observation.forward(x + epsilon * direction) - observation.forward(x)) / epsilon
        np.testing.assert_allclose(delta, observation.forward(direction), rtol=1e-8, atol=1e-3)

    def test_invalid_geometry_pose_and_source_profile_are_rejected(self):
        with self.assertRaises(ValueError): TriangleSurface(np.zeros((1, 3, 3)))
        with self.assertRaises(ValueError): TriangleSurface(np.ones((0, 3, 3)))
        with self.assertRaises(ValueError): self.model(fractional_samples=100)
        with self.assertRaises(ValueError): self.model(range_group=0)
        with self.assertRaises(ValueError):
            self.model().observation(TriangleSurface(rectangle(3.)), np.zeros(3), -np.eye(3))

    def test_source_float32_angular_cancellation_and_nonuniform_grid(self):
        start, end = np.float32(-np.pi / 3), np.float32(np.pi / 3)
        az = start + np.arange(256, dtype=np.float32) * np.float32((end - start) / 255)
        el = np.linspace(-.1, .1, 16)
        SurfacePowerModel(az, el, np.arange(80) * .1, maximum_range_m=7.9)
        az[100] += .001
        with self.assertRaisesRegex(ValueError, 'uniform_source'):
            SurfacePowerModel(az, el, np.arange(80) * .1, maximum_range_m=7.9)

    def test_geometry_derivative_includes_incidence_and_range_changes(self):
        model = self.model(range_group=4, beam_group=3)
        triangles = np.array([[[3.157, -5., -3.], [3.157, 5., -3.], [3.157, 0., 8.]]])
        obs = model.observation(TriangleSurface(triangles), np.zeros(3), np.eye(3))
        rng = np.random.default_rng(35); image = rng.normal(size=model.shape); mu = np.array([.001])
        gradient = obs.geometry_adjoint(image, mu)
        for _ in range(4):
            direction = rng.normal(size=triangles.shape); epsilon = 1e-6
            plus = model.observation(TriangleSurface(triangles + epsilon * direction), np.zeros(3), np.eye(3))
            minus = model.observation(TriangleSurface(triangles - epsilon * direction), np.zeros(3), np.eye(3))
            np.testing.assert_array_equal(plus.hit_triangles, minus.hit_triangles)
            numeric = np.sum((plus.forward(mu) - minus.forward(mu)) * image) / (2 * epsilon)
            np.testing.assert_allclose(numeric, np.sum(gradient * direction), rtol=2e-6)


if __name__ == '__main__':
    unittest.main()
