import unittest
import numpy as np
from posim_slam.sonar_scene_fit import SceneHeightFit, _mean_and_radial, _vertex_radial_gradient
from posim_slam.sonar_surface_power import SurfacePowerModel, TriangleSurface
from posim_slam.sonar_source_surface import SourceSurfaceObservation


def fixture():
    # Two disconnected finite reflectors. No plane is inferred outside them.
    vertices = np.array([[2., -2., -1.], [5., -2., -1.], [5., 0., -1.], [2., 0., -1.],
                         [2., .1, -.6], [5., .1, -.6], [5., 2., -.6], [2., 2., -.6]])
    faces = np.array([[0, 1, 2], [0, 2, 3], [4, 5, 6], [4, 6, 7]])
    model = SurfacePowerModel(np.linspace(-.5, .5, 17), np.linspace(-.5, -.1, 21),
                              np.arange(160)*.05, maximum_range_m=7.9)
    pixels = np.array([(i, j) for i in [48, 56, 64, 72, 80] for j in [3, 6, 10, 13]])
    view = dict(model=model, origin=np.zeros(3), rotation=np.eye(3), pixels=pixels)
    observation = SourceSurfaceObservation(model, TriangleSurface(vertices[faces]))
    view['power'] = observation.moments(np.full(4, .001), pixels)['mean']
    return vertices, faces, view


class SceneFitTests(unittest.TestCase):
    def test_means_match_existing_correlated_operator(self):
        vertices, faces, view = fixture()
        fit = SceneHeightFit(vertices, faces, np.full(4, .001), [view])
        result = fit.evaluate(vertices[:, 2], moments=True)
        row = result['views'][0]
        np.testing.assert_allclose(row['mean'], row['moments']['mean'], rtol=1e-12)
        np.testing.assert_allclose(row['residual'], 0., atol=1e-12)
        self.assertGreater(row['moments']['covariance'][0, 1], 0.)
        self.assertEqual(result['outside_mesh_support'], 'unknown')
        self.assertFalse(result['map_accepted'])

    def test_frozen_normal_radial_derivative_matches_finite_difference(self):
        vertices, faces, view = fixture()
        obs = SourceSurfaceObservation(view['model'], TriangleSurface(vertices[faces]))
        mean, derivative = _mean_and_radial(obs, view['pixels'], np.full(4, .001), True)
        from copy import copy
        values = []
        step = 1e-6
        direction = np.linspace(-1., 1., len(obs.selected_ranges))
        for sign in [-1, 1]:
            modified = copy(obs)
            modified.selected_ranges = obs.selected_ranges+sign*step*direction
            model = obs.model
            r = modified.selected_ranges
            modified.unit_power = obs.unit_power*(r/obs.selected_ranges)**-3*np.exp(
                -4*model.alpha*(r-obs.selected_ranges))
            values.append(_mean_and_radial(modified, view['pixels'], np.full(4, .001))[0])
        np.testing.assert_allclose(derivative@direction, (values[1]-values[0])/(2*step),
                                   rtol=2e-5, atol=mean.max()*1e-6)

    def test_actual_source_search_improves_two_reflector_hypothesis(self):
        vertices, faces, view = fixture()
        seed = vertices.copy()
        seed[:4, 2] -= .035
        seed[4:, 2] += .035
        fit = SceneHeightFit(seed, faces, np.full(4, .001), [view])
        initial = fit.evaluate(seed[:, 2])
        result = fit.fit(iterations=4, maximum_step_m=.025)
        self.assertLess(result['score'], initial['score'])
        self.assertTrue(any(r['kind'] == 'proposal' and r['accepted'] for r in result['trace']))
        current = None
        for row in result['trace']:
            if row['kind'] == 'current':
                current = row['score']
            elif row['accepted']:
                self.assertLess(row['score'], current)
        self.assertFalse(result['native_update'])

    def test_world_height_range_chain_with_rotated_sensor(self):
        vertices, faces, view = fixture()
        angle = .17
        view['rotation'] = np.array([[np.cos(angle),-np.sin(angle),0.],
                                     [np.sin(angle),np.cos(angle),0.],[0.,0.,1.]])
        view['origin'] = np.array([.1,-.2,.05])
        model = view['model']
        surface = TriangleSurface(vertices[faces])
        trace = model.observation(surface,view['origin'],view['rotation'])
        obs = SourceSurfaceObservation(model,TriangleSurface(
            (vertices[faces]-view['origin'])@view['rotation']))
        np.testing.assert_array_equal(obs.triangle_ids,trace.triangle_ids)
        weights = np.linspace(-1.,1.,len(obs.selected_ranges))
        weights[np.min(trace.barycentric,axis=1)<.005] = 0.
        gradient = _vertex_radial_gradient(vertices,faces,view,obs,weights)
        direction = np.array([.3,-.7,.4,-.2,.8,-.1,.2,-.6])
        costs = []
        keep = obs.hit_triangles.ravel()>=0
        rays = model.directions@view['rotation'].T
        for sign in [-1,1]:
            moved = vertices.copy();moved[:,2] += sign*1e-5*direction
            distance, ids = TriangleSurface(moved[faces]).first_hits(view['origin'],rays,7.9)
            np.testing.assert_array_equal(ids[keep][weights!=0],obs.triangle_ids[weights!=0])
            costs.append(weights@distance[keep])
        np.testing.assert_allclose(gradient@direction,(costs[1]-costs[0])/2e-5,rtol=2e-6)

    def test_absent_geometry_does_not_gain_measured_free_space(self):
        vertices, faces, view = fixture()
        fit = SceneHeightFit(vertices, faces[:2], [.001, .001], [view])
        result = fit.evaluate(vertices[:, 2])
        self.assertGreater(np.linalg.norm(result['views'][0]['residual']), .1)
        self.assertEqual(result['outside_mesh_support'], 'unknown')
        self.assertFalse(result['map_accepted'])

    def test_shape_regularizer_changes_are_explicit_not_sensor_covariance(self):
        vertices, faces, view = fixture()
        fit = SceneHeightFit(vertices, faces, np.full(4, .001), [view],
                             edges=np.array([[0, 1]]), regularization=2.)
        z = vertices[:, 2].copy()
        z[0] += .02
        result = fit.evaluate(z)
        self.assertAlmostEqual(result['prior_penalty'], .02**2)

    def test_inputs_reject_invalid_physical_and_mesh_support(self):
        vertices, faces, view = fixture()
        for invalid in [dict(view, rotation=np.diag([-1., 1., 1.])),
                        dict(view, power=np.full(len(view['power']), np.nan)),
                        dict(view, pixels=view['pixels'].astype(float))]:
            with self.assertRaises(ValueError):
                SceneHeightFit(vertices, faces, np.full(4, .001), [invalid])
        fit = SceneHeightFit(vertices, faces, np.full(4, .001), [view])
        with self.assertRaises(ValueError):
            fit.fit(maximum_step_m=0.)


if __name__ == '__main__':
    unittest.main()
