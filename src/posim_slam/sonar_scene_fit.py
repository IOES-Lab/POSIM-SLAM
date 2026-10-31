# SPDX-License-Identifier: Apache-2.0
"""Experimental inverse fit of finite acoustic scene hypotheses.

This optimizer changes shared vertex heights, using original power observations
and fixed navigation means. It reuses SourceSurfaceObservation, including opaque
first hits, float32 range-derived normals and exact periodic Fourier response.
No scene truth, XYZ observations, native update or map admission is supplied.
The mesh is a finite hypothesis: its absence outside a boundary is unknown
support, never measured free space. Even a visible fitted face is not a verified
landmark. Source-normal and visibility derivatives are deliberately *not*
claimed: a frozen-normal radial surrogate proposes steps, and every proposal
is retraced and accepted only by the actual source-model objective.
"""
import numpy as np
from .sonar_source_surface import SourceSurfaceObservation
from .sonar_surface_power import TriangleSurface
from .sonar_speckle import periodic_hamming_amplitude


def _response(observation, pixels):
    m = observation.model
    az = m.azimuth_rad
    beam = abs(np.sinc(.884/(az[1]-az[0])*np.sin(az[:, None]-az)))
    beam /= np.linalg.norm(beam)
    angular = beam[pixels[:, 1, None], observation.beam_ids]
    step = m.native_ranges_m[1]-m.native_ranges_m[0]
    delay = observation.selected_ranges[None, :]/step-pixels[:, 0, None]
    amplitude = periodic_hamming_amplitude(delay, len(m.native_ranges_m))
    return angular, delay, amplitude, step


def _mean_and_radial(observation, pixels, reflectivity, radial=False):
    """Bounded exact mean and optional frozen-normal radial proposal derivative.

    The central difference only proposes a direction (1e-3 native range bin).
    Acceptance never uses this derivative or a linearized/smoothed renderer.
    Eight-row chunks bound the response workspace at the full sensor grid.
    """
    q = reflectivity[observation.triangle_ids]*observation.unit_power
    means, slopes = [], []
    bins = len(observation.model.native_ranges_m)
    for start in range(0, len(pixels), 8):
        angular, delay, amplitude, step = _response(observation, pixels[start:start+8])
        response = angular**2 * abs(amplitude)**2
        means.append(response @ q)
        if radial:
            epsilon = 1e-3
            derivative = (abs(periodic_hamming_amplitude(delay+epsilon, bins))**2
                          - abs(periodic_hamming_amplitude(delay-epsilon, bins))**2)/(2*epsilon)
            slopes.append(q * (response*(-3/observation.selected_ranges
                                        - 4*observation.model.alpha)
                               + angular**2*derivative/step))
    mean = np.concatenate(means)
    return mean, np.concatenate(slopes) if radial else None


def _vertex_radial_gradient(vertices, faces, view, observation, radial):
    """Accumulate first-hit range derivatives onto shared world-z vertices."""
    ids = observation.triangle_ids
    triangles = vertices[faces[ids]]
    directions = view['model'].directions.reshape(*view['model'].ray_shape, 3)
    directions = directions[observation.hit_triangles >= 0] @ view['rotation'].T
    hit = view['origin'] + observation.selected_ranges[:, None]*directions
    e1, e2 = triangles[:, 1]-triangles[:, 0], triangles[:, 2]-triangles[:, 0]
    cross = np.cross(e1, e2)
    normal = cross/np.linalg.norm(cross, axis=1)[:, None]
    offset = hit-triangles[:, 0]
    aa, ab, bb = (np.sum(a*b, axis=1) for a, b in ((e1, e1), (e1, e2), (e2, e2)))
    ca, cb = np.sum(offset*e1, axis=1), np.sum(offset*e2, axis=1)
    determinant = aa*bb-ab**2
    u, v = (bb*ca-ab*cb)/determinant, (aa*cb-ab*ca)/determinant
    barycentric = np.column_stack((1-u-v, u, v))
    factor = normal[:, 2]/np.sum(normal*directions, axis=1)
    gradient = np.zeros(len(vertices))
    for j in range(3):
        np.add.at(gradient, faces[ids, j], radial*barycentric[:, j]*factor)
    return gradient


class SceneHeightFit:
    """Conditional fixed-pose fit; all geometry remains an unqualified hypothesis.

    Views provide model, origin, rotation, integer native pixels and original
    nonnegative power. Reflectivity is supplied per face, not learned as a
    per-pixel gain. One common positive brightness gain is profiled per view.
    The marginal exponential score is composite: correlated pixels are not
    declared independent. ``edges`` and ``regularization`` are a declared shape
    prior on *changes* to neighbouring heights, not sensor noise/confidence.
    """

    def __init__(self, vertices, faces, reflectivity, views, *, edges=None,
                 regularization=0.):
        v, f, mu = np.asarray(vertices), np.asarray(faces), np.asarray(reflectivity)
        if (v.ndim != 2 or v.shape[1:] != (3,) or not 3 <= len(v) <= 4096
                or v.dtype.kind not in 'fiu' or not np.isfinite(v).all()
                or f.ndim != 2 or f.shape[1:] != (3,) or not 1 <= len(f) <= 8192
                or f.dtype.kind not in 'iu' or np.any(f < 0) or np.any(f >= len(v))
                or mu.shape != (len(f),) or mu.dtype.kind not in 'fiu'
                or not np.isfinite(mu).all() or np.any(mu < 0)
                or not np.isfinite(regularization) or regularization < 0
                or not 1 <= len(views) <= 32):
            raise ValueError('bounded_finite_scene_fit_required')
        self.vertices, self.faces = v.astype(float, copy=True), f.copy()
        self.reflectivity = mu.astype(float, copy=True)
        e = np.empty((0, 2), dtype=int) if edges is None else np.asarray(edges)
        if (e.ndim != 2 or e.shape[1:] != (2,) or e.dtype.kind not in 'iu'
                or len(e) > 24576 or np.any(e < 0) or np.any(e >= len(v))):
            raise ValueError('bounded_scene_prior_edges_required')
        self.edges, self.regularization = e.copy(), float(regularization)
        self.views = []
        for view in views:
            p, r = np.asarray(view['origin']), np.asarray(view['rotation'])
            x, power = np.asarray(view['pixels']), np.asarray(view['power'])
            model = view['model']
            if (p.shape != (3,) or r.shape != (3, 3) or not np.isfinite(p).all()
                    or not np.isfinite(r).all()
                    or not np.allclose(r.T@r, np.eye(3), rtol=0, atol=1e-8)
                    or not np.isclose(np.linalg.det(r), 1., rtol=0, atol=1e-8)
                    or x.ndim != 2 or x.shape[1:] != (2,) or not 1 <= len(x) <= 256
                    or x.dtype.kind not in 'iu' or np.any(x < 0)
                    or np.any(x[:, 0] >= len(model.native_ranges_m))
                    or np.any(x[:, 1] >= model.nbeams)
                    or power.shape != (len(x),) or power.dtype.kind not in 'fiu'
                    or not np.isfinite(power).all() or np.any(power < 0)
                    or not np.any(power > 0)):
                raise ValueError('original_power_pixels_and_fixed_rigid_pose_required')
            self.views.append(dict(model=model, origin=p.astype(float, copy=True),
                                   rotation=r.astype(float, copy=True), pixels=x.copy(),
                                   power=power.astype(float, copy=True)))
        TriangleSurface(self.vertices[self.faces])

    def evaluate(self, heights, *, proposal=False, moments=False):
        z = np.asarray(heights)
        if z.shape != (len(self.vertices),) or not np.isfinite(z).all():
            raise ValueError('finite_shared_vertex_heights_required')
        vertices = self.vertices.copy()
        vertices[:, 2] = z
        gradient = np.zeros(len(z))
        counts = np.zeros(len(self.faces), dtype=np.int64)
        score, details = 0., []
        for view in self.views:
            local = (vertices[self.faces]-view['origin'])@view['rotation']
            observation = SourceSurfaceObservation(view['model'], TriangleSurface(local))
            mean, derivative = _mean_and_radial(observation, view['pixels'],
                                               self.reflectivity, proposal)
            active = mean > 0
            power = view['power']
            if np.any((power > 0) & ~active) or not np.any(power[active] > 0):
                raise ValueError('unmodelled_return_support')
            gain = np.mean(power[active]/mean[active])
            value = float(np.log(mean[active]).sum()+active.sum()*(np.log(gain)+1))
            residual = np.zeros_like(mean)
            residual[active] = power[active]/(gain*mean[active])-1
            score += value
            counts += np.bincount(observation.triangle_ids, minlength=len(counts))
            if proposal:
                coefficient = np.zeros_like(mean)
                coefficient[active] = -residual[active]/mean[active]
                gradient += _vertex_radial_gradient(vertices, self.faces, view,
                                                    observation, coefficient@derivative)
            detail = dict(score=value, gain=float(gain), mean=mean, observed=power.copy(),
                          residual=residual, pixels=view['pixels'].copy())
            if moments:
                detail['moments'] = observation.moments(self.reflectivity, view['pixels'])
            details.append(detail)
        change = z-self.vertices[:, 2]
        a, b = self.edges.T
        delta = change[a]-change[b]
        penalty = self.regularization*float(delta@delta)/2
        np.add.at(gradient, a, self.regularization*delta)
        np.add.at(gradient, b, -self.regularization*delta)
        return dict(score=score+penalty, data_score=score, prior_penalty=penalty,
                    gradient=gradient, views=details, face_ray_counts=counts,
                    vertices=vertices, faces=self.faces.copy(), map_accepted=False,
                    fixed_navigation=True, native_update=False,
                    outside_mesh_support='unknown')

    def fit(self, *, iterations=12, maximum_step_m=.05, maximum_change_m=.4,
            backtracks=5, callback=None):
        """Bounded descent with exact re-tracing and a complete proposal trace."""
        if (type(iterations) is not int or not 1 <= iterations <= 100
                or type(backtracks) is not int or not 1 <= backtracks <= 12
                or not np.isfinite([maximum_step_m, maximum_change_m]).all()
                or not 0 < maximum_step_m <= maximum_change_m <= 2):
            raise ValueError('bounded_height_search_required')
        initial = self.vertices[:, 2]
        z = initial.copy()
        trace = []
        def log(row):
            trace.append(row)
            if callback is not None:
                callback(row)
        for iteration in range(iterations):
            current = self.evaluate(z, proposal=True)
            gradient = current['gradient']
            log(dict(iteration=iteration, kind='current', accepted=True,
                     score=current['score'], heights=z.tolist(), error=None))
            scale = np.max(abs(gradient))
            if scale == 0:
                break
            accepted = False
            for backtrack in range(backtracks):
                step = maximum_step_m*2.**-backtrack
                candidate = np.clip(z-step*gradient/scale,
                                    initial-maximum_change_m, initial+maximum_change_m)
                try:
                    trial = self.evaluate(candidate)
                    value, error = trial['score'], None
                    accepted = value < current['score']
                except ValueError as exception:
                    value, error = None, str(exception)
                log(dict(iteration=iteration, kind='proposal', backtrack=backtrack,
                         accepted=accepted, score=value, heights=candidate.tolist(), error=error))
                if accepted:
                    z = candidate
                    break
            if not accepted:
                break
        result = self.evaluate(z, moments=True)
        result['trace'] = trace
        return result
