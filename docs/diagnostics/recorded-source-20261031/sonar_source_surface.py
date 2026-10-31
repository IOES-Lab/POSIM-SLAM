# SPDX-License-Identifier: Apache-2.0
"""Conditional power observations for supplied sensor-local finite surfaces.

Nearest intersections retain occlusion. Float32 spherical ranges feed POSIM's
range-derived normals, rather than ideal triangle normals. Existing exact
Fourier amplitudes and complex-Gaussian moments retain pixel correlations.
Gazebo rasterization, hardware calibration and inferred geometry/pose remain
outside this diagnostic. No navigation or DVL state is a measurement here.
"""
import numpy as np
from .sonar_source_normals import ray_surface_normals
from .sonar_speckle import surface_pixel_moments


class SourceSurfaceObservation:
    """Freeze one caller-supplied surface in the sonar's own Cartesian frame.

    The model is a SurfacePowerModel with every vertical ray selected. Triangle
    labels select reflectivity, independently of opaque first-hit visibility.
    Geometry crossing the near clip is unsupported. This is a conditional
    observation operator, without a geometry fit, native factor or covariance
    for an estimated surface. No ideal-normal geometry derivative is exposed.
    """

    def __init__(self, model, surface):
        trace = model.observation(surface, np.zeros(3), np.eye(3))
        finite = np.isfinite(trace.ranges)
        if np.any(finite & (trace.ranges < model.minimum_range_m)):
            raise ValueError('near_clip_intersection_not_supported')
        self.model, self.size = model, len(surface.triangles)
        self.ranges = np.where(finite, trace.ranges, 0.).astype(np.float32)
        self.hit_triangles = trace.hit_triangles.copy()
        grid = model.directions.reshape(*model.ray_shape, 3)
        elevation = np.arctan2(grid[:, 0, 2], np.hypot(grid[:, 0, 0], grid[:, 0, 1]))
        self.normals = ray_surface_normals(self.ranges, model.azimuth_rad, elevation,
                                           model.maximum_range_m)
        keep = ((self.hit_triangles >= 0) & (self.ranges > 0)
                & (self.ranges <= model.maximum_range_m))
        self.triangle_ids = self.hit_triangles[keep]
        self.beam_ids = np.nonzero(keep)[1]
        self.selected_ranges = self.ranges[keep].astype(float)
        self.unit_power = (model.power_constant * self.selected_ranges**-3
                           * np.exp(-4*model.alpha*self.selected_ranges)
                           * self.normals[..., 2][keep].astype(float)**2)
        for value in (self.ranges, self.hit_triangles, self.normals, self.triangle_ids,
                      self.beam_ids, self.selected_ranges, self.unit_power):
            value.setflags(write=False)

    def moments(self, reflectivity, pixels, weights=None):
        """Exact conditional moments of selected native [range, beam] powers.

        Reflectivity is supplied per triangle. Pixel count and workspace limits
        are inherited from surface_pixel_moments; none are independent looks by
        assumption. Unknown DVL range variance is not supplied by this result.
        """
        return surface_pixel_moments(self, reflectivity, pixels, weights)
