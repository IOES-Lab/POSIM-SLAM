"""Clip metric map output below a calibrated water plane; no simulator input."""
import math
import json
import numpy as np


class SurfaceFilter:
    def __init__(self, plane=None, clearance_m=0.3):
        # Plane n.dot(p) + d = 0, normal points from water towards air.
        self.automatic = None
        self.plane = None
        if plane is not None and len(plane):
            value = np.asarray(plane, dtype=float)
            if value.shape != (4,) or not np.isfinite(value).all():
                raise ValueError('invalid_surface_plane')
            norm = np.linalg.norm(value[:3])
            if not math.isfinite(norm) or norm < 1e-9:
                raise ValueError('invalid_surface_normal')
            self.plane = value / norm
        if not math.isfinite(clearance_m) or not 0 <= clearance_m <= 5:
            raise ValueError('invalid_surface_clearance')
        self.clearance_m = float(clearance_m)
        self.used = False
        self.invalidated = False

    @classmethod
    def from_node(cls, node):
        from rcl_interfaces.msg import ParameterDescriptor
        descriptor = ParameterDescriptor(read_only=True)
        # Zero normal means unconfigured, never assume map z is world height.
        plane = node.declare_parameter('surface_plane_m', [0., 0., 0., 0.], descriptor).value
        clearance = node.declare_parameter('surface_clearance_m', 0.3, descriptor).value
        result=cls(None if list(plane) == [0., 0., 0., 0.] else plane, clearance)
        config=node.declare_parameter('surface_sensors','',descriptor).value
        path=node.declare_parameter('surface_sensor_file','',descriptor).value
        if path:
            if config:raise ValueError('duplicate_surface_sensor_configuration')
            from pathlib import Path
            config=Path(path).read_text()
        if config:
            if result.plane is not None:raise ValueError('conflicting_surface_calibration_modes')
            from surface_inputs import SurfaceInputs
            result.automatic=SurfaceInputs(node,json.loads(config))
        return result

    def update(self,t,origin,rotation):
        if self.automatic is not None:
            self.plane=self.automatic.update(t,origin,rotation)
        return self.automatic is None or self.plane is not None

    def ready(self):
        return not self.invalidated and (self.automatic is None or self.plane is not None)

    def invalidate(self):
        # A reset can change the visual map origin/orientation. Recalibrate on restart.
        if self.automatic is not None:
            self.automatic.reset();self.plane=None;self.used=False;self.invalidated=False
            return
        if self.used:
            self.invalidated = True

    def status(self):
        state = 'map_reset' if self.invalidated else 'active' if self.plane is not None else 'calibrating' if self.automatic is not None else 'unconfigured'
        return {'state': state, 'plane_m': self.plane.tolist() if state == 'active' else None,
                'clearance_m': self.clearance_m}

    def apply(self, points):
        points = np.asarray(points)
        if points.ndim != 2 or points.shape[1] != 3 or not np.isfinite(points).all():
            raise ValueError('invalid_surface_points')
        if self.invalidated:
            # Do not silently republish an unfiltered map after losing calibration.
            raise ValueError('surface_plane_requires_recalibration')
        if self.automatic is not None and self.plane is None:
            raise ValueError('surface_calibration_pending')
        if self.plane is None:
            return points
        self.used = True
        distance = points.astype(float) @ self.plane[:3] + self.plane[3]
        return points[distance < -self.clearance_m]
