"""Bounded segment-local acoustic feature map with explicitly unresolved height.

Points are matched intensity features, not classified solid obstacles. Projection
onto the sensor fan plane is an assumption; z=0 is not measured elevation.
"""
from collections import OrderedDict
import numpy as np


class AcousticMap:
    def __init__(self, *, cell_size_m=.05, max_points=100000):
        if not np.isfinite(cell_size_m) or cell_size_m <= 0:
            raise ValueError('invalid_cell_size')
        if type(max_points) is not int or max_points < 1:
            raise ValueError('invalid_max_points')
        self.cell_size = cell_size_m
        self.max_points = max_points
        self.segment = None
        self.cells = OrderedDict()

    def push(self, update):
        segment = update['segment']
        if type(segment) is not int or segment < 0:
            raise ValueError('invalid_segment')
        if self.segment != segment or update['state'] == 'lost':
            self.cells.clear()
            self.segment = segment
        if update['state'] != 'tracking':
            return
        points = np.asarray(update['registration']['inlier_points_current_fan_m'], dtype=float)
        position = np.asarray(update['position_fan_m'], dtype=float)
        yaw = update['yaw_fan_rad']
        if (points.ndim != 2 or points.shape[1] != 2 or len(points) > 300
                or position.shape != (2,) or not np.isfinite(points).all()
                or not np.isfinite(position).all() or not np.isfinite(yaw)):
            raise ValueError('invalid_map_update')
        c, s = np.cos(yaw), np.sin(yaw)
        transformed = points @ np.array([[c, -s], [s, c]]).T + position
        if not np.isfinite(transformed).all():
            raise ValueError('invalid_map_coordinates')
        for point in transformed:
            key = tuple(int(v) for v in np.floor(point / self.cell_size))
            self.cells[key] = point.tolist()
            self.cells.move_to_end(key)
            if len(self.cells) > self.max_points:
                self.cells.popitem(last=False)

    def snapshot(self):
        return {'segment': self.segment, 'scope': 'sensor_fan_plane',
                'elevation_resolved': False, 'point_kind': 'acoustic_intensity_feature',
                'cell_size_m': self.cell_size, 'capacity': self.max_points,
                'points_fan_m': [list(p) for p in self.cells.values()]}

    def ply(self):
        points = self.snapshot()['points_fan_m']
        header = ['ply', 'format ascii 1.0',
                  'comment scope sensor_fan_plane',
                  'comment elevation_resolved false; z=0 is a projection convention',
                  'comment point_kind acoustic_intensity_feature; obstacle identity unknown',
                  f'comment segment {self.segment}',
                  f'element vertex {len(points)}',
                  'property float x', 'property float y', 'property float z', 'end_header']
        return '\n'.join(header + [f'{x:.9g} {y:.9g} 0' for x, y in points]) + '\n'
