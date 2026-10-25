"""Bounded acoustic hypothesis archive in the native estimator's local frame.

Native factors can retire and rebind one source track. Archive by source ID,
not native factor ID, so rebinding does not inflate the point count. An accepted
joint step is necessary for admission; it does not establish physical identity,
joint uncertainty, or map accuracy. Keep this separate from visual landmarks.
"""
from collections import OrderedDict
import math


class SonarPointArchive:
    def __init__(self, capacity=100000):
        if type(capacity) is not int or not 1 <= capacity <= 100000:
            raise ValueError('invalid_acoustic_archive_capacity')
        self.capacity = capacity
        self.entries = OrderedDict()
        self.stamp = 0
        self.evicted = 0

    def update(self, stamp, report, sources):
        if type(stamp) is not int or stamp <= self.stamp:
            raise ValueError('invalid_acoustic_archive_clock')
        rows = report['point_estimates']
        accepted = report['point_update_accepted']
        if not isinstance(rows, list) or len(rows) > 16 or type(accepted) is not bool:
            raise ValueError('invalid_acoustic_point_report')
        staged = []
        seen = set()
        for row in rows:
            ident, point = row['id'], row['point']
            source = sources.get(ident)
            if (type(ident) is not int or ident < 0 or type(source) is not int
                    or source < 0 or source in seen or not isinstance(point, list)
                    or len(point) != 3 or any(type(v) not in (int, float)
                        or not math.isfinite(v) for v in point)):
                raise ValueError('invalid_acoustic_point_identity_or_coordinates')
            seen.add(source)
            staged.append((source, dict(source_id=source, native_id=ident,
                                       stamp_ns=stamp, point=list(point))))
        # Validate the entire update before changing any retained coordinates.
        if accepted:
            for source, value in staged:
                self.entries[source] = value
                self.entries.move_to_end(source)
                if len(self.entries) > self.capacity:
                    self.entries.popitem(last=False)
                    self.evicted += 1
        self.stamp = stamp

    def snapshot(self):
        return dict(scope='local_acoustic_track_hypotheses', map_qualified=False,
                    physical_identity_verified=False, joint_uncertainty_available=False,
                    stamp_ns=self.stamp, evicted=self.evicted,
                    points=[dict(row, point=list(row['point']))
                            for row in self.entries.values()])

    def ply(self):
        header = ['ply', 'format ascii 1.0',
                  'comment scope local_acoustic_track_hypotheses',
                  'comment map_qualified false; physical identity and joint uncertainty unverified',
                  f'comment stamp_ns {self.stamp}',
                  f'element vertex {len(self.entries)}',
                  'property double x', 'property double y', 'property double z', 'end_header']
        return '\n'.join(header + [' '.join(format(v, '.17g') for v in row['point'])
                                  for row in self.entries.values()]) + '\n'
