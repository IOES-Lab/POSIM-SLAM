"""Stop-and-wait acoustic snapshot lifecycle for one native estimator segment.

Inputs are associated tracks with explicit full covariance, not raw detections.
This module neither invents associations nor assigns measurement uncertainty.
"""
from copy import deepcopy
import numpy as np
from .sonar_packet import encode_snapshot


class SonarSnapshots:
    def __init__(self):
        self.stamp = 0
        self.retained = set()
        self.active = set()
        self.bindings = {}
        self.next_id = 0
        self.pending = None
        self.blocked = set()

    def observe(self, state):
        """Consume each native state in order before preparing the next event."""
        stamp = state['stamp_ns']
        report = state['sonar']
        retained = report['retained_pose_stamps_ns']
        active = report['active_track_ids']
        if (type(stamp) is not int or stamp <= self.stamp or
                any(type(t) is not int or not 0 < t <= stamp for t in retained) or
                retained != sorted(set(retained)) or stamp not in retained or
                any(type(i) is not int or i < 0 for i in active) or
                active != sorted(set(active))):
            raise ValueError('invalid_sonar_state_report')
        rejected = report.get('geometry_rejected_track_ids', [])
        if (not isinstance(rejected, list) or any(type(i) is not int or i < 0 for i in rejected) or
                rejected != sorted(set(rejected))):
            raise ValueError('invalid_sonar_rejection_report')
        blocked = self.blocked.copy()
        if self.pending is not None:
            expected, bindings, next_id, submitted = self.pending
            if stamp != expected or not report.get('owner_received'):
                raise ValueError('missing_sonar_snapshot_ack')
            if not set(rejected) <= submitted:raise ValueError('unknown_rejected_sonar_track')
            bindings = deepcopy(bindings)
            for key, track in list(bindings.items()):
                if track['id'] not in rejected:continue
                old = self.bindings.get(key)
                if old is not None and old['id'] == track['id']:
                    bindings[key] = old
                    blocked.add(key)  # Freeze this association until its owned factors retire.
                else:
                    del bindings[key]
        else:
            if rejected:raise ValueError('unexpected_sonar_rejection_report')
            bindings, next_id = self.bindings, self.next_id
        known = {track['id'] for track in bindings.values()}
        if not set(active) <= known:
            raise ValueError('unknown_native_sonar_track')
        self.bindings = {key: value for key, value in bindings.items() if value['id'] in active}
        self.blocked = blocked.intersection(self.bindings)
        self.next_id = next_id
        self.stamp, self.retained, self.active = stamp, set(retained), set(active)
        self.pending = None

    def prepare(self, stamp, tracks):
        """Prepare full histories keyed by stable source IDs within this segment.

        Call after the preceding state response. New candidates lose observations
        whose poses expired; covariance uses the matching principal submatrix.
        Existing native history is immutable and may only append one observation.
        Return [] when no candidate is ready; still send the image or sonar tick.
        """
        if self.pending is not None:
            raise ValueError('sonar_snapshot_ack_pending')
        if type(stamp) is not int or stamp <= self.stamp or not self.stamp:
            raise ValueError('sonar_snapshot_requires_prior_state')
        if not isinstance(tracks, (list, tuple)) or len(tracks) > 300:
            raise ValueError('invalid_sonar_candidates')
        keys = [track['id'] for track in tracks]
        if any(type(key) is not int or key < 0 for key in keys) or len(set(keys)) != len(keys):
            raise ValueError('invalid_sonar_candidate_ids')
        bindings = deepcopy(self.bindings)
        next_id = self.next_id
        updates = []
        for source in sorted(tracks, key=lambda track: track['id']):
            # Validate the complete supplied covariance before selecting a subset.
            encode_snapshot(stamp, [source])
            if source['id'] in self.blocked:continue
            observations = source['observations']
            covariance = np.asarray(source['covariance'], dtype=float)
            old = bindings.get(source['id'])
            if old is not None:
                history = old['observations']
                # Initial registration can retain noncontiguous keyframe poses.
                # Keep exactly the native-owned timestamps, not every source
                # observation after the first one (which reintroduces holes).
                by_stamp = {obs['stamp_ns']: i for i, obs in enumerate(observations)}
                owned = [obs['stamp_ns'] for obs in history]
                additions = [obs['stamp_ns'] for obs in observations
                             if obs['stamp_ns'] > owned[-1]]
                if any(t not in by_stamp for t in owned) or additions != [stamp]:
                    raise ValueError('changed_sonar_candidate_history')
                indices = [by_stamp[t] for t in owned] + [by_stamp[stamp]]
                selected = [observations[i] for i in indices]
                if selected[:-1] != history:
                    raise ValueError('changed_sonar_candidate_history')
            else:
                if len(bindings) >= 16:
                    continue
                indices = [i for i, obs in enumerate(observations)
                           if obs['stamp_ns'] in self.retained or obs['stamp_ns'] == stamp]
                selected = [observations[i] for i in indices]
                if len(selected) < 3:
                    continue
            axes = [axis for i in indices for axis in (2*i, 2*i+1)]
            noise = covariance[np.ix_(axes, axes)]
            if old is not None and not np.array_equal(noise[:-2, :-2], old['covariance']):
                raise ValueError('changed_sonar_candidate_covariance')
            ident = old['id'] if old is not None else next_id
            if old is None:
                next_id += 1
            update = dict(id=ident, observations=deepcopy(selected), covariance=noise.tolist())
            bindings[source['id']] = update
            updates.append(update)
        updates.sort(key=lambda track: track['id'])
        if updates:
            encode_snapshot(stamp, updates)
            self.pending = (stamp, bindings, next_id, {track["id"] for track in updates})
        return deepcopy(updates)
