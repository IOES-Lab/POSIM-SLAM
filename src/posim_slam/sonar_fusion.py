"""Causal sonar-factor ownership for one independent native estimator segment.

The caller supplies measured frontend tracks and orders camera/sonar events on
their original sensor clock. One state acknowledgement is required per event.
No ROS, application, simulator truth, filesystem or retrospective input is used.
Diagonal noise remains an explicit diagnostic assumption, not calibration.
"""
from copy import deepcopy
from .sonar_candidates import SonarCandidates
from .sonar_snapshots import SonarSnapshots
from .sonar_point_archive import SonarPointArchive


class SonarFusion:
    def __init__(self, range_sigma, azimuth_sigma, *, archive_capacity=100000):
        self.candidates = SonarCandidates(range_sigma, azimuth_sigma)
        self.owner = SonarSnapshots()
        self.archive = SonarPointArchive(archive_capacity)
        self.awaiting = None
        self.expected_snapshot = False
        self.fault = None
        self.snapshots = 0
        self.track_updates = 0
        self.states = 0

    def _healthy(self):
        if self.fault is not None:
            raise RuntimeError('sonar_fusion_segment_failed:' + self.fault)

    def prepare(self, stamp_ns, frame=None):
        """Return snapshots to send immediately before this image/sonar tick.

        ``frame`` is the AcousticTracks output at exactly this event timestamp,
        or None for camera-only events. Advance only after observe acknowledges
        this timestamp, including events that did not submit acoustic factors.
        """
        self._healthy()
        try:
            if self.awaiting is not None:
                raise ValueError('sonar_fusion_state_ack_pending')
            if type(stamp_ns) is not int or not self.owner.stamp < stamp_ns < 2**63:
                raise ValueError('sonar_fusion_event_clock')
            updates = []
            if frame is not None:
                if type(frame.get('stamp_ns')) is not int or frame['stamp_ns'] != stamp_ns:
                    raise ValueError('sonar_fusion_exact_frame_required')
                candidates = self.candidates.push(frame, retained_stamps=self.owner.retained)
                if self.owner.stamp:
                    updates = self.owner.prepare(stamp_ns, candidates)
            self.awaiting = stamp_ns
            self.expected_snapshot = bool(updates)
            self.snapshots += int(bool(updates))
            self.track_updates += len(updates)
            return updates
        except (ValueError, TypeError, KeyError, AttributeError) as error:
            self.fault = str(error)
            raise

    def observe(self, state):
        """Commit validated ownership and points for the exact pending event.

        Capture pending source bindings before native retirement. Validate the
        acknowledgement before allowing any new point into the bounded archive.
        Invalid input fails the segment; the caller must start a new instance.
        """
        self._healthy()
        try:
            stamp_ns = state['stamp_ns']
            if (state.get('type') != 'state' or type(stamp_ns) is not int
                    or self.awaiting is None or stamp_ns != self.awaiting):
                raise ValueError('sonar_fusion_unexpected_state')
            report = state['sonar']
            # Native reports omit owner_received on events without a snapshot.
            if report.get('owner_received', False) is not self.expected_snapshot:
                raise ValueError('sonar_fusion_snapshot_ack_mismatch')
            sources = self.owner.point_sources()
            # Only the bounded factor owner is staged. Never copy the full map.
            owner = deepcopy(self.owner)
            owner.observe(state)
            self.archive.update(stamp_ns, report, sources)
            self.owner = owner
            self.awaiting = None
            self.expected_snapshot = False
            self.states += 1
        except (ValueError, TypeError, KeyError, AttributeError) as error:
            self.fault = str(error)
            raise

    def snapshot(self):
        self._healthy()
        return self.archive.snapshot()

    def ply(self):
        self._healthy()
        return self.archive.ply()
