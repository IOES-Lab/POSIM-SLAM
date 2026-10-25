"""Bounded camera/sonar event ordering on original sensor timestamps.

Each input stream must arrive in increasing timestamp order. Cross-stream
arrival order may differ. A stream's latest timestamp establishes that all its
earlier packets have arrived (or were lost); no old measurement is interpolated
or reused. The caller separately enforces wall-clock liveness and IMU coverage.
"""


class SensorEvents:
    def __init__(self, *, capacity=16, state_clock='camera', dvl=False):
        if type(capacity) is not int or not 2 <= capacity <= 256:
            raise ValueError('sensor_events_capacity')
        self.capacity = capacity
        if state_clock not in ('camera', 'sonar'):
            raise ValueError('sensor_events_state_clock')
        self.state_clock = state_clock
        if type(dvl) is not bool:
            raise ValueError('sensor_events_dvl_selection')
        self.dvl_enabled = dvl
        self.streams = ('sonar',) if state_clock == 'sonar' else ('camera', 'sonar')
        if dvl:
            self.streams += ('dvl',)
        self.pending = {'camera': {}, 'sonar': {}}
        if dvl:
            self.pending['dvl'] = {}
        self.latest = {**dict.fromkeys(self.pending, 0), 'imu': 0}
        self.closed = set()
        self.awaiting = None
        self.last = 0
        self.started = False
        self.auxiliary_used = False
        self.fault = None
        self.peak_pending = 0
        self.before_camera = 0
        self.auxiliary_skipped = 0

    def _healthy(self):
        if self.fault:
            raise RuntimeError('sensor_events_segment_failed:' + self.fault)

    def _fail(self, reason):
        self.fault = reason
        # Release potentially large image payloads after a terminal fault.
        for queue in self.pending.values():
            queue.clear()
        self.awaiting = None
        raise ValueError(reason)

    def push(self, kind, stamp_ns, payload=None):
        """Supply one selected sensor payload or IMU clock.

        IMU samples stay in the caller's bounded integration buffer. Payloads
        must be validated, size-bounded and immutable while queued by callers.
        A DVL loss-status payload advances its stream clock but must not be
        submitted as a native velocity factor. Silence is not a loss status.
        """
        self._healthy()
        if kind not in (*self.streams, 'imu') or kind in self.closed:
            self._fail('sensor_events_input_kind_or_closed')
        if type(stamp_ns) is not int or not self.latest[kind] < stamp_ns < 2**63:
            self._fail('sensor_events_input_clock')
        if kind != 'imu':
            if payload is None:
                self._fail('sensor_events_payload_required')
            if sum(map(len, self.pending.values())) >= self.capacity:
                self._fail('sensor_events_backlog')
            self.pending[kind][stamp_ns] = payload
            self.peak_pending = max(self.peak_pending, sum(map(len, self.pending.values())))
        self.latest[kind] = stamp_ns

    def finish(self, kind):
        """Declare actual end of a finite recording, never a live timeout."""
        self._healthy()
        if kind not in self.streams or kind in self.closed:
            self._fail('sensor_events_finish_kind_or_closed')
        self.closed.add(kind)

    def next_event(self):
        """Release at most one event, requiring its exact native state ack.

        Both streams must have advanced through the event timestamp, and raw
        IMU must extend strictly beyond it. Equal-time camera/sonar packets form
        one event. The native worker accepts at most one auxiliary sonar state
        between consecutive images; excess sonar packets are explicitly counted
        and omitted, never shifted onto a camera timestamp.
        Sonar clock requires only sonar and future IMU; no camera heartbeat or
        auxiliary decimation is applied. Clock selection is fixed per segment.
        Selecting DVL adds its watermark and exact-time payload, and disables
        auxiliary decimation. The native DVL state-clock capability is required.
        """
        self._healthy()
        if self.awaiting is not None:
            return None
        watermark = min([self.latest['imu'] - 1] + [
            self.latest[kind] for kind in self.streams if kind not in self.closed])
        while any(self.pending.values()):
            stamp = min(t for queue in self.pending.values() for t in queue)
            if stamp > watermark:
                return None
            camera = self.pending['camera'].pop(stamp, None)
            sonar = self.pending['sonar'].pop(stamp, None)
            dvl = self.pending['dvl'].pop(stamp, None) if self.dvl_enabled else None
            if camera is None and self.state_clock == 'camera':
                if not self.started:
                    self.before_camera += 1
                    continue
                if self.auxiliary_used and not self.dvl_enabled:
                    self.auxiliary_skipped += 1
                    continue
            if stamp <= self.last:
                self._fail('sensor_events_output_clock')
            self.awaiting = dict(stamp_ns=stamp, camera=camera, sonar=sonar)
            if self.dvl_enabled:
                self.awaiting['dvl'] = dvl
            return dict(self.awaiting)
        return None

    def acknowledge(self, stamp_ns):
        """Call only after SonarFusion.observe has validated the native state."""
        self._healthy()
        if (self.awaiting is None or type(stamp_ns) is not int
                or stamp_ns != self.awaiting['stamp_ns']):
            self._fail('sensor_events_unexpected_ack')
        if self.state_clock == 'sonar' or self.awaiting['camera'] is not None:
            self.started = True
            self.auxiliary_used = False
        else:
            self.auxiliary_used = True
        self.last = stamp_ns
        self.awaiting = None
