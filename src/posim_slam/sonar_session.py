"""Bounded, camera-free streaming ownership of one native sonar/IMU segment.

The caller supplies calibrated sensor data, owns wall-time recovery and labels
the map frame. No ROS, application, simulator pose or camera input is accepted.
"""
from collections import deque
import json
import time
from pathlib import Path
import numpy as np

from .acoustic_tracks import AcousticTracks
from .auxiliary_inputs import bracketed_gyro, validate_auxiliary_ack, fused_recent
from .dvl_packet import encode_dvl
from .gravity_seed import gravity_seed
from .inertial import ImuWindow
from .live_client import LiveClient
from .sensor_events import SensorEvents
from .sonar_fusion import SonarFusion
from .sonar_input import validate_frame


class SonarSession:
    def __init__(self, binary, calibration, log, *, range_sigma, azimuth_sigma,
                 dvl_gyro_sigma=None, clock=time.monotonic, client_factory=LiveClient):
        self.config = json.loads(Path(calibration).read_text())
        if self.config.get('state_clock') != 'sonar' or self.config.get('enable_joint_sonar') is not True:
            raise ValueError('camera_free_sonar_configuration_required')
        self.dvl = self.config.get('enable_joint_dvl', False)
        # Application sonar SLAM requires pressure. Explicit False is retained for
        # independently labelled research ablations only.
        self.pressure = self.config.get('enable_joint_pressure', True)
        if type(self.pressure) is not bool or type(self.dvl) is not bool:
            raise ValueError('invalid_sonar_auxiliary_selection')
        if self.pressure and self.config.get('enable_joint_pressure') is not True:
            raise ValueError('sonar_pressure_calibration_required')
        if self.dvl != (dvl_gyro_sigma is not None):
            raise ValueError('selected_dvl_requires_gyro_uncertainty')
        if dvl_gyro_sigma is not None and (not np.isfinite(dvl_gyro_sigma) or not 0 < dvl_gyro_sigma <= .5):
            raise ValueError('explicit_dvl_gyro_uncertainty_required')
        self.gyro_sigma = dvl_gyro_sigma
        self.clock = clock
        self.binary, self.calibration, self.log = binary, calibration, log
        self.client_factory = client_factory
        self.client = None
        self.events = SensorEvents(capacity=32, state_clock='sonar', dvl=self.dvl, pressure=self.pressure)
        self.fusion = SonarFusion(range_sigma, azimuth_sigma)
        self.tracks = AcousticTracks()
        self.samples = deque(maxlen=4096)
        self.latest = dict(imu=0, sonar=0, dvl=0, pressure=0)
        self.received = dict.fromkeys(self.latest, None)
        self.valid_auxiliary = dict(dvl=False, pressure=False)
        self.counts = dict.fromkeys(self.latest, 0)
        self.pending_sonar = None
        self.start_stamp = 0
        self.bootstrapped = False
        self.started_wall = None
        self.last_result_wall = None
        self.last_result = None
        self.states = 0
        self.sent = 0
        self.closed = False
        self.fault = None
        self.expected_auxiliary = {}
        self.optimized = dict(dvl=0, pressure=0)
        self.fused_stamp = dict(dvl=0, pressure=0)
        self.fused_wall = dict(dvl=None, pressure=None)

    def _arrival(self, kind, stamp):
        if self.closed or self.fault:
            raise RuntimeError('sonar_segment_closed_or_failed')
        if type(stamp) is not int or not self.latest[kind] < stamp < 2**63:
            raise ValueError('sonar_session_input_clock')
        self.latest[kind] = stamp
        self.received[kind] = self.clock()
        self.counts[kind] += 1

    def imu(self, stamp, gyro, accel, covariance):
        values = np.asarray([*gyro, *accel, *covariance], dtype=float)
        if (values.shape != (15,) or not np.isfinite(values).all()
                or np.linalg.norm(gyro) > 35 or np.linalg.norm(accel) > 200):
            raise ValueError('invalid_raw_imu')
        if self.latest['imu'] and stamp-self.latest['imu'] > 20_000_001:
            raise ValueError('imu_gap_or_reset')
        self._arrival('imu', stamp)
        self.samples.append((stamp, list(gyro), list(accel), list(covariance)))
        if self.start_stamp:
            self.events.push('imu', stamp)
        if self.bootstrapped:
            self.client.imu(stamp, gyro, accel)

    def sonar(self, frame):
        validate_frame(frame)
        stamp = int(frame['stamp'][0])*10**9+int(frame['stamp'][1])
        self._arrival('sonar', stamp)
        frame = {k: np.array(frame[k], copy=True) for k in ('stamp', 'intensity_db', 'ranges', 'beam_directions')}
        if self.start_stamp:
            self.events.push('sonar', stamp, frame)
        else:
            # Startup retains only the latest frame, not an unbounded backlog.
            self.pending_sonar = frame

    def auxiliary(self, kind, stamp, payload):
        if kind not in self.events.auxiliary_streams:
            raise ValueError('unselected_sonar_auxiliary_sensor')
        if type(payload.get('valid')) is not bool:
            raise ValueError('auxiliary_validity_required')
        self._arrival(kind, stamp)
        self.valid_auxiliary[kind] = payload['valid']
        if self.start_stamp and stamp >= self.start_stamp:
            self.events.push(kind, stamp, payload)

    def _bootstrap(self, stamp):
        if not self.samples or self.samples[0][0] > stamp-10**9 or self.samples[-1][0] <= stamp:
            raise ValueError('bootstrap_imu_history_missing')
        window = ImuWindow(max_samples=4096, max_gap=.020000001)
        for t, gyro, accel, _ in self.samples:
            window.add(t/1e9, gyro, accel)
        seed = gravity_seed(window, stamp/1e9-1, stamp/1e9, gyro_bias=[0, 0, 0], accel_bias=[0, 0, 0])
        self.client.gravity_seed(stamp, seed['direction_imu'])
        for t, gyro, accel, _ in self.samples:
            if t >= stamp-200_000_000:
                self.client.imu(t, gyro, accel)
        self.bootstrapped = True

    def tick(self):
        """Advance at most one original event; return newly acknowledged poses."""
        if self.closed or self.fault:
            raise RuntimeError('sonar_segment_closed_or_failed')
        try:
            return self._tick()
        except (ValueError, RuntimeError, OSError, KeyError) as error:
            self.fault = str(error)
            raise

    def _tick(self):
        now = self.clock()
        if self.client is None:
            # Wait for the required depth input before allocating a worker.
            # A silent pressure source must not cause repeated empty restarts.
            if self.pressure and (not self.valid_auxiliary['pressure']
                    or self.received['pressure'] is None
                    or now-self.received['pressure'] >= 2):
                return []
            if (self.pending_sonar is None or not self.samples
                    or self.samples[0][0] > self.latest['sonar']-10**9
                    or self.samples[-1][0] <= self.latest['sonar']):
                return []
            self.client = self.client_factory(self.binary, self.calibration, self.log)
            self.started_wall = now
        for kind in ('imu', 'sonar', *self.events.auxiliary_streams):
            received = self.received[kind]
            if now-self.started_wall > 3 and (received is None or now-received > 2):
                raise RuntimeError(kind+'_stale')
        results = []
        for value in self.client.poll():
            if value['type'] == 'ready':
                if (value.get('state_clock') != 'sonar' or value.get('joint_sonar_enabled') is not True
                        or value.get('sonar_clock_transport') != 'ordered_sonar_state_v1'):
                    raise RuntimeError('worker_sonar_clock_contract')
                continue
            if value['type'] != 'state' or value.get('state_clock') != 'sonar':
                raise RuntimeError('unexpected_camera_free_worker_output')
            included = validate_auxiliary_ack(value, {kind:self.expected_auxiliary.get(kind,False)
                                                       for kind in ('dvl','pressure')})
            self.fusion.observe(value)
            self.events.acknowledge(value['stamp_ns'])
            for kind in included:
                self.optimized[kind] += 1
                self.fused_stamp[kind] = value['stamp_ns']
                self.fused_wall[kind] = now
            self.states += 1
            self.last_result = value
            self.last_result_wall = now
            results.append(value)
        if not self.client.ready.is_set():
            if now-self.started_wall > 10:
                raise RuntimeError('worker_start_timeout')
            return results
        if not self.start_stamp:
            self.start_stamp = self.latest['sonar']
            self.events.push('imu', self.latest['imu'])
            self.events.push('sonar', self.start_stamp, self.pending_sonar)
            self.pending_sonar = None
        event = self.events.next_event()
        if event:
            stamp = event['stamp_ns']
            if not self.bootstrapped:
                self._bootstrap(stamp)
            measured = None
            if event['sonar'] is not None:
                measured = self.tracks.push(event['sonar'])
                measured['stamp_ns'] = stamp
            updates = self.fusion.prepare(stamp, measured)
            if updates:
                self.client.sonar_snapshot(stamp, updates)
            dvl = event.get('dvl')
            self.expected_auxiliary=dict(dvl=bool(dvl and dvl['valid']),
                                         pressure=bool(event.get('pressure') and event['pressure']['valid']))
            if dvl and dvl['valid']:
                gyro = bracketed_gyro([(t, g, c) for t, g, _, c in self.samples], stamp, model_sigma=self.gyro_sigma)
                packet = dict(stamp_ns=stamp, velocity=dvl['velocity'], covariance=dvl['covariance'],
                              gyro=gyro['gyro'], gyro_stamp_ns=stamp, gyro_covariance=gyro['covariance'],
                              bottom_locked=True, locked_beams=dvl['locked_beams'])
                encode_dvl(**packet)
                self.client.dvl(**packet)
            pressure = event.get('pressure')
            if pressure and pressure['valid']:
                if pressure['packet']['stamp_ns'] != stamp:
                    raise ValueError('pressure_original_clock_required')
                self.client.pressure(**pressure['packet'])
            self.client.sonar_tick(stamp)
            self.sent += 1
        if now-(self.last_result_wall or self.started_wall) > 10:
            raise RuntimeError('sonar_worker_or_ordering_stalled')
        return results

    def auxiliary_fresh(self, kind, *, max_age_s=2.):
        """Fresh acknowledged fusion, independently of incoming sensor traffic."""
        if kind not in self.optimized or not np.isfinite(max_age_s) or max_age_s <= 0:
            raise ValueError('invalid_auxiliary_freshness_request')
        return bool(not self.closed and not self.fault and self.last_result is not None
                    and fused_recent(self.last_result['stamp_ns'],self.fused_stamp[kind],
                                     self.fused_wall[kind],self.clock(),max_age_s=max_age_s))

    def close(self):
        self.closed = True
        return self.client.close() if self.client else None
