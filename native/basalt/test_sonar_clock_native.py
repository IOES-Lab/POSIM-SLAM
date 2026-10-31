"""Native camera-free event ownership, acoustic optimization and bounded state tests.

Analytic IMU and range/bearing observations exercise the worker, not image
association or terrain reconstruction. No poses, point seeds or pixels cross
the worker boundary. Initial velocity is zero in this synthetic fixture.
"""
import json
import math
import os
import struct
from pathlib import Path
import tempfile
import time
import unittest

import numpy as np
from posim_slam.live_client import LiveClient


@unittest.skipUnless(os.environ.get('POSIM_SLAM_STREAM_BINARY'), 'Native worker binary required')
class SonarClockTests(unittest.TestCase):
    def worker(self, root, **settings):
        config = root / 'calibration.json'
        values = dict(
            schema='posim-slam-sonar-inertial-recording-1', state_clock='sonar',
            orientation_used=False, enable_joint_sonar=True, require_gravity_seed=True,
            max_recent_states=16, max_sonar_gap_s=.5, imu_rate_hz=100, gyro_noise_density=.001,
            accel_noise_density=.02, gyro_bias_random_walk=.0001,
            accel_bias_random_walk=.001, gravity_m_s2=9.80665, max_imu_gap_s=.02)
        values.update(settings)
        config.write_text(json.dumps(values))
        (root / 'sonar-calibration.json').write_text(json.dumps(dict(
            schema='posim-slam-sonar-calibration-1', imu_to_sonar=[0, 0, 0],
            sonar_to_imu=np.eye(3).tolist(), range_min=.2, range_max=10.,
            horizontal_fov=2.1, vertical_fov=.7)))
        log = (root / 'worker.log').open('w')
        self.addCleanup(log.close)
        client = LiveClient(os.environ['POSIM_SLAM_STREAM_BINARY'], config, log)
        self.addCleanup(client.close)
        ready = self.receive(client, 'ready')
        self.assertEqual(ready['state_clock'], 'sonar')
        self.assertEqual(ready['raw_resolution'], [0, 0])
        self.assertIsNone(ready['map_export'])
        self.assertTrue(client.sonar_clock_transport)
        return client

    def receive(self, client, kind, timeout=10):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            for value in client.poll():
                if value['type'] == kind:
                    return value
                self.assertNotEqual(value['type'], 'map', 'Sonar mode emitted a visual map')
            time.sleep(.002)
        self.fail('worker reply timeout: ' + kind)

    def test_moving_seed_rotates_velocity_and_retains_sensor_biases(self):
        with tempfile.TemporaryDirectory() as folder:
            client = self.worker(Path(folder))
            initial = 1_000_000_000
            velocity = np.array([.7, -.2, .1])
            bg, ba = np.array([.01, -.02, .03]), np.array([.1, -.2, .05])
            client.inertial_seed(initial, [0, 0, -1], velocity, bg, ba)
            next_imu = initial
            first = None
            for index in range(3):
                stamp = initial+index*200_000_000
                while next_imu <= stamp+10_000_000:
                    client.imu(next_imu, bg, np.array([0, 0, -9.80665])+ba)
                    next_imu += 10_000_000
                client.sonar_tick(stamp)
                state = self.receive(client, 'state')
                if first is None:
                    first = state
                    x,y,z,w = state['orientation_xyzw']
                    rotation = np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                        [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                        [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
                np.testing.assert_allclose(state['velocity'], rotation @ velocity, atol=1e-7)
                np.testing.assert_allclose(state['position'], index*.2*(rotation @ velocity), atol=1e-7)
                np.testing.assert_allclose(state['initialization_window']['gyro_bias'], bg, atol=1e-7)
                np.testing.assert_allclose(state['initialization_window']['accel_bias'], ba, atol=1e-7)

    def test_native_rejects_nonfinite_and_duplicate_state_seed(self):
        for malformed in (True, False):
            with self.subTest(malformed=malformed), tempfile.TemporaryDirectory() as folder:
                client = self.worker(Path(folder))
                values = [0., 0., 1., 0., 0., 0., 0., 0., 0., 0., 0., 0.]
                if malformed:values[3] = float('nan')
                client.send(13, struct.pack('<q12d', 1_000_000_000, *values))
                if not malformed:client.inertial_seed(1_000_000_000,[0,0,1],[0,0,0],[0,0,0],[0,0,0])
                reason = 'invalid_inertial_seed' if malformed else 'inertial_seed_contract'
                with self.assertRaisesRegex(RuntimeError, reason):self.receive(client, 'state')

    def test_sampled_range_beyond_clip_keeps_native_state_processing(self):
        # A source-grid coordinate is noisy evidence; physical point support
        # remains bounded by the ten-metre calibration. This synthetic stream
        # supplies no map-point seed and does not qualify a reconstruction.
        with tempfile.TemporaryDirectory() as folder:
            client=self.worker(Path(folder));initial=1_000_000_000;next_imu=initial
            client.gravity_seed(initial,[0,0,1]);observations=[];states=[]
            spacing=1500./(2*29900.)
            for index in range(12):
                stamp=initial+index*200_000_000;t=index*.2
                while next_imu<=stamp+10_000_000:
                    client.imu(next_imu,[0,0,0],[.02,0,9.80665]);next_imu+=10_000_000
                distance=9.999-.01*t*t
                measured=round(distance/spacing)*spacing
                observations.append(dict(stamp_ns=stamp,range_m=measured,azimuth_rad=0.))
                if index>=2:
                    client.sonar_snapshot(stamp,[dict(id=17,observations=list(observations),
                        covariance=np.diag([.025**2,.008**2]*len(observations)))])
                client.sonar_tick(stamp);state=self.receive(client,'state')
                self.assertFalse(state['map_qualified']);self.assertTrue(np.isfinite(state['position']).all())
                states.append(state)
            self.assertGreater(observations[0]['range_m'],10.)
            self.assertEqual(states[-1]['stamp_ns'],initial+2_200_000_000)

    def test_acoustic_states_marginalize_without_any_camera(self, with_dvl=False):
        with tempfile.TemporaryDirectory() as folder:
            client = self.worker(Path(folder))
            initial = 1_000_000_000
            client.gravity_seed(initial, [0, 0, 1])
            next_imu = initial
            points = np.array([[4, .5, .2], [3, -.6, .3], [5, 1, -.2], [4.5, -1, -.1]])
            observations = [[] for _ in points]
            active = {}
            retained = set()
            next_id = 0
            states = []
            acoustic_states = 0
            max_after_bootstrap = 0
            had_acoustic = False
            for index in range(61):
                stamp = initial + index * 200_000_000
                t = index * .2
                truth = np.array([.012*t*t, .2*(1-math.cos(.7*t)), .15*(1-math.cos(.9*t))])
                while next_imu <= stamp + 10_000_000:
                    u = (next_imu-initial)*1e-9
                    client.imu(next_imu, [0, 0, 0],
                               [.024, .098*math.cos(.7*u), 9.80665+.1215*math.cos(.9*u)])
                    next_imu += 10_000_000
                submitted = {}
                for k, point in enumerate(points):
                    local = point - truth
                    observation = dict(stamp_ns=stamp, range_m=float(np.linalg.norm(local)),
                                       azimuth_rad=math.atan2(local[1], local[0]))
                    observations[k].append(observation)
                    if k in active:
                        ident, old = active[k]
                        history = old + [observation]
                    else:
                        history = [v for v in observations[k] if v['stamp_ns'] in retained or v['stamp_ns']==stamp][-10:]
                        if len(history) < 3:
                            continue
                        ident = next_id
                        next_id += 1
                    noise = np.diag(np.tile([.002**2, .0005**2], len(history)))
                    submitted[k] = dict(id=ident, observations=history, covariance=noise)
                if submitted:
                    client.sonar_snapshot(stamp, sorted(submitted.values(), key=lambda row: row['id']))
                if with_dvl:
                    client.dvl(stamp,[.024*t,.14*math.sin(.7*t),.135*math.sin(.9*t)],np.eye(3)*.0001,
                               gyro=[0,0,0],gyro_stamp_ns=stamp,gyro_covariance=np.eye(3)*.000001,
                               bottom_locked=True,locked_beams=4)
                client.sonar_tick(stamp)
                state = self.receive(client, 'state')
                self.assertEqual(state['stamp_ns'], stamp)
                self.assertEqual(state['state_clock'], 'sonar')
                self.assertFalse(state['map_qualified'])
                if with_dvl:self.assertTrue(state['dvl']['optimization_included'])
                report = state['sonar']
                retained = set(report['retained_pose_stamps_ns'])
                live = set(report['active_track_ids'])
                rejected = set(report.get('geometry_rejected_track_ids', []))
                updated = {}
                for k, row in submitted.items():
                    if row['id'] in live:
                        updated[k] = active[k] if row['id'] in rejected else (row['id'], row['observations'])
                active = updated
                if live:
                    acoustic_states += 1
                    had_acoustic = True
                if had_acoustic:
                    max_after_bootstrap = max(max_after_bootstrap, len(retained))
                states.append((state, truth))
            self.assertGreater(acoustic_states, 20)
            self.assertLessEqual(max_after_bootstrap, 16)
            self.assertGreater(min(retained), initial)
            errors = [np.linalg.norm(np.asarray(state['position'])-truth) for state, truth in states]
            self.assertLess(max(errors), .1)
            client.snapshot(1)
            self.assertEqual(self.receive(client, 'export')['reason'], 'map_unavailable')
            self.assertEqual(client.close(), client.process.returncode)
            self.assertFalse(client.writer.is_alive() or client.reader.is_alive())
            print(json.dumps(dict(camera_packets=0, dvl_enabled=with_dvl, states=len(states), acoustic_states=acoustic_states,
                                  max_retained_states=max_after_bootstrap, max_position_error_m=max(errors),
                                  scope='analytic sensor/ownership fixture; no image or terrain acceptance')))

    def test_no_geometry_exhausts_startup_instead_of_claiming_slam(self):
        with tempfile.TemporaryDirectory() as folder:
            client = self.worker(Path(folder))
            initial = 1_000_000_000
            client.gravity_seed(initial, [0, 0, 1])
            next_imu = initial
            with self.assertRaisesRegex(RuntimeError, 'sonar_initialization_window_exhausted'):
                for index in range(130):
                    stamp = initial + index*200_000_000
                    while next_imu <= stamp+10_000_000:
                        client.imu(next_imu, [0, 0, 0], [0, 0, 9.80665])
                        next_imu += 10_000_000
                    client.sonar_tick(stamp)
                    state = self.receive(client, 'state')
                    self.assertEqual(state['sonar']['active_track_ids'], [])
                    self.assertFalse(state['map_qualified'])

    def test_missing_gravity_seed_rejects_first_tick(self):
        with tempfile.TemporaryDirectory() as folder:
            client = self.worker(Path(folder))
            client.imu(1_000_000_000, [0, 0, 0], [0, 0, 9.80665])
            client.sonar_tick(1_000_000_000)
            with self.assertRaisesRegex(RuntimeError, 'gravity_seed_sensor_stamp'):
                self.receive(client, 'state')

    def test_sonar_clock_rejects_reversal_gap_and_camera_packets(self):
        for case in ('reversal', 'gap', 'camera'):
            with self.subTest(case=case), tempfile.TemporaryDirectory() as folder:
                client = self.worker(Path(folder))
                initial = 1_000_000_000
                client.gravity_seed(initial, [0, 0, 1])
                for stamp in (initial, initial+10_000_000):
                    client.imu(stamp, [0, 0, 0], [0, 0, 9.80665])
                client.sonar_tick(initial)
                self.receive(client, 'state')
                if case == 'camera':
                    # Bypass the client check: the worker independently rejects it.
                    client.send(2, b'')
                    reason = 'camera_packet_in_sonar_clock'
                else:
                    for stamp in range(initial+20_000_000, initial+530_000_000, 10_000_000):
                        client.imu(stamp, [0, 0, 0], [0, 0, 9.80665])
                    client.sonar_tick(initial if case=='reversal' else initial+510_000_000)
                    reason = 'auxiliary_sonar_clock_or_capacity'
                with self.assertRaisesRegex(RuntimeError, reason):
                    self.receive(client, 'state')
                client.close()


if __name__ == '__main__':
    unittest.main()
