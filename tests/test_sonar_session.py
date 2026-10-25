import json
from pathlib import Path
import tempfile
import threading
import unittest
import numpy as np
from posim_slam.sonar_session import SonarSession
from test_acoustic_flow import frame


class Worker:
    def __init__(self, *args):
        self.ready = threading.Event(); self.ready.set()
        self.outputs = [dict(type='ready', state_clock='sonar', joint_sonar_enabled=True,
                             sonar_clock_transport='ordered_sonar_state_v1')]
        self.seeds = []; self.imus = []; self.ticks = []; self.closed = False

    def poll(self):
        result, self.outputs = self.outputs, []
        return result

    def gravity_seed(self, stamp, direction): self.seeds.append((stamp, list(direction)))
    def imu(self, stamp, gyro, accel): self.imus.append(stamp)
    def sonar_tick(self, stamp): self.ticks.append(stamp)
    def close(self): self.closed = True


class SonarSessionTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory(); self.addCleanup(self.directory.cleanup)
        config = Path(self.directory.name)/'calibration.json'
        config.write_text(json.dumps(dict(state_clock='sonar', enable_joint_sonar=True)))
        self.now = 0.
        self.session = SonarSession('worker', config, None, range_sigma=.02, azimuth_sigma=.01,
                                    clock=lambda: self.now, client_factory=Worker)
        self.addCleanup(self.session.close)

    def feed(self):
        for t in range(1_000_000_000, 2_020_000_000, 10_000_000):
            self.session.imu(t, [0, 0, 0], [0, 0, 9.80665], [0]*9)
        image = frame(0); image['stamp'] = np.array([2, 0])
        self.session.sonar(image)

    def state(self, stamp=2_000_000_000):
        return dict(type='state', state_clock='sonar', stamp_ns=stamp, sonar=dict(
            retained_pose_stamps_ns=[stamp], active_track_ids=[], point_estimates=[],
            point_update_accepted=False, owner_received=False))

    def test_bootstrap_requires_future_imu_and_uses_no_camera_or_pose(self):
        self.assertEqual(self.session.tick(), [])
        self.assertIsNone(self.session.client)
        self.feed(); self.assertEqual(self.session.tick(), [])
        worker = self.session.client
        self.assertEqual(worker.ticks, [2_000_000_000])
        np.testing.assert_allclose(worker.seeds[0][1], [0, 0, 1])
        self.assertGreater(worker.imus[-1], worker.ticks[0])
        worker.outputs = [self.state()]
        self.assertEqual(len(self.session.tick()), 1)
        self.assertEqual(self.session.states, 1)
        self.assertEqual(self.session.fusion.snapshot()['points'], [])
        self.assertEqual(worker.ticks, [2_000_000_000])

    def test_wrong_auxiliary_ack_fails_before_publication(self):
        self.feed(); self.session.tick()
        state = self.state(); state['dvl'] = dict(owner_received=True)
        self.session.client.outputs = [state]
        with self.assertRaisesRegex(RuntimeError, 'auxiliary_ack'):
            self.session.tick()
        self.assertEqual(self.session.states, 0)
        self.assertEqual(self.session.fusion.archive.stamp, 0)
        with self.assertRaisesRegex(RuntimeError, 'failed'):
            self.session.tick()

    def test_camera_output_cannot_enter_the_sonar_clock(self):
        self.feed(); self.session.tick()
        self.session.client.outputs = [dict(type='map', state_clock='camera')]
        with self.assertRaisesRegex(RuntimeError, 'unexpected_camera_free'):
            self.session.tick()
        self.assertIsNone(self.session.last_result)
        self.assertEqual(self.session.fusion.archive.stamp, 0)

    def test_wrong_state_time_is_terminal_and_cannot_acknowledge_pending_event(self):
        self.feed(); self.session.tick()
        self.session.client.outputs = [self.state(2_000_000_001)]
        with self.assertRaisesRegex(ValueError, 'unexpected_state'):
            self.session.tick()
        self.assertEqual(self.session.states, 0)
        self.assertIsNone(self.session.last_result)

    def test_selected_input_timeout_is_terminal(self):
        self.feed(); self.session.tick(); self.now = 4
        with self.assertRaisesRegex(RuntimeError, 'imu_stale'):
            self.session.tick()
        self.session.close()
        self.assertTrue(self.session.client.closed)


if __name__ == '__main__': unittest.main()
