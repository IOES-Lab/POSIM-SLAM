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
        self.pressures = []; self.seeds = []; self.imus = []; self.ticks = []; self.closed = False

    def poll(self):
        result, self.outputs = self.outputs, []
        return result

    def gravity_seed(self, stamp, direction): self.seeds.append((stamp, list(direction)))
    def imu(self, stamp, gyro, accel): self.imus.append(stamp)
    def sonar_tick(self, stamp): self.ticks.append(stamp)
    def pressure(self, **packet): self.pressures.append(packet)
    def close(self): self.closed = True


class SonarSessionTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory(); self.addCleanup(self.directory.cleanup)
        config = Path(self.directory.name)/'calibration.json'
        config.write_text(json.dumps(dict(state_clock='sonar', enable_joint_sonar=True, enable_joint_pressure=False)))
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

    def test_raw_imu_waits_with_the_pending_state_instead_of_filling_native_queue(self):
        self.feed(); self.session.tick()
        worker = self.session.client
        initial = list(worker.imus)
        for t in range(2_020_000_000, 5_000_000_000, 10_000_000):
            self.session.imu(t, [0, 0, 0], [0, 0, 9.80665], [0]*9)
        image = frame(0); image['stamp'] = np.array([2, 500_000_000])
        self.session.sonar(image); self.session.tick()
        self.assertEqual(worker.imus, initial)
        worker.outputs = [self.state()]
        self.session.tick()
        self.assertEqual(worker.ticks, [2_000_000_000, 2_500_000_000])
        self.assertEqual(worker.imus, list(range(1_800_000_000, 2_520_000_000, 10_000_000)))

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


    def test_pressure_default_requires_native_calibration_and_ablation_is_explicit(self):
        config=Path(self.directory.name)/'missing-pressure.json'
        config.write_text(json.dumps(dict(state_clock='sonar',enable_joint_sonar=True)))
        with self.assertRaisesRegex(ValueError,'pressure_calibration_required'):
            SonarSession('worker',config,None,range_sigma=.02,azimuth_sigma=.01)

    def pressure_session(self):
        config=Path(self.directory.name)/'pressure.json'
        config.write_text(json.dumps(dict(state_clock='sonar',enable_joint_sonar=True,enable_joint_pressure=True)))
        self.session.close()
        self.session=SonarSession('worker',config,None,range_sigma=.02,azimuth_sigma=.01,
                                  clock=lambda:self.now,client_factory=Worker)
        self.addCleanup(self.session.close)
        self.feed();self.session.tick()
        self.assertIsNone(self.session.client)
        self.session.auxiliary('pressure',1_990_000_000,dict(valid=True,packet=dict(stamp_ns=1_990_000_000,height_m=-2.,variance_m2=.01)))
        self.session.tick()
        self.assertFalse(self.session.auxiliary_fresh('pressure'))
        packet=dict(stamp_ns=2_000_000_000,height_m=-2.,variance_m2=.01)
        self.session.auxiliary('pressure',2_000_000_000,dict(valid=True,packet=packet))
        self.session.tick()
        return self.session.client

    def test_all_selected_auxiliaries_must_be_valid_and_recent_before_allocation(self):
        for enabled in (('dvl',), ('pressure',), ('dvl', 'pressure')):
            with self.subTest(enabled=enabled):
                self.session.close(); self.now=0.
                config=Path(self.directory.name)/'selected.json'
                config.write_text(json.dumps(dict(state_clock='sonar',enable_joint_sonar=True,
                    enable_joint_dvl='dvl' in enabled,enable_joint_pressure='pressure' in enabled)))
                self.session=SonarSession('worker',config,None,range_sigma=.02,azimuth_sigma=.01,
                    dvl_gyro_sigma=.001 if 'dvl' in enabled else None,
                    clock=lambda:self.now,client_factory=Worker)
                self.addCleanup(self.session.close);self.feed()
                for self.now in (0.,4.,40.):
                    self.assertEqual(self.session.tick(),[])
                    self.assertIsNone(self.session.client)
                # Each selected source is required, not just the first one.
                for kind in enabled[:-1]:
                    self.session.auxiliary(kind,1_990_000_000,dict(valid=True))
                missing=enabled[-1]
                self.session.auxiliary(missing,1_990_000_000,dict(valid=False))
                self.session.tick();self.assertIsNone(self.session.client)
                self.session.auxiliary(missing,1_991_000_000,dict(valid=True))
                self.now+=2.
                self.session.tick();self.assertIsNone(self.session.client)
                for kind in enabled:
                    self.session.auxiliary(kind,1_992_000_000,dict(valid=True))
                self.session.tick();self.assertIsNotNone(self.session.client)
                self.assertEqual(self.session.states,0)
                self.assertFalse(any(self.session.optimized.values()))

    def test_pressure_traffic_is_not_fusion_and_freshness_expires_on_either_clock(self):
        worker=self.pressure_session()
        self.assertEqual(len(worker.pressures),1)
        self.assertFalse(self.session.auxiliary_fresh('pressure'))
        state=self.state();state['pressure']=dict(owner_received=True,optimization_included=True)
        worker.outputs=[state];self.session.tick()
        self.assertTrue(self.session.auxiliary_fresh('pressure'))
        self.session.auxiliary('pressure',2_010_000_000,dict(valid=False))
        self.now=2.01
        self.assertFalse(self.session.auxiliary_fresh('pressure'))
        self.now=.1;self.session.last_result=dict(state,stamp_ns=4_000_000_000)
        self.assertFalse(self.session.auxiliary_fresh('pressure'))
        self.assertEqual(self.session.optimized['pressure'],1)

    def test_no_echo_continues_original_imu_pressure_and_native_state_clock(self):
        worker=self.pressure_session()
        state=self.state();state['pressure']=dict(owner_received=True,optimization_included=True)
        worker.outputs=[state];self.session.tick()
        for t in range(2_020_000_000,2_520_000_000,10_000_000):
            self.session.imu(t,[0,0,0],[0,0,9.80665],[0]*9)
        absent=frame(0);absent['stamp']=np.array([2,500_000_000]);absent['intensity_db'][:]=-np.inf
        self.session.sonar(absent)
        packet=dict(stamp_ns=2_500_000_000,height_m=-2.,variance_m2=.01)
        self.session.auxiliary('pressure',2_500_000_000,dict(valid=True,packet=packet))
        self.session.tick()
        self.assertIsNone(self.session.fault)
        self.assertEqual(worker.ticks,[2_000_000_000,2_500_000_000])
        self.assertEqual(worker.pressures[-1],packet)
        self.assertGreater(worker.imus[-1],worker.ticks[-1])
        self.assertEqual(self.session.fusion.snapshot()['points'],[])
        self.assertTrue(np.isneginf(absent['intensity_db']).all())

    def test_rejected_pressure_and_wrong_state_never_record_successful_fusion(self):
        worker=self.pressure_session()
        state=self.state();state['pressure']=dict(owner_received=True,optimization_included=False)
        worker.outputs=[state];self.session.tick()
        self.assertFalse(self.session.auxiliary_fresh('pressure'))
        self.assertEqual(self.session.optimized['pressure'],0)

    def test_unsolicited_optimization_fails_before_map_or_freshness_update(self):
        self.feed();self.session.tick()
        state=self.state();state['pressure']=dict(owner_received=False,optimization_included=True)
        self.session.client.outputs=[state]
        with self.assertRaisesRegex(RuntimeError,'optimization_mismatch'):self.session.tick()
        self.assertEqual(self.session.fusion.archive.stamp,0)
        self.assertEqual(self.session.optimized['pressure'],0)


if __name__ == '__main__': unittest.main()
