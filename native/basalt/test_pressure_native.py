"""Synthetic pressure ownership and depth observability, not map acceptance."""
import json
import math
import os
from pathlib import Path
import struct
import tempfile
import unittest

import numpy as np
import test_sonar_clock_native as fixture
from posim_slam.pressure_packet import encode_pressure
from posim_slam.live_client import LiveClient


@unittest.skipUnless(os.environ.get('POSIM_SLAM_STREAM_BINARY'), 'Native worker binary required')
class PressureNativeTests(unittest.TestCase):
    receive = fixture.SonarClockTests.receive

    def worker(self, root, dvl=False):
        (root/'pressure-calibration.json').write_text(json.dumps(dict(
            schema='posim-slam-pressure-calibration-1', imu_to_pressure=[.2,-.1,.3])))
        if dvl:
            (root/'dvl-calibration.json').write_text(json.dumps(dict(
                schema='posim-slam-dvl-calibration-1', dvl_to_imu=np.eye(3).tolist(), imu_to_dvl=[0,0,0])))
        client = fixture.SonarClockTests.worker(self, root, enable_joint_pressure=True, enable_joint_dvl=dvl)
        self.assertTrue(client.pressure_transport)
        return client

    def test_unknown_datum_noisy_first_sample_loss_and_repeated_marginalization(self):
        for dvl in (False, True):
            with self.subTest(dvl=dvl), tempfile.TemporaryDirectory() as folder:
                client = self.worker(Path(folder), dvl)
                initial = 1_000_000_000; next_imu = initial
                client.gravity_seed(initial, [0,0,1]); errors=[]; reports=0
                for index in range(101):
                    stamp=initial+index*200_000_000; t=index*.2
                    while next_imu<=stamp+10_000_000:
                        u=(next_imu-initial)*1e-9
                        client.imu(next_imu,[0,0,0],[0,0,9.80665+.128*math.cos(.8*u)+.01])
                        next_imu+=10_000_000
                    z=.2*(1-math.cos(.8*t))
                    if dvl:
                        client.dvl(stamp,[.1,0,.16*math.sin(.8*t)],np.eye(3)*.0001,
                            gyro=[0,0,0],gyro_stamp_ns=stamp,gyro_covariance=np.eye(3)*1e-6,
                            bottom_locked=True,locked_beams=4)
                    present=not 40<=index<60
                    if present:
                        noise=.02 if index==0 else .003*math.sin(2.1*t)
                        client.pressure(stamp,z+.3+17+noise,.005**2)
                    client.sonar_tick(stamp); state=self.receive(client,'state')
                    self.assertFalse(state['map_qualified'])
                    self.assertEqual(state['sonar']['active_track_ids'],[])
                    retained=state['sonar']['retained_pose_stamps_ns']
                    self.assertLessEqual(len(retained),16)
                    if present:
                        self.assertTrue(state['pressure']['owner_received'])
                        self.assertTrue(state['pressure']['optimization_included']); reports+=1
                    else:self.assertIsNone(state['pressure'])
                    errors.append(abs(state['position'][2]-z))
                self.assertGreater(min(retained),initial)
                self.assertLess(max(errors),.08)
                self.assertLess(errors[-1],.025)
                self.assertLess(abs(state['pressure']['datum_m']-17),.025)
                if dvl:self.assertLess(abs(state['position'][0]-2),.02)
                print(json.dumps(dict(states=101,pressure_reports=reports,dvl=dvl,camera_packets=0,
                    pressure_loss_s=4,max_depth_error_m=max(errors),final_depth_error_m=errors[-1],
                    datum_m=state['pressure']['datum_m'],scope='synthetic depth fusion; no terrain or map acceptance')))

    def test_invalid_packet_replay_and_missing_exact_state(self):
        for case in ('variance','nonfinite','clock','length','missing_state','replay'):
            with self.subTest(case=case), tempfile.TemporaryDirectory() as folder:
                client=self.worker(Path(folder)); initial=1_000_000_000
                client.gravity_seed(initial,[0,0,1]); packet=bytearray(encode_pressure(initial,-2,.01))
                if case=='variance':struct.pack_into('<d',packet,16,0)
                if case=='nonfinite':struct.pack_into('<d',packet,8,float('nan'))
                if case=='clock':struct.pack_into('<q',packet,0,0)
                if case=='length':packet=packet[:-1]
                for stamp in range(initial,initial+220_000_000,10_000_000):
                    client.imu(stamp,[0,0,0],[0,0,9.80665])
                client.send(10,bytes(packet))
                if case=='replay':client.send(10,bytes(packet))
                else:client.sonar_tick(initial+200_000_000 if case=='missing_state' else initial)
                reason=dict(variance='invalid_pressure_observation',nonfinite='invalid_pressure_observation',
                    clock='pressure_clock',length='pressure_packet_size',
                    missing_state='pressure_observation_missing_state',replay='reordered_pressure_observation')[case]
                with self.assertRaisesRegex(RuntimeError,reason):self.receive(client,'state')

    def test_pressure_only_auxiliary_states_between_camera_frames(self):
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder);config=root/'calibration.json'
            config.write_text(json.dumps(dict(schema='posim-slam-vio-recording-1',orientation_used=False,
                enable_joint_pressure=True,require_gravity_seed=True,max_recent_states=16,
                width=32,height=32,fx=25.6,fy=25.6,cx=16,cy=16,
                T_i_c_q_xyzw=[0,0,0,1],T_i_c_translation_m=[0,0,0],
                imu_rate_hz=100,gyro_noise_density=.001,accel_noise_density=.02,
                gyro_bias_random_walk=.0001,accel_bias_random_walk=.001,gravity_m_s2=9.80665,max_imu_gap_s=.02)))
            (root/'pressure-calibration.json').write_text(json.dumps(dict(
                schema='posim-slam-pressure-calibration-1',imu_to_pressure=[0,0,0])))
            log=(root/'worker.log').open('w');self.addCleanup(log.close)
            client=LiveClient(os.environ['POSIM_SLAM_STREAM_BINARY'],config,log);self.addCleanup(client.close)
            self.receive(client,'ready');self.assertTrue(client.sensor_state_transport)
            initial=1_000_000_000;next_imu=initial;client.gravity_seed(initial,[0,0,1])
            for index in range(65):
                stamp=initial+index*50_000_000;t=index*.05
                while next_imu<=stamp+10_000_000:
                    client.imu(next_imu,[0,0,0],[0,0,9.80665+.02]);next_imu+=10_000_000
                client.pressure(stamp,-2+.01*t*t,.0001)
                if index%4==0:client.image(stamp,32,32,bytes(1024))
                else:client.sonar_tick(stamp)
                # Camera mode emits empty visual-map reports as well as state.
                import time
                deadline=time.monotonic()+5;state=None
                while time.monotonic()<deadline and state is None:
                    for value in client.poll():
                        if value['type']=='state':state=value
                        elif value['type']=='map':self.assertEqual(value['landmarks_window'],0)
                    time.sleep(.002)
                self.assertIsNotNone(state);self.assertEqual(state['stamp_ns'],stamp)
                self.assertTrue(state['pressure']['optimization_included'])
                self.assertLessEqual(state['initialization_window']['states'],17)
            self.assertLess(abs(state['position'][2]-.01*3.2**2),.005)


if __name__=='__main__':unittest.main()
