"""Raw velocity integration; synthetic sensors, not terrain/map acceptance."""
import json
import os
from pathlib import Path
import struct
import tempfile
import time
import unittest
import numpy as np
import test_sonar_clock_native as fixture
from posim_slam.dvl_packet import encode_dvl
from posim_slam.live_client import LiveClient


@unittest.skipUnless(os.environ.get('POSIM_SLAM_STREAM_BINARY'),'Native worker binary required')
class DvlNativeTests(unittest.TestCase):
    receive=fixture.SonarClockTests.receive

    def worker(self,root):
        (root/'dvl-calibration.json').write_text(json.dumps(dict(
            schema='posim-slam-dvl-calibration-1',dvl_to_imu=np.eye(3).tolist(),imu_to_dvl=[0,0,0])))
        client=fixture.SonarClockTests.worker(self,root,enable_joint_dvl=True)
        self.assertTrue(client.dvl_transport)
        return client

    def observation(self,stamp,velocity):
        return dict(stamp_ns=stamp,velocity=velocity,covariance=np.eye(3)*.0001,
                    gyro=[0,0,0],gyro_stamp_ns=stamp,gyro_covariance=np.eye(3)*.000001,
                    bottom_locked=True,locked_beams=4)

    def test_joint_sonar_tracks_and_dvl_share_optimizer_and_retirement(self):
        fixture.SonarClockTests.test_acoustic_states_marginalize_without_any_camera(self,with_dvl=True)

    def test_moving_start_loss_and_recovery_with_no_visual_or_acoustic_features(self):
        with tempfile.TemporaryDirectory() as folder:
            client=self.worker(Path(folder));initial=1_000_000_000;next_imu=initial
            client.gravity_seed(initial,[0,0,1]);states=[];retained=set();reports=0
            for index in range(101):
                stamp=initial+index*200_000_000;t=index*.2
                while next_imu<=stamp+10_000_000:
                    client.imu(next_imu,[0,0,0],[.02,0,9.80665]);next_imu+=10_000_000
                # Four seconds of deliberate bottom-lock loss. Invalid velocity
                # is not inserted; raw IMU propagation and prior remain active.
                present=not 40<=index<60
                if present:client.dvl(**self.observation(stamp,[.25+.02*t,.05,0]))
                client.sonar_tick(stamp);state=self.receive(client,'state')
                self.assertFalse(state['map_qualified']);self.assertEqual(state['sonar']['active_track_ids'],[])
                if index==0:
                    self.assertGreater(state['dvl']['accepted_steps'],0)
                    np.testing.assert_allclose(state['velocity'],[.25,.05,0],atol=1e-6)
                retained=set(state['sonar']['retained_pose_stamps_ns'])
                self.assertLessEqual(len(retained),16)
                if present:
                    self.assertTrue(state['dvl']['owner_received']);self.assertTrue(state['dvl']['optimization_included']);reports+=1
                else:self.assertIsNone(state['dvl'])
                truth=np.array([.25*t+.01*t*t,.05*t,0]);states.append((state,truth))
            errors=[np.linalg.norm(np.array(s['position'])-p) for s,p in states]
            velocity_error=np.linalg.norm(np.array(states[-1][0]['velocity'])-[.65,.05,0])
            self.assertLess(max(errors),.15);self.assertLess(velocity_error,.01)
            self.assertGreater(min(retained),initial)
            print(json.dumps(dict(states=len(states),dvl_reports=reports,camera_packets=0,acoustic_tracks=0,
                                  bottom_lock_loss_s=4,max_position_error_m=max(errors),final_velocity_error_m_s=velocity_error,
                                  scope='analytic raw velocity ownership and marginalization; no map acceptance')))

    def test_worker_rejects_invalid_lock_clock_covariance_and_skipped_state(self):
        for case in ('lock','clock','covariance','missing_state','replay'):
            with self.subTest(case=case),tempfile.TemporaryDirectory() as folder:
                client=self.worker(Path(folder));initial=1_000_000_000
                client.gravity_seed(initial,[0,0,1])
                packet=bytearray(encode_dvl(**self.observation(initial,[0,0,0])))
                if case=='lock':struct.pack_into('<I',packet,16,0)
                if case=='clock':struct.pack_into('<q',packet,8,initial+1)
                if case=='covariance':struct.pack_into('<d',packet,72,-1)
                for stamp in range(initial,initial+220_000_000,10_000_000):client.imu(stamp,[0,0,0],[0,0,9.80665])
                client.send(9,bytes(packet))
                if case=='replay':client.send(9,bytes(packet))
                else:client.sonar_tick(initial+200_000_000 if case=='missing_state' else initial)
                reason={'lock':'dvl_clock_or_bottom_lock','clock':'dvl_clock_or_bottom_lock',
                        'covariance':'invalid_dvl_covariance','missing_state':'dvl_observation_missing_state',
                        'replay':'reordered_dvl_observation'}[case]
                with self.assertRaisesRegex(RuntimeError,reason):self.receive(client,'state')

    def test_camera_with_multiple_original_dvl_states_between_images(self):
        for width,height in ((32,32),(320,240)):
            for recent in (3,16):
                with self.subTest(width=width,height=height,recent_states=recent):
                    self.camera_case(width,height,recent)

    def camera_case(self,width,height,recent):
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder);config=root/'calibration.json'
            config.write_text(json.dumps(dict(schema='posim-slam-vio-recording-1',orientation_used=False,
                enable_joint_dvl=True,require_gravity_seed=True,max_recent_states=recent,
                width=width,height=height,fx=.8*width,fy=.8*width,cx=width/2,cy=height/2,T_i_c_q_xyzw=[0,0,0,1],T_i_c_translation_m=[0,0,0],
                imu_rate_hz=100,gyro_noise_density=.001,accel_noise_density=.02,
                gyro_bias_random_walk=.0001,accel_bias_random_walk=.001,gravity_m_s2=9.80665,max_imu_gap_s=.02)))
            (root/'dvl-calibration.json').write_text(json.dumps(dict(
                schema='posim-slam-dvl-calibration-1',dvl_to_imu=np.eye(3).tolist(),imu_to_dvl=[0,0,0])))
            log=(root/'worker.log').open('w');self.addCleanup(log.close)
            client=LiveClient(os.environ['POSIM_SLAM_STREAM_BINARY'],config,log);self.addCleanup(client.close)
            self.receive(client,'ready');self.assertTrue(client.sensor_state_transport)
            initial=1_000_000_000;next_imu=initial;client.gravity_seed(initial,[0,0,1])
            for index in range(65):
                stamp=initial+index*50_000_000
                while next_imu<=stamp+10_000_000:
                    client.imu(next_imu,[0,0,0],[0,0,9.80665]);next_imu+=10_000_000
                client.dvl(**self.observation(stamp,[.15,.04,0]))
                if index%4==0:client.image(stamp,width,height,bytes(width*height))
                else:client.sonar_tick(stamp)
                deadline=time.monotonic()+5;state=None
                while time.monotonic()<deadline and state is None:
                    try:values=client.poll()
                    except RuntimeError as error:
                        self.fail(str(error)+' at event '+str(index)+'\n'+(root/'worker.log').read_text()[-2500:])
                    for value in values:
                        if value['type']=='state':state=value
                        elif value['type']=='map':self.assertEqual(value['landmarks_window'],0)
                    time.sleep(.002)
                self.assertIsNotNone(state);self.assertEqual(state['stamp_ns'],stamp)
                self.assertTrue(state['dvl']['optimization_included'])
                self.assertLessEqual(state['initialization_window']['states'],recent+1)
            np.testing.assert_allclose(state['position'],[.15*3.2,.04*3.2,0],atol=1e-4)


if __name__=='__main__':unittest.main()
