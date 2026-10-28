"""Camera/IMU factor ownership during long-window feature retirement.

The translated texture is a deterministic lifecycle fixture, not a metric
accuracy or observability certificate. No auxiliary sensor is enabled or sent.
"""
import json
import os
from pathlib import Path
import tempfile
import time
import unittest
import numpy as np
from posim_slam.live_client import LiveClient


@unittest.skipUnless(os.environ.get('POSIM_SLAM_STREAM_BINARY'), 'Native binary required')
class VisualWindowTests(unittest.TestCase):
    def receive(self, client, stamp):
        result={};deadline=time.monotonic()+10
        while time.monotonic()<deadline:
            for row in client.poll():
                if row.get('stamp_ns')==stamp and row['type'] in ('state','map'):
                    result[row['type']]=row
            if len(result)==2:return result
            time.sleep(.002)
        self.fail('Camera/IMU state/map timeout at '+str(stamp))

    def test_young_lost_hosts_survive_full_inertial_window(self):
        # Sixteen states span multiple keyframes. Some lost features therefore
        # have young hosts that remain full IMU states during marginalization.
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder);config=root/'calibration.json'
            config.write_text(json.dumps(dict(schema='posim-slam-vio-recording-1',
                width=160,height=128,fx=100.,fy=100.,cx=80.,cy=64.,
                T_i_c_q_xyzw=[0,0,0,1],T_i_c_translation_m=[0,0,0],
                imu_rate_hz=100.,gravity_m_s2=9.80665,max_imu_gap_s=.02,
                orientation_used=False,require_gravity_seed=True,
                max_recent_states=16,optical_flow_type='patch',
                gyro_noise_density=.001,accel_noise_density=.02,
                gyro_bias_random_walk=.0001,accel_bias_random_walk=.001)))
            log=(root/'worker.log').open('w');self.addCleanup(log.close)
            client=LiveClient(os.environ['POSIM_SLAM_STREAM_BINARY'],config,log)
            self.addCleanup(client.close)
            deadline=time.monotonic()+10;ready=None
            while ready is None and time.monotonic()<deadline:
                ready=next((r for r in client.poll() if r['type']=='ready'),None)
                time.sleep(.002)
            self.assertIsNotNone(ready)
            self.assertEqual(ready['max_recent_states'],16)
            self.assertFalse(ready['joint_ping_enabled'])
            self.assertFalse(ready['joint_sonar_enabled'])
            self.assertIsNone(ready['dvl_measurement_transport'])
            self.assertIsNone(ready['pressure_measurement_transport'])
            texture=(np.random.default_rng(17).integers(0,2,(32,104))*220+15).astype(np.uint8)
            texture=np.repeat(np.repeat(texture,4,axis=0),4,axis=1)
            initial=1_000_000_000;next_imu=initial;peak=0
            client.inertial_seed(initial,[0,0,1],[.2,0,0],[0,0,0],[0,0,0])
            try:
                for index in range(121):
                    stamp=initial+index*100_000_000
                    while next_imu<=stamp+10_000_000:
                        client.imu(next_imu,[0,0,0],[0,0,9.80665]);next_imu+=10_000_000
                    # New texture enters while old tracks leave the field.
                    image=texture[:,2*index:2*index+160].copy()
                    client.image(stamp,160,128,image.tobytes())
                    rows=self.receive(client,stamp);state=rows['state']
                    self.assertTrue(np.isfinite([*state['position'],*state['velocity'],*state['orientation_xyzw']]).all())
                    self.assertFalse(state['ping_fused'])
                    self.assertIsNone(state['ping_observation'])
                    self.assertIsNone(state['sonar'])
                    self.assertIsNone(state['dvl'])
                    self.assertIsNone(state['pressure'])
                    peak=max(peak,rows['map']['landmarks_window'])
                    if peak:
                        # Diagnostics are copied just before retirement, so
                        # one newly inserted state may exceed the configured16.
                        self.assertLessEqual(state['initialization_window']['states'],17)
                self.assertGreater(peak,5,'Fixture must create real visual landmarks')
                self.assertGreater(rows['map']['observations'],5)
                self.assertEqual(state['stamp_ns'],13_000_000_000)
                self.assertGreater(state['initialization_window']['oldest_stamp_ns'],initial)
            except (RuntimeError,AssertionError) as error:
                self.fail(str(error)+'\n'+(root/'worker.log').read_text()[-2500:])


if __name__=='__main__':unittest.main()
