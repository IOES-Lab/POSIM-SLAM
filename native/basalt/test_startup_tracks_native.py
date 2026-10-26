"""Native optical bootstrap ownership; no estimator pose before a seed."""
import json
import os
from pathlib import Path
import tempfile
import time
import unittest
import numpy as np
from posim_slam.live_client import LiveClient


@unittest.skipUnless(os.environ.get('POSIM_SLAM_STREAM_BINARY'), 'Native binary required')
class StartupTracksTests(unittest.TestCase):
    def worker(self, root):
        config = dict(schema='posim-slam-vio-recording-1',width=160,height=128,
            fx=100.,fy=100.,cx=80.,cy=64.,T_i_c_q_xyzw=[0,0,0,1],
            T_i_c_translation_m=[0,0,0],imu_rate_hz=100.,gravity_m_s2=9.80665,
            max_imu_gap_s=.02,orientation_used=False,require_gravity_seed=True,
            gyro_noise_density=.001,accel_noise_density=.02,
            gyro_bias_random_walk=.0001,accel_bias_random_walk=.001)
        file=root/'calibration.json';file.write_text(json.dumps(config))
        log=(root/'worker.log').open('w');self.addCleanup(log.close)
        client=LiveClient(os.environ['POSIM_SLAM_STREAM_BINARY'],file,log)
        self.addCleanup(client.close)
        self.receive(client,'ready')
        self.assertTrue(client.startup_tracks_transport)
        return client

    def receive(self, client, kind):
        deadline=time.monotonic()+10
        while time.monotonic()<deadline:
            for row in client.poll():
                if row['type']==kind:return row
                if kind=='startup_tracks':self.fail('Estimator output before initialization: '+row['type'])
            time.sleep(.002)
        self.fail('Native startup reply timeout')

    def test_tracks_without_imu_then_seeded_estimator_handoff(self):
        with tempfile.TemporaryDirectory() as folder:
            client=self.worker(Path(folder));rows=[]
            image=(np.random.default_rng(17).integers(0,2,(32,40))*220+15).astype(np.uint8)
            image=np.repeat(np.repeat(image,4,axis=0),4,axis=1)
            for i in range(3):
                stamp=1_000_000_000+i*200_000_000
                client.startup_image(stamp,160,128,np.roll(image,i,axis=1).tobytes())
                row=self.receive(client,'startup_tracks');rows.append(row)
                self.assertEqual(row['stamp_ns'],stamp)
                self.assertGreater(len(row['tracks']),0)
                self.assertTrue(all(set(v)=={'id','pixel'} for v in row['tracks']))
            self.assertTrue(set(v['id'] for v in rows[0]['tracks']) & set(v['id'] for v in rows[-1]['tracks']))
            stamp=1_600_000_000
            client.inertial_seed(stamp,[0,0,1],[.2,0,0],[0,0,0],[0,0,0])
            for t in [stamp,stamp+10_000_000]:client.imu(t,[0,0,0],[0,0,9.80665])
            client.image(stamp,160,128,np.roll(image,3,axis=1).tobytes())
            state=self.receive(client,'state');self.assertEqual(state['stamp_ns'],stamp)
            np.testing.assert_allclose(state['velocity'],[.2,0,0],atol=1e-8)
            np.testing.assert_allclose(state['position'],[0,0,0],atol=1e-8)

    def test_preinitialization_timestamp_must_increase(self):
        with tempfile.TemporaryDirectory() as folder:
            client=self.worker(Path(folder));pixels=bytes(160*128)
            client.startup_image(1_000_000_000,160,128,pixels)
            self.receive(client,'startup_tracks')
            client.startup_image(1_000_000_000,160,128,pixels)
            with self.assertRaisesRegex(RuntimeError,'startup_image_clock'):
                self.receive(client,'startup_tracks')

    def test_startup_frame_capacity_is_bounded(self):
        with tempfile.TemporaryDirectory() as folder:
            client=self.worker(Path(folder));pixels=bytes(160*128)
            for index in range(32):
                stamp=1_000_000_000+index*200_000_000
                client.startup_image(stamp,160,128,pixels)
                self.receive(client,'startup_tracks')
            client.startup_image(stamp+200_000_000,160,128,pixels)
            with self.assertRaisesRegex(RuntimeError,'startup_image_contract'):
                self.receive(client,'startup_tracks')


if __name__=='__main__':unittest.main()
