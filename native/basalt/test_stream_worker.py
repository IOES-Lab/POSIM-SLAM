"""Native clock/saturation/packet failure and process cleanup checks."""
import json,os,struct,tempfile,time,unittest
from pathlib import Path
from posim_slam.live_client import LiveClient


@unittest.skipUnless(os.environ.get('POSIM_SLAM_STREAM_BINARY'),'Native worker binary required')
class StreamWorkerTests(unittest.TestCase):
    def check_fault(self,kind,reason):
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder);config=root/'calibration.json'
            config.write_text(json.dumps(dict(schema='posim-slam-vio-recording-1',orientation_used=False,
                width=32,height=32,fx=30,fy=30,cx=16,cy=16,T_i_c_q_xyzw=[0,0,0,1],T_i_c_translation_m=[0,0,0],
                imu_rate_hz=100,gyro_noise_density=.001,accel_noise_density=.02,
                gyro_bias_random_walk=.0001,accel_bias_random_walk=.001,gravity_m_s2=9.80665,max_imu_gap_s=.02)))
            with (root/'worker.log').open('w') as log:
                client=LiveClient(os.environ['POSIM_SLAM_STREAM_BINARY'],config,log)
                try:
                    ready=[];deadline=time.monotonic()+3
                    while not ready and time.monotonic()<deadline:
                        ready=client.poll();time.sleep(.005)
                    self.assertTrue(ready)
                    client.imu(1_000_000_000,[0,0,0],[0,0,9.8])
                    if kind=='duplicate':client.imu(1_000_000_000,[0,0,0],[0,0,9.8])
                    if kind=='gap':client.imu(1_050_000_000,[0,0,0],[0,0,9.8])
                    if kind=='saturation':client.imu(1_010_000_000,[100,0,0],[0,0,9.8])
                    if kind=='packet':client.send(2,b'')
                    if kind=='snapshot':client.send(4,b'')
                    if kind.startswith('ping_'):
                        pt=999_999_999 if kind=='ping_stamp' else 1_000_000_000
                        r=float('nan') if kind=='ping_nan' else -1 if kind=='ping_negative' else 2.5
                        client.send(5,struct.pack('<qIIqd',1_000_000_000,32,32,pt,r)+bytes(1024))
                    deadline=time.monotonic()+3
                    while not client.fault and time.monotonic()<deadline:time.sleep(.005)
                    self.assertIn(reason,client.fault or '')
                    with self.assertRaises(RuntimeError):client.poll()
                finally:client.close()
                self.assertFalse(client.writer.is_alive());self.assertFalse(client.reader.is_alive())
                self.assertIsNotNone(client.process.poll())
                self.assertFalse(config.with_suffix('.visual-tracks.jsonl').exists())

    def test_featureless_start_preserves_initial_velocity_state(self):
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder);config=root/'calibration.json'
            config.write_text(json.dumps(dict(schema='posim-slam-vio-recording-1',orientation_used=False,
                record_visual_tracks=True,width=320,height=240,fx=250,fy=250,cx=160,cy=120,T_i_c_q_xyzw=[0,0,0,1],T_i_c_translation_m=[0,0,0],
                imu_rate_hz=100,gyro_noise_density=.001,accel_noise_density=.02,
                gyro_bias_random_walk=.0001,accel_bias_random_walk=.001,gravity_m_s2=9.80665,max_imu_gap_s=.02)))
            with (root/'worker.log').open('w') as log:
                client=LiveClient(os.environ['POSIM_SLAM_STREAM_BINARY'],config,log)
                try:
                    deadline=time.monotonic()+3
                    while not client.ready.is_set() and time.monotonic()<deadline:
                        client.poll();time.sleep(.005)
                    self.assertTrue(client.ready.is_set());client.poll()
                    next_imu=1_000_000_000
                    for index in range(8):
                        stamp=1_000_000_000+index*200_000_000
                        while next_imu<=stamp+10_000_000:
                            client.imu(next_imu,[0,0,0],[0,0,9.80665]);next_imu+=10_000_000
                        client.image(stamp,320,240,bytes(320*240))
                        received=[];deadline=time.monotonic()+5
                        while time.monotonic()<deadline:
                            received.extend(client.poll())
                            if any(v['type']=='state' for v in received) and any(v['type']=='map' for v in received):break
                            time.sleep(.005)
                        state=next(v for v in received if v['type']=='state')
                        cloud=next(v for v in received if v['type']=='map')
                        self.assertEqual(cloud['landmarks_window'],0)
                        self.assertEqual(state['initialization_window']['oldest_stamp_ns'],1_000_000_000)
                        self.assertEqual(state['initialization_window']['states'],index+1)
                finally:client.close()
                self.assertFalse(client.writer.is_alive() or client.reader.is_alive())
                trace=[json.loads(line) for line in config.with_suffix('.visual-tracks.jsonl').read_text().splitlines()]
                self.assertEqual(len(trace),8)
                self.assertTrue(all(row['tracks']==[] for row in trace))
                self.assertTrue(all(row['width']==320 and row['height']==240 for row in trace))

    def test_ping_timestamp_mismatch(self):self.check_fault('ping_stamp','invalid_exact_image_ping')
    def test_ping_nonfinite(self):self.check_fault('ping_nan','invalid_exact_image_ping')
    def test_ping_negative(self):self.check_fault('ping_negative','invalid_exact_image_ping')

    def test_clock_reversal(self):self.check_fault('duplicate','imu_gap_or_reset')
    def test_imu_gap(self):self.check_fault('gap','imu_gap_or_reset')
    def test_saturation(self):self.check_fault('saturation','invalid_imu')
    def test_incomplete_image(self):self.check_fault('packet','image_packet_size')
    def test_incomplete_snapshot_request(self):self.check_fault('snapshot','snapshot_request_size')


if __name__=='__main__':unittest.main()
