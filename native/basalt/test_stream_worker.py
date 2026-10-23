"""Native clock/saturation/packet failure and process cleanup checks."""
import json,os,tempfile,time,unittest
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
                    deadline=time.monotonic()+3
                    while not client.fault and time.monotonic()<deadline:time.sleep(.005)
                    self.assertIn(reason,client.fault or '')
                    with self.assertRaises(RuntimeError):client.poll()
                finally:client.close()
                self.assertFalse(client.writer.is_alive());self.assertFalse(client.reader.is_alive())
                self.assertIsNotNone(client.process.poll())

    def test_clock_reversal(self):self.check_fault('duplicate','imu_gap_or_reset')
    def test_imu_gap(self):self.check_fault('gap','imu_gap_or_reset')
    def test_saturation(self):self.check_fault('saturation','invalid_imu')
    def test_incomplete_image(self):self.check_fault('packet','image_packet_size')
    def test_incomplete_snapshot_request(self):self.check_fault('snapshot','snapshot_request_size')


if __name__=='__main__':unittest.main()
