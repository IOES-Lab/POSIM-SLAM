"""Exercise original frontend-result handoff and refusal in the actual worker."""
import json,os,struct,tempfile,time,unittest
from contextlib import contextmanager
from pathlib import Path
from posim_slam.live_client import LiveClient


@unittest.skipUnless(os.environ.get('POSIM_SLAM_STREAM_BINARY'),'Native worker binary required')
class StartupHistoryWorkerTests(unittest.TestCase):
    @contextmanager
    def worker(self,enabled=True):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);cal=root/'calibration.json'
            cal.write_text(json.dumps(dict(schema='posim-slam-vio-recording-1',orientation_used=False,
                require_gravity_seed=True,retain_startup_measurements=enabled,width=32,height=32,
                fx=30,fy=30,cx=16,cy=16,T_i_c_q_xyzw=[0,0,0,1],T_i_c_translation_m=[0,0,0],
                imu_rate_hz=100,gyro_noise_density=.001,accel_noise_density=.02,
                gyro_bias_random_walk=.0001,accel_bias_random_walk=.001,gravity_m_s2=9.80665,max_imu_gap_s=.02)))
            with (root/'worker.log').open('w') as log:
                client=LiveClient(os.environ['POSIM_SLAM_STREAM_BINARY'],cal,log)
                try:
                    self.wait(client,lambda rows:any(r['type']=='ready' for r in rows),30)
                    yield client
                finally:
                    client.close()
                    self.assertFalse(client.writer.is_alive() or client.reader.is_alive())
                    self.assertIsNotNone(client.process.poll())

    def wait(self,client,predicate,seconds=1):
        rows=[];deadline=time.monotonic()+seconds
        while not predicate(rows):
            rows.extend(client.poll())
            if time.monotonic()>deadline:self.fail('original native acknowledgement budget exceeded')
            time.sleep(.005)
        return rows

    def startup(self,client):
        for stamp in (1_000_000_000,1_200_000_000,1_400_000_000):
            client.startup_image(stamp,32,32,bytes(1024))
            rows=self.wait(client,lambda rows:any(r.get('stamp_ns')==stamp for r in rows))
            self.assertEqual([r['type'] for r in rows],['startup_tracks'])

    def imu(self,client,end,start=800_000_000):
        for stamp in range(start,end+10_000_001,10_000_000):client.imu(stamp,[0,0,0],[0,0,9.80665])

    def expect_fault(self,client,reason):
        with self.assertRaisesRegex(RuntimeError,reason):self.wait(client,lambda _:False)

    def test_retained_results_enter_original_optimizer_once_then_fresh_images_resume(self):
        with self.worker() as client:
            self.startup(client);client.inertial_seed(1_000_000_000,[0,0,1],[0,0,0],[0,0,0],[0,0,0])
            next_imu=800_000_000
            for stamp in (1_000_000_000,1_200_000_000,1_400_000_000,1_600_000_000):
                self.imu(client,stamp,next_imu);next_imu=stamp+20_000_000
                if stamp<1_600_000_000:client.retained_image(stamp)
                else:client.image(stamp,32,32,bytes(1024))
                rows=self.wait(client,lambda rows:any(r['type']=='state' and r['stamp_ns']==stamp for r in rows))
                state=next(r for r in rows if r['type']=='state')
                self.assertEqual(state['initialization_window']['oldest_stamp_ns'],1_000_000_000)

    def test_fresh_image_cannot_skip_pending_original_results(self):
        with self.worker() as client:
            self.startup(client);self.imu(client,1_600_000_000)
            client.image(1_600_000_000,32,32,bytes(1024))
            self.expect_fault(client,'startup_history_pending')

    def test_handoff_order_cannot_skip_first_result(self):
        with self.worker() as client:
            self.startup(client);self.imu(client,1_200_000_000)
            client.retained_image(1_200_000_000)
            self.expect_fault(client,'startup_history_handoff_order')

    def test_fresh_image_cannot_skip_an_unacknowledged_frontend_result(self):
        with self.worker() as client:
            client.startup_image(1_000_000_000,32,32,bytes(1024))
            self.imu(client,1_200_000_000)
            client.image(1_200_000_000,32,32,bytes(1024))
            self.expect_fault(client,'startup_history_pending')

    def test_disabled_transport_is_refused_by_native_parser(self):
        with self.worker(False) as client:
            client.send(16,struct.pack('<q',1_000_000_000))
            self.expect_fault(client,'packet_size_or_type')


if __name__=='__main__':unittest.main()
