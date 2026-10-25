"""Camera-free raw Ping lifecycle on a synthetic plane; not terrain-map acceptance."""
import json,math,os,struct,tempfile,unittest
from pathlib import Path
import numpy as np
import test_sonar_clock_native as fixture
from posim_slam.ping_packet import encode_ping


@unittest.skipUnless(os.environ.get('POSIM_SLAM_STREAM_BINARY'),'Native worker required')
class TerrainPingNativeTests(unittest.TestCase):
    def receive(self,client,kind,timeout=10):
        try:return fixture.SonarClockTests.receive(self,client,kind,timeout)
        except Exception:
            import sys
            print(self.native_log.read_text()[-6000:],file=sys.stderr)
            raise

    def worker(self,root):
        self.native_log=root/'worker.log'
        (root/'ping-calibration.json').write_text(json.dumps(dict(
            schema='posim-slam-ping-calibration-1',imu_to_ping=[.1,0,.2],ping_to_imu=np.eye(3).tolist(),
            rays=[[0,0,-1]],range_min=.2,range_max=10.,range_bias=.1,per_ray_noise_stddev=.005)))
        (root/'dvl-calibration.json').write_text(json.dumps(dict(
            schema='posim-slam-dvl-calibration-1',dvl_to_imu=np.eye(3).tolist(),imu_to_dvl=[0,0,0])))
        client=fixture.SonarClockTests.worker(self,root,enable_terrain_ping=True,enable_joint_dvl=True,
            terrain_return_sigma_m=.03,terrain_patch_radius_m=.75,terrain_patch_max_age_s=2.,terrain_innovation_sigma=5.)
        self.assertTrue(client.terrain_ping_transport)
        return client

    def test_unknown_plane_camera_absent_loss_and_retirement(self):
        self.plane_case(False)

    def test_unknown_sloped_plane(self):
        self.plane_case(True)

    def plane_case(self,sloped):
        with tempfile.TemporaryDirectory() as folder:
            client=self.worker(Path(folder));initial=10**9;next_imu=initial
            client.gravity_seed(initial,[0,0,1]);planes=set();received=0;ranks=[];errors=[]
            for i in range(61):
                stamp=initial+i*200_000_000;t=i*.2
                while next_imu<=stamp+10_000_000:
                    u=(next_imu-initial)*1e-9
                    client.imu(next_imu,[0,0,0],[0,.025*math.cos(.5*u),9.80665+.128*math.cos(.8*u)])
                    next_imu+=10_000_000
                client.dvl(stamp,[.2,.05*math.sin(.5*t),.16*math.sin(.8*t)],np.eye(3)*.0001,
                    gyro=[0,0,0],gyro_stamp_ns=stamp,gyro_covariance=np.eye(3)*1e-6,bottom_locked=True,locked_beams=4)
                present=not 20<=i<30
                terrain_change=-.02*t+.005*(1-math.cos(.5*t)) if sloped else 0.
                if present:client.terrain_ping(stamp,3.3+.2*(1-math.cos(.8*t))+terrain_change+.003*math.sin(2*t))
                client.sonar_tick(stamp);state=self.receive(client,'state')
                self.assertLessEqual(len(state['sonar']['retained_pose_stamps_ns']),16)
                self.assertFalse(state['map_qualified'])
                truth=np.array([.2*t,.1*(1-math.cos(.5*t)),.2*(1-math.cos(.8*t))])
                errors.append(float(np.linalg.norm(np.asarray(state['position'])-truth)))
                if i==0:
                    self.assertGreater(state['dvl']['accepted_steps'],0)
                if present:
                    self.assertTrue(state['ping_owner_received']);received+=1
                    candidate=state['ping_candidate'];self.assertEqual(candidate['reason'],'terrain_registered')
                    planes.add(candidate['plane_id']);ranks.append(candidate['conditional_plane_rank'])
                    self.assertFalse(candidate['map_qualified'])
                else:self.assertIsNone(state['ping_observation'])
            self.assertGreater(len(planes),3)
            self.assertGreater(min(state['sonar']['retained_pose_stamps_ns']),initial)
            self.assertGreater(max(ranks),1)
            self.assertLess(max(errors),.1)
            print(json.dumps(dict(states=61,range_reports=received,planes=len(planes),camera_packets=0,
                range_loss_s=2,sloped=sloped,max_position_error_m=max(errors),
                scope='synthetic transport and repeated marginalization; no map qualification')))

    def test_bad_packets_and_missing_exact_state_fail(self):
        for case in ('nonfinite','range','clock','length','missing_state','replay'):
            with self.subTest(case=case),tempfile.TemporaryDirectory() as folder:
                client=self.worker(Path(folder));initial=10**9;client.gravity_seed(initial,[0,0,1])
                packet=bytearray(encode_ping(initial,3.3))
                if case=='nonfinite':struct.pack_into('<d',packet,8,float('nan'))
                if case=='range':struct.pack_into('<d',packet,8,11.)
                if case=='clock':struct.pack_into('<q',packet,0,0)
                if case=='length':packet=packet[:-1]
                client.send(11,packet)
                if case=='replay':client.send(11,packet)
                if case=='missing_state':
                    client.imu(initial,[0,0,0],[0,0,9.80665]);client.imu(initial+10_000_000,[0,0,0],[0,0,9.80665])
                    client.sonar_tick(initial+5_000_000)
                with self.assertRaises(RuntimeError):self.receive(client,'state')


if __name__=='__main__':unittest.main()
