"""Ground raw ranges with native IMU/DVL and repeated marginalization."""
import json
import math
import os
from pathlib import Path
import struct
import tempfile
import unittest
import numpy as np
import test_sonar_clock_native as fixture
from posim_slam.sonar_ground_packet import encode_ground


@unittest.skipUnless(os.environ.get('POSIM_SLAM_STREAM_BINARY'),'Native worker required')
class GroundNativeTests(unittest.TestCase):
    def receive(self,client,kind,timeout=10):
        try:return fixture.SonarClockTests.receive(self,client,kind,timeout)
        except Exception:
            import sys
            print(self.native_log.read_text()[-6000:],file=sys.stderr);raise

    def worker(self,root,pressure=False):
        self.native_log=root/'worker.log'
        if pressure:
            (root/'pressure-calibration.json').write_text(json.dumps(dict(
                schema='posim-slam-pressure-calibration-1',imu_to_pressure=[0,0,0])))
        (root/'dvl-calibration.json').write_text(json.dumps(dict(schema='posim-slam-dvl-calibration-1',
            dvl_to_imu=np.eye(3).tolist(),imu_to_dvl=[0,0,0])))
        client=fixture.SonarClockTests.worker(self,root,enable_sonar_ground=True,enable_joint_dvl=True,
            enable_joint_pressure=pressure,
            ground_individual_sigma_m=.03,ground_frame_common_sigma_m=.02,ground_time_common_sigma_m=.01,
            ground_support_radius_m=.75,ground_max_age_s=2.,ground_max_gap_s=.5,ground_innovation_sigma=5.)
        self.assertTrue(client.ground_transport);self.assertFalse(client.sonar_transport);return client

    def test_asynchronous_single_imu_intervals_keep_historical_poses(self):
        # DVL and pressure generate distinct states only one IMU sample apart.
        # Check every owned historical pose, not just the newest online output.
        with tempfile.TemporaryDirectory() as folder:
            client=self.worker(Path(folder),pressure=True);initial=10**9;next_imu=initial;maximum=0.
            client.gravity_seed(initial,[0,0,1])
            def rotation(t):
                c,s=math.cos(.12*t),math.sin(.12*t)
                return np.array([[c,-s,0],[s,c,0],[0,0,1]])
            def position(t):return np.array([.1*t,.1*(1-math.cos(.3*t)),.2*(1-math.cos(.4*t))])
            for i in range(61):
                for offset in (0,10_000_000,60_000_000):
                    stamp=initial+i*200_000_000+offset;t=(stamp-initial)*1e-9
                    while next_imu<=stamp+10_000_000:
                        u=(next_imu-initial)*1e-9
                        acceleration=rotation(u).T@np.array([0,.009*math.cos(.3*u),9.80665+.032*math.cos(.4*u)])
                        acceleration+=np.array([.002*math.sin(1.7*u),.001*math.cos(1.3*u),.001])
                        client.imu(next_imu,[0,0,.12],acceleration);next_imu+=10_000_000
                    if offset==0:
                        velocity=rotation(t).T@np.array([.1,.03*math.sin(.3*t),.08*math.sin(.4*t)])
                        client.dvl(stamp,velocity,np.eye(3)*.0001,gyro=[0,0,.12],gyro_stamp_ns=stamp,
                            gyro_covariance=np.eye(3)*1e-6,bottom_locked=True,locked_beams=4)
                    if offset==10_000_000:client.pressure(stamp,position(t)[2]+17+.001*math.sin(2.1*t),.005**2)
                    client.sonar_tick(stamp);state=self.receive(client,'state')
                    for pose in state['ground']['pose_estimates']:
                        expected=position((pose['stamp_ns']-initial)*1e-9)
                        maximum=max(maximum,float(np.linalg.norm(np.asarray(pose['position'])-expected)))
                    self.assertLessEqual(len(state['ground']['pose_estimates']),17)
                    self.assertLess(maximum,.1)
            print(json.dumps(dict(scope='synthetic asynchronous IMU/DVL/pressure retirement regression',
                states=183,minimum_state_interval_s=.01,maximum_owned_pose_error_m=maximum)))

    def observation(self,t):
        z=.2*(1-math.cos(.4*t));ranges=np.ones(16)*(2+z)/math.sin(.35)+.003*math.sin(2*t)
        return dict(beam_ids=list(range(16)),azimuth_rad=np.linspace(-.8,.8,16).tolist(),
            range_m=ranges.tolist(),boundary_sign=-1,normal_sonar=[0,0,1],distance_sonar_m=-(2+z))

    def test_rotating_sloped_ground_preserves_owned_trajectory(self):
        # Exact synthetic acoustic returns isolate geometry/ownership from the
        # recorded echo-onset frontend. Truth is used only to generate sensors
        # and score outputs; the worker receives no poses or world-plane prior.
        def motion(t):
            r,p,y=.10*math.sin(.4*t),.08*math.sin(.3*t),.12*t
            cr,sr,cp,sp,cy,sy=math.cos(r),math.sin(r),math.cos(p),math.sin(p),math.cos(y),math.sin(y)
            rotation=np.array([[cy,-sy,0],[sy,cy,0],[0,0,1]])@np.array([[cp,0,sp],[0,1,0],[-sp,0,cp]])@np.array([[1,0,0],[0,cr,-sr],[0,sr,cr]])
            gyro=np.array([.04*math.cos(.4*t)-.12*sp,
                .024*math.cos(.3*t)*cr+.12*sr*cp,
                -.024*math.cos(.3*t)*sr+.12*cr*cp])
            position=np.array([.1*t,.1*(1-math.cos(.3*t)),.2*(1-math.cos(.4*t))])
            velocity=np.array([.1,.03*math.sin(.3*t),.08*math.sin(.4*t)])
            acceleration=np.array([0,.009*math.cos(.3*t),9.80665+.032*math.cos(.4*t)])
            return rotation,gyro,position,rotation.T@velocity,rotation.T@acceleration
        normal=np.array([.08,-.04,1.]);normal/=np.linalg.norm(normal)
        azimuth=np.linspace(-.8,.8,16)
        rays=np.column_stack((np.cos(azimuth)*math.cos(.35),np.sin(azimuth)*math.cos(.35),np.full(16,-math.sin(.35))))
        results=[]
        for use_ground in (False,True):
            with tempfile.TemporaryDirectory() as folder:
                client=self.worker(Path(folder),pressure=True);initial=10**9;next_imu=initial
                client.gravity_seed(initial,[0,0,1]);maximum=0.;registered=0;multi=0;planes=set()
                for i in range(61):
                    for offset in (0,10_000_000,60_000_000):
                        stamp=initial+i*200_000_000+offset;t=(stamp-initial)*1e-9
                        rotation,gyro,position,velocity,_=motion(t)
                        while next_imu<=stamp+10_000_000:
                            _,w,_,_,acceleration=motion((next_imu-initial)*1e-9)
                            client.imu(next_imu,w,acceleration);next_imu+=10_000_000
                        if offset==0:
                            client.dvl(stamp,velocity,np.eye(3)*.0001,gyro=gyro,gyro_stamp_ns=stamp,
                                gyro_covariance=np.eye(3)*1e-6,bottom_locked=True,locked_beams=4)
                        if offset==10_000_000:client.pressure(stamp,position[2]+17,.005**2)
                        if offset==60_000_000 and use_ground:
                            ranges=(-1.2-normal@position)/(rays@(rotation.T@normal))
                            self.assertGreater(float(ranges.min()),.2)
                            self.assertLess(float(ranges.max()),10.)
                            # Recover the packet seed from the synthetic range
                            # samples, exactly as an unknown local-plane fit.
                            points=rays*ranges[:,None];center=points.mean(axis=0)
                            _,_,v=np.linalg.svd(points-center);local_normal=v[-1]
                            if local_normal[2]<0:local_normal=-local_normal
                            client.ground(stamp,beam_ids=list(range(16)),azimuth_rad=azimuth.tolist(),
                                range_m=ranges.tolist(),boundary_sign=-1,normal_sonar=local_normal.tolist(),
                                distance_sonar_m=float(local_normal@center))
                        client.sonar_tick(stamp);state=self.receive(client,'state')
                        for pose in state['ground']['pose_estimates']:
                            truth=motion((pose['stamp_ns']-initial)*1e-9)[2]
                            maximum=max(maximum,float(np.linalg.norm(np.asarray(pose['position'])-truth)))
                        if state['ground'].get('factor_registered'):
                            registered+=1;planes.add(state['ground']['plane_id'])
                            multi+=int(state['ground'].get('group_frames',0)>1)
                        self.assertFalse(state['map_qualified'])
                if use_ground:
                    self.assertEqual(registered,61);self.assertGreater(multi,20);self.assertGreater(len(planes),3)
                results.append(dict(ground=use_ground,maximum_owned_pose_error_m=maximum,
                    ground_registered=registered,multi_frame_states=multi,planes=len(planes)))
                self.assertLess(maximum,.03)
        print(json.dumps(dict(scope='synthetic sloped plane and three-axis rotation; no image acceptance',
            states_per_run=183,results=results)))

    def test_ground_event_enters_solver_without_dvl_or_pressure_event(self):
        with tempfile.TemporaryDirectory() as folder:
            client=self.worker(Path(folder));initial=10**9
            client.gravity_seed(initial,[0,0,1])
            client.imu(initial,[0,0,0],[0,0,9.80665])
            client.imu(initial+10_000_000,[0,0,0],[0,0,9.80665])
            client.ground(initial,**self.observation(0));client.sonar_tick(initial)
            state=self.receive(client,'state')
            self.assertTrue(state['ground']['factor_registered'])
            self.assertTrue(state['ground']['optimization_included'])
            self.assertIsNone(state['dvl'])
            self.assertFalse(state['map_qualified'])

    def test_raw_ground_loss_and_repeated_retirement(self):
        with tempfile.TemporaryDirectory() as folder:
            client=self.worker(Path(folder));initial=10**9;next_imu=initial
            client.gravity_seed(initial,[0,0,1]);planes=set();multi=0;registered=0;errors=[]
            for i in range(61):
                stamp=initial+i*200_000_000;t=.2*i
                while next_imu<=stamp+10_000_000:
                    u=(next_imu-initial)*1e-9
                    client.imu(next_imu,[0,0,0],[0,.009*math.cos(.3*u),9.80665+.032*math.cos(.4*u)])
                    next_imu+=10_000_000
                client.dvl(stamp,[.12,.03*math.sin(.3*t),.08*math.sin(.4*t)],np.eye(3)*.0001,
                    gyro=[0,0,0],gyro_stamp_ns=stamp,gyro_covariance=np.eye(3)*1e-6,bottom_locked=True,locked_beams=4)
                present=not 20<=i<30
                if present:client.ground(stamp,**self.observation(t))
                client.sonar_tick(stamp);state=self.receive(client,'state')
                self.assertLessEqual(len(state['sonar']['retained_pose_stamps_ns']),16)
                self.assertFalse(state['map_qualified']);self.assertEqual(state['sonar']['active_track_ids'],[])
                self.assertTrue(state['ground']['pose_estimates'])
                self.assertLessEqual(len(state['ground']['point_estimates']),512*8)
                truth=np.array([.12*t,.1*(1-math.cos(.3*t)),.2*(1-math.cos(.4*t))])
                errors.append(float(np.linalg.norm(np.asarray(state['position'])-truth)))
                if present:
                    g=state['ground'];self.assertTrue(g['owner_received']);self.assertTrue(g['factor_registered'])
                    registered+=1;planes.add(g['plane_id']);multi+=int(g.get('group_frames',0)>1)
                else:self.assertFalse(state['ground']['owner_received'])
            self.assertGreater(len(planes),3);self.assertGreater(multi,20);self.assertLess(max(errors),.1)
            self.assertGreater(min(state['sonar']['retained_pose_stamps_ns']),initial)
            print(json.dumps(dict(scope='synthetic native IMU/DVL/ground lifecycle, not recorded-map acceptance',
                states=61,ground_registered=registered,multi_frame_states=multi,planes=len(planes),
                max_position_error_m=max(errors),camera_packets=0,loss_seconds=2)))

    def test_bad_packets_replay_missing_clock_and_double_use_fail(self):
        for case in ('nonfinite','range','duplicate','length','replay','missing_state','point_factors'):
            with self.subTest(case=case),tempfile.TemporaryDirectory() as folder:
                client=self.worker(Path(folder));initial=10**9;client.gravity_seed(initial,[0,0,1])
                packet=bytearray(encode_ground(initial,**self.observation(0)))
                if case=='nonfinite':struct.pack_into('<d',packet,16,float('nan'))
                if case=='range':struct.pack_into('<d',packet,60,20.)
                if case=='duplicate':struct.pack_into('<I',packet,68,0)
                if case=='length':packet=packet[:-1]
                if case=='point_factors':client.send(7,b'')
                else:client.send(12,packet)
                if case=='replay':client.send(12,packet)
                if case=='missing_state':
                    client.imu(initial,[0,0,0],[0,0,9.80665]);client.imu(initial+10_000_000,[0,0,0],[0,0,9.80665])
                    client.sonar_tick(initial+5_000_000)
                with self.assertRaises(RuntimeError):self.receive(client,'state')


if __name__=='__main__':unittest.main()
