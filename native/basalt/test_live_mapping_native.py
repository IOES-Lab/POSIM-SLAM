"""Actual camera/IMU policy and map-failure isolation in the native worker.

Constant-velocity translation of one plane leaves the recovered mapping system
singular. Both backends must withhold the map and keep navigation draining.
Successful mapping and physical accuracy require separate representative data.
"""
import json
import os
from pathlib import Path
import tempfile
import time
import unittest
import numpy as np
from posim_slam.live_client import LiveClient
from posim_slam.map_quality import native_settings, validate_ready, validate_geometry


@unittest.skipUnless(os.environ.get('POSIM_SLAM_STREAM_BINARY'), 'Native binary required')
class LiveMappingTests(unittest.TestCase):
    def wait(self, client, predicate):
        deadline=time.monotonic()+15
        while time.monotonic()<deadline:
            for row in client.poll():
                if predicate(row):return row
            time.sleep(.002)
        self.fail('Native worker response timeout')

    def run_case(self, writable, backend='conditional_paged_v1'):
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder);work=root/'worker';work.mkdir()
            path=work/'calibration.json'
            settings=dict(schema='posim-slam-vio-recording-1',
                width=160,height=128,fx=100.,fy=100.,cx=80.,cy=64.,
                T_i_c_q_xyzw=[0,0,0,1],T_i_c_translation_m=[0,0,0],
                imu_rate_hz=100.,gravity_m_s2=9.80665,max_imu_gap_s=.02,
                orientation_used=False,require_gravity_seed=True,optical_flow_type='patch',
                gyro_noise_density=.001,accel_noise_density=.02,
                gyro_bias_random_walk=.0001,accel_bias_random_walk=.001,
                map_epoch='native-mapping-lifecycle',**dict(native_settings(),map_backend=backend))
            path.write_text(json.dumps(settings))
            log_path=root/'worker.log';log=log_path.open('w');client=None
            if not writable:work.chmod(0o555)
            try:
                client=LiveClient(os.environ['POSIM_SLAM_STREAM_BINARY'],path,log)
                ready=self.wait(client,lambda row:row['type']=='ready')
                if backend=='conditional_paged_v1':validate_ready(ready,epoch=settings['map_epoch'])
                else:self.assertEqual(ready['map_backend'],backend)
                texture=(np.random.default_rng(17).integers(0,2,(32,256))*220+15).astype(np.uint8)
                texture=np.repeat(np.repeat(texture,4,axis=0),4,axis=1)
                initial=1_000_000_000;next_imu=initial;last_state=None;latest_map=None
                client.inertial_seed(initial,[0,0,1],[.2,0,0],[0,0,0],[0,0,0])

                def frame(index):
                    nonlocal next_imu,last_state,latest_map
                    stamp=initial+index*100_000_000
                    while next_imu<=stamp+10_000_000:
                        client.imu(next_imu,[0,0,0],[0,0,9.80665]);next_imu+=10_000_000
                    client.image(stamp,160,128,texture[:,2*index:2*index+160].copy().tobytes())
                    rows={}
                    def received(row):
                        if row.get('stamp_ns')==stamp and row['type'] in ('state','map'):rows[row['type']]=row
                        return len(rows)==2
                    self.wait(client,received);last_state=rows['state'];latest_map=rows['map']
                    self.assertTrue(np.isfinite([*last_state['position'],*last_state['velocity'],*last_state['orientation_xyzw']]).all())
                    validate_geometry(latest_map['point_geometry'],latest_map['landmarks_window'],
                        total_points=latest_map['landmarks_accumulated'],stamp_ns=stamp)

                count=241 if writable else 21
                for index in range(count):frame(index)
                if writable:
                    client.snapshot(1)
                    exported=self.wait(client,lambda row:row['type']=='export' and row['request_id']==1)
                    frame(count)
                    geometry=latest_map['point_geometry']
                    self.assertEqual(geometry['error'],'mapping_recovered_information_singular')
                    self.assertFalse(exported['success'])
                    self.assertEqual(geometry['optimization_revision'],0)
                    self.assertEqual(geometry['candidate_points'],0)
                    self.assertEqual(geometry['eligible_points'],0)
                    self.assertLessEqual(geometry['row_cache_peak_bytes'],ready['map_row_cache_bytes'])
                    rows=list(work.glob('worker-map-inputs-*.sqlite.rows/rows.sqlite'))
                    self.assertEqual(bool(rows),backend=='conditional_paged_v1')
                    self.assertFalse((work/'worker-map.graph.pmap').exists())
                    self.assertEqual(last_state['stamp_ns'],initial+count*100_000_000)
                else:
                    geometry=latest_map['point_geometry']
                    self.assertIsInstance(geometry['error'],str)
                    self.assertIn('storage',geometry['error'])
                    self.assertEqual(geometry['eligible_points'],0)
                    self.assertEqual(geometry['world_observed_points'],0)
                    self.assertEqual(last_state['stamp_ns'],initial+(count-1)*100_000_000)
                    self.assertEqual([p.name for p in work.iterdir()],['calibration.json'])
                print(json.dumps(dict(writable=writable,backend=backend,frames=count+int(writable),
                    stamp_ns=last_state['stamp_ns'],geometry=latest_map['point_geometry'])),flush=True)
                return last_state
            except (RuntimeError,AssertionError,ValueError) as error:
                log.flush();self.fail(str(error)+'\n'+log_path.read_text()[-2500:])
            finally:
                if client is not None:client.close()
                work.chmod(0o755);log.close()

    def test_singular_map_withholds_output_and_keeps_both_navigation_owners_alive(self):
        full=self.run_case(True,'full_graph_v1')
        paged=self.run_case(True)
        for key in ('position','velocity','orientation_xyzw'):
            np.testing.assert_allclose(full[key],paged[key],rtol=1e-12,atol=1e-12)

    @unittest.skipUnless(os.name=='posix' and os.geteuid()!=0,'Requires ordinary POSIX directory permissions')
    def test_unwritable_map_storage_keeps_navigation_alive(self):
        self.run_case(False)


if __name__=='__main__':unittest.main()
