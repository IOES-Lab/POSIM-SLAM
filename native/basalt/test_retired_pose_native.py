"""Actual native marginalization capture, independent of field recordings."""
import json
import os
from pathlib import Path
import tempfile
import unittest
import numpy as np
from posim_slam.retired_pose_archive import RetiredPoseArchive
import test_sonar_clock_native as fixture


@unittest.skipUnless(os.environ.get('POSIM_SLAM_STREAM_BINARY'),'Native worker binary required')
class RetiredPoseNativeTests(unittest.TestCase):
    receive=fixture.SonarClockTests.receive

    def test_capture_preserves_trajectory_and_recovers_a_retired_pose(self):
        traces=[];nodes=0;initial=1_000_000_000
        for capture in (False,True):
            with tempfile.TemporaryDirectory() as folder:
                root=Path(folder)
                (root/'pressure-calibration.json').write_text(json.dumps(dict(
                    schema='posim-slam-pressure-calibration-1',imu_to_pressure=[0,0,0])))
                client=fixture.SonarClockTests.worker(self,root,enable_joint_pressure=True,
                    capture_retired_pose_conditionals=capture,conditional_archive_epoch='native-control')
                archive=RetiredPoseArchive('native-control');trace=[];next_imu=initial
                client.gravity_seed(initial,[0,0,1])
                for index in range(28):
                    stamp=initial+index*200_000_000
                    while next_imu<=stamp+10_000_000:
                        client.imu(next_imu,[0,0,0],[0,0,9.80665]);next_imu+=10_000_000
                    client.pressure(stamp,-2.,.01);client.sonar_tick(stamp)
                    state=self.receive(client,'state');trace.append([*state['position'],*state['orientation_xyzw'],*state['velocity']])
                    packet=state['sonar'].get('pose_archive')
                    if capture:
                        self.assertIsNotNone(packet);archive.ingest(packet);nodes+=len(packet['conditionals'])
                    else:self.assertIsNone(packet)
                if capture:
                    self.assertNotIn(initial,state['sonar']['retained_pose_stamps_ns'])
                    result=archive.recover([initial,stamp])
                    self.assertTrue(np.isfinite(result['deltas']).all())
                    self.assertTrue(result['live_coordinates_treated_as_known'])
                    self.assertFalse(result['map_qualified'])
                client.close();traces.append(trace)
        self.assertGreater(nodes,0)
        np.testing.assert_array_equal(np.asarray(traces[0]),np.asarray(traces[1]))


if __name__=='__main__':unittest.main()
