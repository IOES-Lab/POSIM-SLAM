"""Counter-evidence for retained coordinates, including retired optimizer IDs."""
import json, os, subprocess, tempfile, unittest
from pathlib import Path


@unittest.skipUnless(os.environ.get('POSIM_SLAM_ARCHIVE_OBSERVATION_PROBE'),
                     'compiled native archive observation probe required')
class ArchiveObservationTests(unittest.TestCase):
    def test_current_observations_retirement_occlusion_and_fresh_support(self):
        config=dict(schema='posim-slam-vio-recording-1',orientation_used=False,
            width=800,height=600,max_image_pixels=120000,fx=600.,fy=610.,cx=399.5,cy=299.5,
            T_i_c_q_xyzw=[0,0,.7071067811865475,.7071067811865476],
            T_i_c_translation_m=[.1,-.2,.05],imu_rate_hz=100,
            gyro_noise_density=.001,accel_noise_density=.02,
            gyro_bias_random_walk=.0001,accel_bias_random_walk=.001,
            gravity_m_s2=9.80665,max_imu_gap_s=.02)
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'calibration.json';path.write_text(json.dumps(config))
            result=subprocess.run([os.environ['POSIM_SLAM_ARCHIVE_OBSERVATION_PROBE'],str(path)],
                capture_output=True,text=True,check=True,timeout=10)
        self.assertIn('retirement, occlusion and fresh admission controls passed',result.stdout)


if __name__=='__main__':unittest.main()
