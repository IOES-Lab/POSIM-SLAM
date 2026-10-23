"""Actual native image/calibration boundary and reprojection diagnostics."""
import json,math,os,subprocess,tempfile,unittest
from pathlib import Path


@unittest.skipUnless(os.environ.get('POSIM_SLAM_INPUT_PROBE'),'Native input probe required')
class InputContractTests(unittest.TestCase):
    def probe(self,width,height,budget=None,**intrinsics):
        config=dict(schema='posim-slam-vio-recording-1',orientation_used=False,
            width=width,height=height,fx=1300.,fy=1250.,cx=(width-1)/2,cy=(height-1)/2,
            T_i_c_q_xyzw=[0,0,0,1],T_i_c_translation_m=[0,0,0],imu_rate_hz=100,
            gyro_noise_density=.001,accel_noise_density=.02,gyro_bias_random_walk=.0001,
            accel_bias_random_walk=.001,gravity_m_s2=9.80665,max_imu_gap_s=.02)
        config.update(intrinsics)
        if budget is not None:config['max_image_pixels']=budget
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'calibration.json';path.write_text(json.dumps(config))
            result=subprocess.run([os.environ['POSIM_SLAM_INPUT_PROBE'],str(path)],capture_output=True,text=True,timeout=5)
        if result.returncode:raise ValueError(result.stderr.strip())
        return config,json.loads(result.stdout)

    def test_odd_and_even_camera_sizes_preserve_normalized_rays_within_budget(self):
        for width,height,budget in ((800,600,120000),(803,601,120000),(640,360,70000),(1920,1080,150000)):
            with self.subTest(width=width,height=height):
                c,r=self.probe(width,height,budget)
                self.assertLessEqual(r['width']*r['height'],budget)
                sx,sy=r['width']/width,r['height']/height
                fx,fy,cx,cy=r['intrinsics']
                for axis,scale,focal,center in (('x',sx,fx,cx),('y',sy,fy,cy)):
                    source_focal=c['f'+axis];source_center=c['c'+axis]
                    coordinate=123.25
                    self.assertAlmostEqual((coordinate-source_center)/source_focal,
                        ((coordinate+.5)*scale-.5-center)/focal)

    def test_no_budget_preserves_source_calibration_even_with_off_image_principal_point(self):
        c,r=self.probe(320,241,cx=-2.,cy=0.)
        self.assertEqual((r['width'],r['height']),(320,241))
        self.assertEqual(r['intrinsics'],[c[k] for k in ('fx','fy','cx','cy')])

    def test_pixels_use_area_sampling_and_estimator_uses_same_image(self):
        _,r=self.probe(800,600,120000)
        self.assertEqual((r['width'],r['height']),(400,300))
        self.assertEqual(r['first_pixel'],2)
        self.assertEqual(r['estimator_first_pixel'],2<<8)

    def test_invalid_budget_is_rejected_before_constructing_estimator(self):
        for budget in (True,1023,120000.5,2073601,2**32+120000):
            with self.subTest(budget=budget),self.assertRaisesRegex(ValueError,'invalid_pixel_budget'):
                self.probe(800,600,budget)

    def test_diagnostics_match_optimized_projections_to_current_feature_ids(self):
        _,r=self.probe(320,240)
        self.assertEqual(r['mapping_inliers'],[0])
        support=r['visual_support']
        self.assertEqual(support['reprojected_landmarks'],2)
        self.assertEqual(support['full_weight_reprojections'],1)
        self.assertAlmostEqual(support['reprojection_rmse_px'],math.sqrt(12.5))
        self.assertTrue(support['visual_measurements_present'])
        self.assertFalse(support['observability_qualified'])

    def test_map_exports_landmark_ids_once_despite_repeated_observations(self):
        _,r=self.probe(320,240)
        export=r['map_export']
        self.assertEqual(sorted(export['ids']),[7,42])
        # Different IDs at identical coordinates must remain distinct landmarks.
        self.assertEqual(export['points'],[[0.,0.,2.],[0.,0.,2.]])

    def test_feature_cell_budget_preserves_coverage_across_processing_sizes(self):
        cells=[]
        for width,height,budget in ((800,600,480000),(800,600,120000),(803,601,120000),(1920,1080,120000)):
            _,r=self.probe(width,height,budget,max_feature_cells=200)
            self.assertLessEqual(r['feature_cells'],200)
            self.assertGreaterEqual(r['feature_cells'],170)
            self.assertGreaterEqual(r['feature_grid'],16)
            cells.append(r['feature_cells'])
        self.assertEqual(cells[0],cells[1])

    def test_invalid_feature_budgets_and_unsupported_aspect_ratios_are_rejected(self):
        for cells in (True,31,513,200.5,2**32+200):
            with self.subTest(cells=cells),self.assertRaisesRegex(ValueError,'invalid_feature_budget'):
                self.probe(800,600,120000,max_feature_cells=cells)
        with self.assertRaisesRegex(ValueError,'feature_budget_aspect_ratio'):
            self.probe(1920,64,None,max_feature_cells=32)


if __name__=='__main__':unittest.main()
