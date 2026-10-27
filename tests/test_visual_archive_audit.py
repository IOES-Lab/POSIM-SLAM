import json
import sys
from pathlib import Path
from types import SimpleNamespace
import tempfile
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from audit_visual_archive import audit, track_information


class VisualArchiveAuditTests(unittest.TestCase):
    def scene(self, baseline=.5):
        point = np.array([0., 0., 4.])
        origins = np.array([[-baseline/2, 0., 0.], [baseline/2, 0., 0.]])
        rotations = np.repeat(np.eye(3)[None], 2, axis=0)
        pixels = (point-origins)[:, :2]/4*200
        return point, origins, rotations, pixels, [200, 200, 0, 0]

    def test_rectified_stereo_depth_variance_matches_analytic_derivative(self):
        result = track_information(*self.scene(), .5)
        expected = np.sqrt(2)*.5*4**2/(200*.5)
        self.assertAlmostEqual(result['sigma_position_m'], expected)
        self.assertAlmostEqual(result['baseline_m'], .5)
        self.assertEqual(result['maximum_reprojection_px'], 0.)
        wide = track_information(*self.scene(1.), .5)
        self.assertAlmostEqual(wide['sigma_position_m'], expected/2)

    def test_repeated_view_has_no_depth_information(self):
        result = track_information(*self.scene(0), .5)
        self.assertEqual(result['rank'], 2)
        self.assertIsNone(result['sigma_position_m'])

    def test_pixel_noise_scaling_and_rigid_gauge_preserve_information(self):
        values = list(self.scene())
        base = track_information(*values, .5)
        scaled = track_information(*values, 1.)
        self.assertAlmostEqual(scaled['sigma_position_m'], 2*base['sigma_position_m'])
        angle = .7
        rotation = np.array([[np.cos(angle), 0, np.sin(angle)], [0, 1, 0],
                             [-np.sin(angle), 0, np.cos(angle)]])
        translation = np.array([3., -2., 5.])
        values[0] = rotation @ values[0] + translation
        values[1] = values[1] @ rotation.T + translation
        values[2] = rotation @ values[2]
        changed = track_information(*values, .5)
        self.assertAlmostEqual(changed['sigma_position_m'], base['sigma_position_m'])
        self.assertLess(changed['maximum_reprojection_px'], 1e-10)

    def test_inconsistent_pixels_and_behind_camera_remain_reported(self):
        values = list(self.scene())
        values[3][1, 0] += 4.
        result = track_information(*values, .5)
        self.assertEqual(result['maximum_reprojection_px'], 4.)
        values[0][2] = -4.
        result = track_information(*values, .5)
        self.assertEqual(result['behind_camera'], 2)
        self.assertIsNone(result['sigma_position_m'])

    def test_invalid_geometry_and_noise_are_rejected(self):
        for sigma in (0, -1, float('nan')):
            with self.assertRaisesRegex(ValueError, 'invalid_track'):
                track_information(*self.scene(), sigma)
        values = list(self.scene()); values[2][0, 0, 0] = 2.
        with self.assertRaisesRegex(ValueError, 'invalid_camera_rotation'):
            track_information(*values, .5)

    def test_file_audit_checks_exact_map_identity_and_retains_every_vertex(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            point = np.array([[0, 0, 4]], dtype='<f4')
            (root/'map.ply').write_bytes((
                'ply\nformat binary_little_endian 1.0\n'
                'comment scope accumulated_local_vio_landmarks\ncomment frame_id local\n'
                'comment request_id 1\ncomment stamp_ns 3000000000\n'
                'element vertex 1\nproperty float x\nproperty float y\nproperty float z\nend_header\n'
            ).encode() + point.tobytes())
            archive = dict(schema='posim-visual-archive-trace-1', ids=[17],
                           points=point.tolist(), request_id=1, stamp_ns=3_000_000_000)
            (root/'archive.json').write_text(json.dumps(archive))
            (root/'cal.json').write_text(json.dumps(dict(width=32, height=32,
                fx=200, fy=200, cx=16, cy=16, T_i_c_translation_m=[0, 0, 0],
                T_i_c_q_xyzw=[0, 0, 0, 1])))
            times = [1_000_000_000, 2_000_000_000, 3_000_000_000]
            origins = [-.25, 0., .25]
            (root/'poses.csv').write_text('stamp_ns,x,y,z,qx,qy,qz,qw\n' + ''.join(
                f'{t},{x},0,0,0,0,0,1\n' for t, x in zip(times, origins)))
            rows = [dict(stamp_ns=t, width=32, height=32,
                tracks=[dict(id=17, full_weight=True, pixel=[16-50*x, 16])])
                for t, x in zip(times, origins)]
            (root/'trace.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in rows))
            args = SimpleNamespace(map=root/'map.ply', archive=root/'archive.json',
                calibration=root/'cal.json', trace=root/'trace.jsonl', poses=root/'poses.csv',
                frame='local', pixel_sigma=.5, output=root/'report.json')
            report = audit(args)
            self.assertEqual(report['points'], 1)
            self.assertEqual(report['rows'][0]['exposures'], 3)
            self.assertFalse(report['map_filtered'])
            self.assertFalse(report['map_qualified'])
            self.assertEqual(report['missing_pose_observations'], 0)
            args.frame = 'other_segment'
            with self.assertRaisesRegex(ValueError, 'provenance'):
                audit(args)
            args.frame = 'local'; archive['points'][0][2] = 5
            (root/'archive.json').write_text(json.dumps(archive))
            with self.assertRaisesRegex(ValueError, 'points_differ'):
                audit(args)


if __name__ == '__main__':
    unittest.main()
