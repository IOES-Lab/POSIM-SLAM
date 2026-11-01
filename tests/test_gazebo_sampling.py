import unittest
import hashlib,json
import numpy as np
from posim_slam.gazebo_sampling import cubemap_rays,dvl_beam_rays
from posim_slam.inertial import exp_so3
from posim_slam import resources


def geometry():
    # Preserve the retained original calibration values. Discrete footprint
    # truncation must not be reconstructed from rounded/transformed axes.
    axes=np.array([[-.35355339059327373,.3535533905932737,-.8660254037844387],
        [.3535533905932737,.35355339059327373,-.8660254037844387],
        [-.35355339059327373,-.3535533905932737,-.8660254037844387],
        [.3535533905932737,-.35355339059327373,-.8660254037844387]])
    return dict(axes=axes,apertures_rad=np.full(4,np.deg2rad(2.)),scan_spacing_m_at_1m=.01,
        reference_to_sensor=np.eye(3),gz_sensors_version='10.1.1',gz_rendering_version='10.0.2')


class GazeboSamplingTests(unittest.TestCase):
    def test_wire_angle_storage_preserves_unit_directions_and_source_texels(self):
        azimuth=np.linspace(-np.pi/3,np.pi/3,256,dtype=np.float32)
        elevation=np.linspace(-np.pi/18,np.pi/18,256,dtype=np.float32)
        actual,texels=cubemap_rays(azimuth,elevation,256)
        reference,reference_texels=cubemap_rays(azimuth.astype(float),elevation.astype(float),256)
        self.assertEqual(actual.dtype,np.dtype('float64'))
        np.testing.assert_array_equal(texels,reference_texels)
        np.testing.assert_array_equal(actual,reference)
        np.testing.assert_allclose(np.linalg.norm(actual,axis=1),1.,rtol=0,atol=1e-15)

    def test_runtime_adaptation_notice_is_packaged_and_hash_bound(self):
        record=json.loads((resources.NOTICES/'gazebo-sampling.json').read_text())
        self.assertEqual(hashlib.sha256((resources.NOTICES/record['notice']).read_bytes()).hexdigest(),record['notice_sha256'])
        text=(resources.NOTICES/record['notice']).read_text()
        for fragment in ['Copyright 2018 Open Source Robotics Foundation','Copyright 2022 Open Source Robotics Foundation','Apache License']:
            self.assertIn(fragment,text)
        self.assertEqual(record['sources']['gz_sensors']['sha256'],'a070ffe031a4c25d0dcc8c91252c0759c3aafaf35f396a56a6ba95ed2160ea86')

    def test_fixed_source_dimensions_directions_and_reference_frame_equivariance(self):
        inputs=geometry();out=dvl_beam_rays(**inputs)
        self.assertEqual((out['width'],out['height'],out['cube_size']),(83,77,128))
        self.assertEqual([len(row['ray_directions']) for row in out['beams']],[6,6,5,5])
        q=exp_so3([.4,-.2,.3]);rotated=dvl_beam_rays(**dict(inputs,reference_to_sensor=q))
        for a,b in zip(out['beams'],rotated['beams']):
            np.testing.assert_array_equal(a['scan_indices'],b['scan_indices'])
            np.testing.assert_array_equal(a['texels'],b['texels'])
            np.testing.assert_allclose(a['ray_directions']@q,b['ray_directions'],atol=2e-15)
            np.testing.assert_allclose(np.linalg.norm(a['ray_directions'],axis=1),1.,atol=1e-15)
        self.assertFalse(out['variance_inferred']);self.assertFalse(out['map_qualified'])

    def test_retained_flat_source_prediction_without_fitted_offset(self):
        inputs=geometry();out=dvl_beam_rays(**inputs)
        # Retained native stationary fixture: sensor origin 1.38 m over the floor.
        expected=[1.5840002298355103,1.5840002298355103,1.5840001106262207,1.5840002298355103]
        for row,recorded in zip(out['beams'],expected):
            predicted=np.min(-1.38/row['ray_directions'][:,2])
            self.assertLess(abs(predicted-recorded),3e-7)

    def test_unqualified_versions_nonunit_axes_and_unbounded_workspaces_fail(self):
        inputs=geometry()
        for patch in [dict(gz_sensors_version='10.1.2'),dict(gz_rendering_version='unknown'),
            dict(axes=inputs['axes']*2),dict(scan_spacing_m_at_1m=1e-10),dict(scan_spacing_m_at_1m=True),
            dict(apertures_rad=np.zeros(4)),dict(reference_to_sensor=np.zeros((3,3)))]:
            with self.subTest(patch=patch),self.assertRaises(ValueError):dvl_beam_rays(**dict(inputs,**patch))
        for az,el,size in [([0.],[0.],True),([np.nan],[0.],128),([4.],[0.],128),([0.],[2.],128),([0.],[0.],129)]:
            with self.assertRaises(ValueError):cubemap_rays(az,el,size)


if __name__=='__main__':unittest.main()
