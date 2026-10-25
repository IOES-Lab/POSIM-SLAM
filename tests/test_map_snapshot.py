import struct,unittest
from posim_slam.map_snapshot import annotate_snapshot,validate_snapshot,sonar_snapshot


def raw(points=1001):
    return (f'ply\nformat binary_little_endian 1.0\ncomment scope complete_active_local_vio_window\n'
        f'comment request_id 4\ncomment stamp_ns 100\nelement vertex {points}\n'
        'property float x\nproperty float y\nproperty float z\nend_header\n').encode()+struct.pack('<fff',1,2,3)*points


class SnapshotTests(unittest.TestCase):
    def test_sonar_export_preserves_hypothesis_scope_and_full_archive(self):
        data=sonar_snapshot([[1,2,3]]*1001,request_id=4,stamp_ns=100)
        data=annotate_snapshot(data,nonce='a',frame='vehicle/sonar_inertial_a_segment_1',
                              request_id=4,stamp_ns=100,points=1001)
        value=validate_snapshot(data,dict(scope='accumulated_local_sonar_hypotheses',map_qualified='false'))
        self.assertEqual(value['points'],1001)
        with self.assertRaisesRegex(ValueError,'provenance'):
            validate_snapshot(data,dict(scope='accumulated_local_vio_landmarks'))
        for points in ([],[[float('nan'),0,0]],[[1,2]],[[1,2,3]]*100001):
            with self.assertRaises(ValueError):sonar_snapshot(points,request_id=4,stamp_ns=100)

    def test_complete_snapshot_is_not_limited_by_the_display_sample(self):
        data=annotate_snapshot(raw(),nonce='a',frame='vehicle/vio_a_segment_1',request_id=4,stamp_ns=100,points=1001)
        self.assertEqual(validate_snapshot(data,dict(session_nonce='a'))['points'],1001)
        self.assertIn(b'comment navigation_ready false\n',data)

    def test_accumulated_scope_is_preserved_and_cannot_masquerade_as_active_window(self):
        data=raw(1).replace(b'complete_active_local_vio_window',b'accumulated_local_vio_landmarks')
        result=annotate_snapshot(data,nonce='a',frame='vehicle/vio_a_segment_1',request_id=4,stamp_ns=100,points=1)
        self.assertEqual(validate_snapshot(result,{})['scope'],'accumulated_local_vio_landmarks')
        with self.assertRaisesRegex(ValueError,'provenance'):
            validate_snapshot(result,dict(scope='complete_active_local_vio_window'))

    def test_previous_request_and_cross_session_results_are_rejected(self):
        with self.assertRaisesRegex(ValueError,'provenance'):
            annotate_snapshot(raw(),nonce='a',frame='vehicle/vio_a_segment_1',request_id=5,stamp_ns=100,points=1001)
        data=annotate_snapshot(raw(),nonce='a',frame='vehicle/vio_a_segment_1',request_id=4,stamp_ns=100,points=1001)
        for expected in (dict(session_nonce='b'),dict(frame_id='vehicle/vio_a_segment_2'),dict(stamp_ns=101)):
            with self.assertRaisesRegex(ValueError,'provenance'):validate_snapshot(data,expected)

    def test_partial_or_nonfinite_geometry_and_duplicate_metadata_are_rejected(self):
        data=raw(1)
        for invalid in (data[:-1],data[:-12]+struct.pack('<fff',float('nan'),0,0),
                        data.replace(b'comment request_id 4\n',b'comment request_id 4\ncomment request_id 4\n')):
            with self.subTest(size=len(invalid)),self.assertRaises(ValueError):validate_snapshot(invalid,{})


if __name__=='__main__':unittest.main()
