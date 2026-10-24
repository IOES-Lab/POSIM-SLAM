import unittest
from types import SimpleNamespace as N
import numpy as np
from posim_slam.sonar_input import projected_frame

class SonarInput(unittest.TestCase):
    def message(self,big=False):
        values=np.arange(256,dtype=np.float32).reshape(16,16);values[0,0]=-np.inf
        angles=np.linspace(-1,1,16)
        return N(header=N(stamp=N(sec=3,nanosec=7)),ranges=np.linspace(.1,10,16),
                 beam_directions=[N(x=np.cos(a),y=np.sin(a),z=0) for a in angles],
                 image=N(beam_count=16,dtype=4,DTYPE_FLOAT32=4,is_bigendian=big,
                         data=values.astype('>f4' if big else '<f4').tobytes()))
    def test_byte_orders_preserve_values_and_no_return(self):
        a,b=projected_frame(self.message()),projected_frame(self.message(True))
        np.testing.assert_equal(a['intensity_db'],b['intensity_db'])
        self.assertTrue(np.isneginf(a['intensity_db'][0,0]))
        self.assertEqual(a['intensity_db'][15,15],255)
        self.assertEqual(a['stamp'].tolist(),[3,7])
    def test_truncated_payload_rejected(self):
        m=self.message();m.image.data=m.image.data[:-4]
        with self.assertRaisesRegex(ValueError,'byte_count'):projected_frame(m)
    def test_clock_and_geometry_rejected(self):
        m=self.message();m.header.stamp.nanosec=1_000_000_000
        with self.assertRaisesRegex(ValueError,'timestamp'):projected_frame(m)
        m=self.message();m.beam_directions[0].x=5
        with self.assertRaisesRegex(ValueError,'geometry'):projected_frame(m)
    def test_invalid_intensity_and_oversized_shape(self):
        m=self.message();v=np.zeros((16,16),dtype='<f4');v[0,0]=np.nan;m.image.data=v.tobytes()
        with self.assertRaisesRegex(ValueError,'intensity'):projected_frame(m)
        m=self.message();m.image.beam_count=1000000
        with self.assertRaisesRegex(ValueError,'shape_limit'):projected_frame(m)

if __name__=='__main__':unittest.main()
