"""A covering cell keeps its physical coordinate, instead of extending the clip."""
import math
import unittest
import numpy as np
from posim_slam.sonar_input import validate_calibrated_frame


def sample(maximum=10.,bandwidth=29900.,speed=1500.,beams=32):
    count=math.ceil(2*maximum*bandwidth/speed)+1
    # The producer rounds delta_t before computing and publishing each range.
    dt=np.float32(1/bandwidth)
    ranges=np.asarray(dt*np.arange(count,dtype=np.float32)*speed/2,dtype=np.float32)
    az=np.linspace(-.6,.6,beams)
    value=dict(intensity_db=np.full((count,beams),-np.inf,dtype=np.float32),ranges=ranges,
        beam_directions=np.column_stack((np.cos(az),np.sin(az),az*0)),stamp=np.array([1,0]))
    calibration=dict(beams=beams,horizontal_fov=1.2,range_max=maximum,
        bandwidth_hz=bandwidth,sound_speed_m_s=speed)
    return value,calibration


class CalibratedFrameTests(unittest.TestCase):
    def test_actual_grid_covers_ten_and_fifteen_metres_without_clamping(self):
        for maximum,count in ((10.,400),(10.02,401),(15.,599),(7.123,285)):
            with self.subTest(maximum=maximum):
                value,c=sample(maximum)
                self.assertEqual(len(value['ranges']),count)
                original=value['ranges'].copy();validate_calibrated_frame(value,**c)
                np.testing.assert_array_equal(value['ranges'],original)
                self.assertGreaterEqual(float(value['ranges'][-1])+2e-6,maximum)
        value,c=sample();self.assertGreater(float(value['ranges'][-1]),c['range_max'])

    def test_wrong_counts_stretched_clamped_and_corrupted_axes_are_errors(self):
        for kind in ('missing','extra','stretched','clamped','jitter','bearing'):
            value,c=sample()
            if kind=='missing':
                value['intensity_db']=value['intensity_db'][:-1];value['ranges']=value['ranges'][:-1]
            elif kind=='extra':
                value['intensity_db']=np.r_[value['intensity_db'],value['intensity_db'][:1]]
                value['ranges']=np.r_[value['ranges'],value['ranges'][-1]*2]
            elif kind=='stretched':value['ranges']=np.linspace(0,10,400)
            elif kind=='clamped':value['ranges'][-1]=10.
            elif kind=='jitter':value['ranges'][150]+=.001
            else:value['beam_directions'][15]=[1.,0.,0.]
            with self.subTest(kind=kind),self.assertRaises(ValueError):validate_calibrated_frame(value,**c)

    def test_invalid_calibration_and_backend_shape_limit_are_not_clamped(self):
        value,c=sample()
        for name in ('horizontal_fov','range_max','bandwidth_hz','sound_speed_m_s'):
            for invalid in (0.,-1.,np.nan,np.inf):
                with self.subTest(name=name,value=invalid),self.assertRaises(ValueError):
                    validate_calibrated_frame(value,**{**c,name:invalid})
        with self.assertRaises(ValueError):validate_calibrated_frame(value,**{**c,'range_max':1e308})
        value,c=sample(4095*1500/(2*29900),beams=16)
        validate_calibrated_frame(value,**c)
        with self.assertRaises(ValueError):validate_calibrated_frame(value,**{**c,'range_max':4096*1500/(2*29900)})


if __name__=='__main__':unittest.main()
