"""Pressure/attitude-derived surface plane, including arbitrary map orientation."""
from pathlib import Path
import sys
import unittest
import numpy as np
from posim_slam.surface_calibration import SurfaceCalibration

class CalibrationTests(unittest.TestCase):
    def test_moving_camera_and_pressure_lever_arm(self):
        c=SurfaceCalibration(np.eye(3),[0,0,-.2])
        for i in range(8):
            depth=2+i*.1;t=i*.2
            plane=c.add(t,[i*.3,0,-depth],np.eye(3),t,101325+(depth+.2)*1025*9.80665,1,t,[0,0,0,1])
        np.testing.assert_allclose(plane,[0,0,1,0],atol=1e-12)

    def test_rotated_map_and_imu_mount(self):
        # Camera/map rotate world up to +x; pressure plane is x=5.
        rotation=np.array([[0,0,1],[0,1,0],[-1,0,0]])
        mount=np.diag([1,-1,-1])
        c=SurfaceCalibration(mount,[0,0,0])
        for i in range(8):
            t=i*.2
            plane=c.add(t,[3,i,7],rotation,t,101325+2*1025*9.80665,1,t,[1,0,0,0])
        np.testing.assert_allclose(plane,[1,0,0,-5],atol=1e-12)

    def test_stale_invalid_and_time_reset(self):
        c=SurfaceCalibration(np.eye(3),[0,0,0])
        for i in range(8):c.add(i*.2,[0,0,-2],np.eye(3),i*.2,121428.6325,1,i*.2,[0,0,0,1])
        self.assertIsNotNone(c.plane)
        with self.assertRaises(ValueError):c.add(2,[0,0,-2],np.eye(3),1,121428.,1,2,[0,0,0,1])
        self.assertIsNone(c.plane)
        self.assertIsNone(c.add(0,[0,0,-2],np.eye(3),0,121428.,1,0,[0,0,0,1]))
        with self.assertRaises(ValueError):c.add(.2,[0,0,-2],np.eye(3),.2,121428.,1,.2,[0,0,0,0])

    def test_unstable_pressure_does_not_calibrate(self):
        c=SurfaceCalibration(np.eye(3),[0,0,0])
        for i in range(30):
            t=i*.2;pressure=101325+(2+(i%2)*.3)*1025*9.80665
            self.assertIsNone(c.add(t,[0,0,-2],np.eye(3),t,pressure,1,t,[0,0,0,1]))

if __name__=='__main__':unittest.main()
