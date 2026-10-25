import copy
import struct
import unittest
import numpy as np
from posim_slam.live_client import LiveClient
from posim_slam.sonar_ground_packet import encode_ground
from posim_slam.sonar_factor import within_fov,point_return_factor


class GroundPacketTests(unittest.TestCase):
    def setUp(self):
        self.observation=dict(beam_ids=list(range(8)),azimuth_rad=np.linspace(-.5,.5,8),
            range_m=np.ones(8)*4,boundary_sign=-1,normal_sonar=[0,0,1],distance_sonar_m=-2.)

    def test_original_time_identity_and_negotiation(self):
        packet=encode_ground(123,**self.observation)
        self.assertEqual(len(packet),48+8*20)
        self.assertEqual(struct.unpack('<qIi4d',packet[:48]),(123,8,-1,0.,0.,1.,-2.))
        for i in range(8):
            ident,angle,value=struct.unpack_from('<I2d',packet,48+20*i)
            self.assertEqual(ident,i);self.assertEqual(angle,self.observation['azimuth_rad'][i]);self.assertEqual(value,4.)
        client=LiveClient.__new__(LiveClient)
        with self.assertRaises(ValueError):client.ground(123,**self.observation)
        client.ground_transport=True;sent=[];client.send=lambda *a:sent.append(a)
        client.ground(123,**self.observation);self.assertEqual(sent,[(12,packet)])

    def test_invalid_measurements_and_identity(self):
        for change in (dict(beam_ids=[0]*8),dict(beam_ids=[2**31]*8),dict(azimuth_rad=[0]*8),
                       dict(range_m=[float('nan')]*8),dict(boundary_sign=0),dict(normal_sonar=[0,0,0]),
                       dict(distance_sonar_m=float('inf'))):
            with self.subTest(change=change),self.assertRaises(ValueError):encode_ground(123,**dict(self.observation,**change))
        for stamp in (0,-1,True,1.5,2**63):
            with self.subTest(stamp=stamp),self.assertRaises(ValueError):encode_ground(stamp,**self.observation)

    def test_float32_direction_endpoint_keeps_original_angle(self):
        fov=2*np.pi/3;half=np.float32(fov/2)
        direction=np.array([np.cos(half),np.sin(half)],dtype=np.float32).astype(float)
        angle=float(np.arctan2(direction[1],direction[0]))
        self.assertGreater(angle,fov/2);self.assertTrue(within_fov(angle,fov))
        self.assertFalse(within_fov(fov/2+1e-5,fov))
        point=4*np.array([np.cos(angle),np.sin(angle),0.])
        result=point_return_factor(position=[0,0,0],attitude=np.eye(3),imu_to_sonar=[0,0,0],
            sonar_to_imu=np.eye(3),landmark=point,measured_range=4.,measured_azimuth=angle,
            range_min=.2,range_max=10.,horizontal_fov=fov,vertical_fov=.35)
        np.testing.assert_allclose(result['residual'],0.,atol=1e-12)


if __name__=='__main__':unittest.main()
