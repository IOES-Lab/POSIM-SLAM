import struct
import unittest
import numpy as np
from posim_slam.dvl_packet import encode_dvl
from posim_slam.live_client import LiveClient


class DvlPacketTests(unittest.TestCase):
    def values(self):
        return dict(stamp_ns=123, velocity=[.1,.2,.3], covariance=np.eye(3)*.01,
                    gyro=[.01,.02,.03], gyro_stamp_ns=123, gyro_covariance=np.eye(3)*.001,
                    bottom_locked=True, locked_beams=4)

    def test_full_covariance_row_major_packet(self):
        values=self.values();values['covariance'][0,1]=values['covariance'][1,0]=.002
        packet=encode_dvl(**values)
        self.assertEqual(len(packet),216)
        unpacked=struct.unpack('<qqII24d',packet)
        self.assertEqual(unpacked[:4],(123,123,1,4))
        np.testing.assert_equal(unpacked[10:19],values['covariance'].ravel())
        np.testing.assert_equal(unpacked[19:],values['gyro_covariance'].ravel())

    def test_bad_lock_clock_vectors_covariances(self):
        for key,value in [('stamp_ns',True),('stamp_ns',2**63),('gyro_stamp_ns',124),
                          ('bottom_locked',False),('bottom_locked',1),('locked_beams',2),
                          ('velocity',[0,0,float('nan')]),('velocity',[True]*3),
                          ('gyro',[0,0,36]),('covariance',np.zeros((3,3))),
                          ('covariance',[[1,.1,0],[0,1,0],[0,0,1]]),
                          ('gyro_covariance',-np.eye(3)),('covariance',[1]*9)]:
            with self.subTest(key=key,value=value):
                values=self.values();values[key]=value
                with self.assertRaises(ValueError):encode_dvl(**values)

    def test_requires_native_capability_and_forwards_exact_packet(self):
        client=LiveClient.__new__(LiveClient);client.dvl_transport=False
        with self.assertRaisesRegex(ValueError,'worker_dvl_unsupported'):client.dvl(**self.values())
        client.dvl_transport=True;packets=[];client.send=lambda *row:packets.append(row)
        client.dvl(**self.values())
        self.assertEqual(packets,[(9,encode_dvl(**self.values()))])


if __name__=='__main__':unittest.main()
