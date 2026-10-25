import struct
import unittest
from posim_slam.ping_packet import encode_ping
from posim_slam.live_client import LiveClient


class PingPacketTests(unittest.TestCase):
    def test_negotiated_original_stamp_scalar_range(self):
        client=LiveClient.__new__(LiveClient)
        with self.assertRaisesRegex(ValueError,'worker_terrain_ping_unsupported'):client.terrain_ping(123,2.)
        client.terrain_ping_transport=True;sent=[];client.send=lambda *args:sent.append(args)
        client.terrain_ping(123,2.)
        self.assertEqual(sent,[(11,struct.pack('<qd',123,2.))])

    def test_invalid_stamp_or_nonfinite_range(self):
        for args in [(True,2.),(0,2.),(2**63,2.),(1,True),(1,0),(1,-1),(1,float('nan')),(1,float('inf'))]:
            with self.subTest(args=args),self.assertRaises(ValueError):encode_ping(*args)


if __name__=='__main__':unittest.main()
