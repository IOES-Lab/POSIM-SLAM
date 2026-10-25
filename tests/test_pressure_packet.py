import struct
import unittest
from posim_slam.pressure_packet import encode_pressure
from posim_slam.live_client import LiveClient


class PressurePacketTests(unittest.TestCase):
    def test_requires_negotiation_and_preserves_exact_stamp_and_si_units(self):
        client=LiveClient.__new__(LiveClient)
        with self.assertRaisesRegex(ValueError,'worker_pressure_unsupported'):client.pressure(123,-2,.01)
        client.pressure_transport=True;sent=[];client.send=lambda *v:sent.append(v)
        client.pressure(123,-2,.01)
        self.assertEqual(sent,[(10,struct.pack('<qdd',123,-2,.01))])

    def test_bad_time_height_and_unknown_variance(self):
        for args in [(True,0,1),(0,0,1),(2**63,0,1),(123,True,1),(123,float('nan'),1),
                     (123,20001,1),(123,0,0),(123,0,-1),(123,0,float('inf'))]:
            with self.subTest(args=args),self.assertRaises(ValueError):encode_pressure(*args)


if __name__=='__main__':unittest.main()
