import unittest
from posim_slam.exact_pair import ExactImagePing


class PairTests(unittest.TestCase):
    def test_both_arrival_orders_preserve_exact_timestamp_and_image_order(self):
        for first in ('image','ping'):
            b=ExactImagePing()
            if first=='image':b.image(100,'a',0);b.ping(100,2.)
            else:b.ping(100,2.);b.image(100,'a',0)
            self.assertEqual(b.drain(.01),[('a',(100,2.))])
        b=ExactImagePing();b.image(100,'a',0);b.image(200,'b',.01);b.ping(200,3.)
        self.assertEqual(b.drain(.02),[])
        self.assertEqual(b.drain(.16),[('a',None),('b',(200,3.))])

    def test_nearby_and_late_samples_never_retime_and_invalid_return_is_missing(self):
        b=ExactImagePing();b.image(100,'a',0);b.ping(99,2.)
        self.assertEqual(b.drain(.16),[('a',None)])
        b.ping(100,2.);self.assertEqual(b.late,1)
        b.ping(200,None);b.image(200,'b',.2)
        self.assertEqual(b.drain(.2),[('b',None)])
        self.assertEqual((b.matched,b.unpaired),(0,2))

    def test_bounded_inputs_and_clock_reversal(self):
        b=ExactImagePing()
        for i in range(4):b.image(i+1,i,0)
        with self.assertRaisesRegex(ValueError,'backlog'):b.image(5,None,0)
        b=ExactImagePing()
        for i in range(64):b.ping(i+1,2.)
        with self.assertRaisesRegex(ValueError,'backlog'):b.ping(65,2.)
        with self.assertRaisesRegex(ValueError,'clock'):b.ping(60,2.)
        self.assertFalse(ExactImagePing().pings)

if __name__=='__main__':unittest.main()
