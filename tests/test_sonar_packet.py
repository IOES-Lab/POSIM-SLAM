import copy
import json
import os
import struct
import subprocess
import unittest
import numpy as np
from posim_slam.sonar_packet import encode_snapshot, MAX_PAYLOAD


class SonarPacket(unittest.TestCase):
    def tracks(self):
        noise=np.eye(4)*.0004
        noise[0,2]=noise[2,0]=.0001
        return [dict(id=2**53+7, observations=[
            dict(stamp_ns=100,range_m=3.25,azimuth_rad=-.2),
            dict(stamp_ns=200,range_m=3.15,azimuth_rad=-.1)], covariance=noise.tolist())]

    def test_sizes_and_integer_precision(self):
        payload=encode_snapshot(200,self.tracks())
        self.assertEqual(len(payload),16+16+2*24+4*4*8)
        self.assertEqual(struct.unpack_from('<q',payload,16)[0],2**53+7)
        self.assertLess(len(payload),MAX_PAYLOAD)

    def test_input_rejection(self):
        for mutation in ('repeat_stamp','image_stamp','nan','asymmetric','indefinite','zero','bool_id'):
            tracks=self.tracks();track=tracks[0]
            if mutation=='repeat_stamp':track['observations'][0]['stamp_ns']=200
            if mutation=='image_stamp':track['observations'][-1]['stamp_ns']=199
            if mutation=='nan':track['observations'][-1]['range_m']=float('nan')
            if mutation=='asymmetric':track['covariance'][0][1]=.1
            if mutation=='indefinite':track['covariance'][0][2]=track['covariance'][2][0]=1
            if mutation=='zero':track['covariance'][0][0]=0
            if mutation=='bool_id':track['id']=True
            with self.subTest(mutation=mutation),self.assertRaises(ValueError):encode_snapshot(200,tracks)
        with self.assertRaises(ValueError):encode_snapshot(200,self.tracks()*2)

    @unittest.skipUnless(os.environ.get('NATIVE_SONAR_PACKET_PROBE'),'native decoder not configured')
    def test_native_roundtrip_and_malformed_payloads(self):
        binary=os.environ['NATIVE_SONAR_PACKET_PROBE'];tracks=self.tracks()
        second=copy.deepcopy(tracks[0]);second['id']+=1;second['observations'][0]['range_m']=7.1
        tracks.append(second)
        payload=encode_snapshot(200,tracks)
        result=subprocess.run([binary],input=payload,capture_output=True,check=True)
        self.assertEqual(json.loads(result.stdout),dict(stamp_ns=200,tracks=tracks))
        bad=[payload[:-1],payload+b'\0']
        for offset,fmt,value in [(8,'<I',0),(12,'<I',1),(16,'<Q',2**63),
                                  (32,'<q',201),(40,'<d',float('nan')),
                                  (80,'<d',-.1)]:
            data=bytearray(payload);struct.pack_into(fmt,data,offset,value);bad.append(bytes(data))
        indefinite=bytearray(payload)
        struct.pack_into("<d",indefinite,96,1.)
        struct.pack_into("<d",indefinite,144,1.)
        bad.append(bytes(indefinite))
        for data in bad:
            with self.subTest(length=len(data)):
                result=subprocess.run([binary],input=data,capture_output=True)
                self.assertNotEqual(result.returncode,0)

if __name__=='__main__':unittest.main()
