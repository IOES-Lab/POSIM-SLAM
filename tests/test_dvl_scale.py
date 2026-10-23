"""Analytic sensor trajectories exercise scale, rotation baseline and rejection."""
from pathlib import Path
import sys
import unittest
import numpy as np
from posim_slam.dvl_scale import DvlScaleWindow, BODY_TO_OPTICAL, CAMERA_TO_DVL

class Tests(unittest.TestCase):
    def sample(self,w,t,p,v=(.3,0,0),r=None,**kw):
        return w.add(t,p,np.eye(3) if r is None else r,t,v,np.eye(3)*8e-6,**kw)
    def test_scale_and_eight_independent_intervals(self):
        for scale in [.2,1,2.5,20]:
            w=DvlScaleWindow()
            for i in range(9):
                result=self.sample(w,i,BODY_TO_OPTICAL@np.array([.3*i,0,0])/scale)
                self.assertEqual(result['ready'],i==8)
            self.assertAlmostEqual(result['scale'],scale)
    def test_rotating_sensor_baseline(self):
        w=DvlScaleWindow();p=np.zeros(3);r0=np.eye(3);velocity_map=np.array([.3,.1,0])
        for i in range(9):
            a=.1*i;r=np.array([[np.cos(a),-np.sin(a),0],[np.sin(a),np.cos(a),0],[0,0,1]])
            if i:p+=(velocity_map-(r-r0)@CAMERA_TO_DVL)/2.5
            body_velocity=BODY_TO_OPTICAL.T@r.T@velocity_map
            result=self.sample(w,i,p,body_velocity,r);r0=r
        self.assertTrue(result['ready']);self.assertAlmostEqual(result['scale'],2.5)
    def test_stationary_does_not_establish_scale(self):
        w=DvlScaleWindow()
        for i in range(20):self.assertFalse(self.sample(w,i,[0,0,0],v=[0,0,0])['ready'])
    def test_rejection_clears_previous_evidence(self):
        for invalid in ['loss','gap','reverse','sync','rotation','covariance','direction']:
            w=DvlScaleWindow();self.sample(w,0,[0,0,0]);self.sample(w,1,[0,-.3,0])
            args=[2,[0,-.6,0],np.eye(3),2,[.3,0,0],np.eye(3)*8e-6];kw={}
            if invalid=='loss':kw['valid']=False
            if invalid=='gap':args[0]=args[3]=4
            if invalid=='reverse':args[0]=args[3]=1
            if invalid=='sync':args[3]=1.7
            if invalid=='rotation':args[2]=np.diag([-1,-1,1])
            if invalid=='covariance':args[5]=-np.eye(3)
            if invalid=='direction':args[1]=[1,-.3,0]
            with self.assertRaises(ValueError,msg=invalid):w.add(*args,**kw)
            self.assertIsNone(w.previous);self.assertEqual(len(w.values),0)
    def test_scale_jump_requires_new_consistent_window(self):
        w=DvlScaleWindow()
        for i in range(9):result=self.sample(w,i,[0,-.3*i,0])
        self.assertTrue(result['ready'])
        self.assertFalse(self.sample(w,9,[0,-2.55,0])['ready'])

if __name__=='__main__':unittest.main()
