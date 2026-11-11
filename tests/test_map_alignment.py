import copy
import io
import json
import math
import os
import subprocess
import queue
import threading
from types import SimpleNamespace
import unittest
import numpy as np
from posim_slam.map_alignment import CONTRACT,MapAlignment,_rotation
from posim_slam.live_client import LiveClient


def pair(stamp=100,revision=1,anchor=80,transform=None):
    state=dict(type='state',stamp_ns=stamp,position=[1.,2.,3.],orientation_xyzw=[0.,0.,0.,1.])
    alignment=dict(schema=CONTRACT,epoch='e',optimization_revision=revision,anchor_stamp_ns=anchor,
        map_from_local=(np.eye(4) if transform is None else transform).tolist(),source='pinned_mapping_input_pose',
        uncertainty_available=False,navigation_ready=False)
    return state,dict(type='map',stamp_ns=stamp,mapping_alignment=alignment,
        point_geometry=dict(optimization_revision=revision,optimized_through_ns=anchor,error=None))


class MapAlignmentTests(unittest.TestCase):
    def test_rigid_composition_preserves_local_input_and_no_covariance_claim(self):
        c=MapAlignment('e');t=np.eye(4);t[:3,:3]=[[0,-1,0],[1,0,0],[0,0,1]];t[:3,3]=[.4,.15,0]
        state,packet=pair(transform=t);original=copy.deepcopy((state,packet))
        out=c.paired_pose(state,packet)
        np.testing.assert_allclose(out['position'],[-1.6,1.15,3])
        np.testing.assert_allclose(_rotation(out['orientation_xyzw']),t[:3,:3],atol=1e-14)
        self.assertEqual((state,packet),original)
        self.assertIs(out['navigation_ready'],False);self.assertIs(out['uncertainty_available'],False)
        self.assertNotIn('covariance',out)

    def test_quaternion_branches_include_half_turns_and_nonidentity_local_orientation(self):
        for axis in ([1,0,0],[0,1,0],[0,0,1],[1,2,3]):
            axis=np.asarray(axis,dtype=float);axis/=np.linalg.norm(axis)
            for angle in (0,.2,math.pi-1e-10,math.pi,math.pi+1e-10):
                q=np.r_[axis*np.sin(angle/2),np.cos(angle/2)]
                transform=np.eye(4);transform[:3,:3]=_rotation(q)
                state,packet=pair(transform=transform);state['orientation_xyzw']=[.5,.5,.5,.5]
                out=MapAlignment('e').paired_pose(state,packet)
                np.testing.assert_allclose(_rotation(out['orientation_xyzw']),transform[:3,:3]@_rotation(state['orientation_xyzw']),atol=1e-14)

    def test_revision_first_seen_is_causal_immutable_and_epoch_scoped(self):
        c=MapAlignment('e');first=c.paired_pose(*pair())
        self.assertEqual(first['alignment_observed_stamp_ns'],100)
        second=c.paired_pose(*pair(stamp=110));self.assertEqual(second['alignment_observed_stamp_ns'],100)
        third=c.paired_pose(*pair(stamp=120,revision=2,anchor=105))
        self.assertEqual(third['alignment_observed_stamp_ns'],120)
        for bad in (pair(stamp=119,revision=2,anchor=105),pair(stamp=130,revision=1),pair(stamp=140,revision=3,anchor=141)):
            with self.assertRaises(ValueError):c.paired_pose(*bad)
        with self.assertRaisesRegex(ValueError,'contract'):MapAlignment('another').paired_pose(*pair())

    def test_same_revision_cannot_change_transform_or_anchor(self):
        for kind in ('transform','anchor'):
            c=MapAlignment('e');c.paired_pose(*pair());state,packet=pair(stamp=110)
            if kind=='transform':packet['mapping_alignment']['map_from_local'][0][3]=.1
            else:
                packet['mapping_alignment']['anchor_stamp_ns']=81;packet['point_geometry']['optimized_through_ns']=81
            with self.assertRaisesRegex(ValueError,'revision_changed'):c.paired_pose(state,packet)

    def test_unavailable_and_faulted_maps_never_invent_identity_or_reuse_old_correction(self):
        c=MapAlignment('e');state,packet=pair();packet['mapping_alignment']=None
        self.assertIsNone(c.paired_pose(state,packet))
        self.assertIsNotNone(c.paired_pose(*pair(stamp=110)))
        state,packet=pair(stamp=120);packet['point_geometry']['error']='mapping_limit'
        self.assertIsNone(c.paired_pose(state,packet));self.assertIsNone(c.transform)

    def test_bad_geometry_and_ready_claims_fail_closed(self):
        for kind in ('scale','reflection','nan','lastrow','epoch','revision','qualified','shape','pair'):
            state,packet=pair();a=packet['mapping_alignment']
            if kind=='scale':a['map_from_local'][0][0]=2
            if kind=='reflection':a['map_from_local'][0][0]=-1
            if kind=='nan':a['map_from_local'][0][3]=float('nan')
            if kind=='lastrow':a['map_from_local'][3][0]=1
            if kind=='epoch':a['epoch']='other'
            if kind=='revision':a['optimization_revision']=True
            if kind=='qualified':a['navigation_ready']=True
            if kind=='shape':a['map_from_local']=[]
            if kind=='pair':packet['stamp_ns']=99
            with self.subTest(kind=kind),self.assertRaises(ValueError):MapAlignment('e').paired_pose(state,packet)

    def test_live_client_requires_ack_and_never_applies_alignment_in_reader(self):
        c=object.__new__(LiveClient);c.map_alignment_transport=False;c.map_alignment=None;c.map_alignment_epoch='e';c.processing_request={}
        c.fault=None;c.ready=threading.Event();c.ready.set()
        self.assertIsNone(c.map_aligned_pose(*pair()));self.assertIsNone(c.map_alignment)
        c.map_alignment_transport=True
        self.assertEqual(c.map_aligned_pose(*pair())['stamp_ns'],100)
        other=object.__new__(LiveClient);other.map_alignment_transport=True;other.map_alignment=None;other.map_alignment_epoch='other';other.processing_request={}
        other.fault=None;other.ready=threading.Event();other.ready.set()
        with self.assertRaises(ValueError):other.map_aligned_pose(*pair(stamp=110))
        c.fault='worker_closed'
        with self.assertRaises(RuntimeError):c.map_aligned_pose(*pair(stamp=110))

    def test_ready_binds_worker_generated_epoch_and_rejects_conflicting_epoch(self):
        for expected,actual,accepted in ((None,'generated',True),('e','e',True),('e','other',False),(None,None,False)):
            c=object.__new__(LiveClient);c.closed=threading.Event();c.ready=threading.Event()
            c.output=queue.Queue(maxsize=8);c.output_notification=None;c.fault=None;c.map_alignment=None
            c.processing_request={} if expected is None else {'map_epoch':expected}
            ready=dict(type='ready',protocol=1,inertial_fused=True,map_alignment_transport=CONTRACT,mapping_epoch=actual)
            c.process=SimpleNamespace(stdout=io.BytesIO((json.dumps(ready)+'\n').encode()))
            c._read()
            self.assertEqual(c.ready.is_set(),accepted);self.assertIsNone(c.map_alignment)
            if accepted:self.assertEqual(c.map_alignment_epoch,actual)
            else:self.assertIn('worker_map_alignment_epoch',c.fault)

    @unittest.skipUnless(os.environ.get('POSIM_SLAM_ALIGNMENT_PROBE'),'native probe not supplied')
    def test_native_producer_and_python_consumer_agree(self):
        alignment=json.loads(subprocess.check_output([os.environ['POSIM_SLAM_ALIGNMENT_PROBE']],text=True))
        state,packet=pair(stamp=180900000001,revision=222,anchor=180900000000)
        alignment['epoch']='e';packet['mapping_alignment']=alignment
        out=MapAlignment('e').paired_pose(state,packet)
        theta=-.2;r=np.array([[np.cos(theta),-np.sin(theta),0],[np.sin(theta),np.cos(theta),0],[0,0,1]])
        np.testing.assert_allclose(out['position'],r@state['position']+[.4,.15,-.01],atol=1e-14)


if __name__=='__main__':unittest.main()
