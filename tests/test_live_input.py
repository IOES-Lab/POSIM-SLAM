import io,json,queue,struct,threading,unittest
from pathlib import Path
import tempfile
import numpy as np
from posim_slam.image_input import gray8
from posim_slam.live_client import LiveClient,WorkerNotReady


class LiveInputTests(unittest.TestCase):
    def client(self):
        c=LiveClient.__new__(LiveClient);c.closed=threading.Event();c.fault=None
        c.processing_request={}
        c.input=queue.Queue(maxsize=256);c.output=queue.Queue(maxsize=8)
        c.budget_lock=threading.Lock();c.pending_bytes=0
        c.ready=threading.Event();c.ready.set()
        c.surface_transport=False;c.surface_directory=None;c._surface_files={};c._surface_request_id=0
        c._surface_lock=threading.Lock()
        return c

    def test_surface_file_conflicts_preserve_existing_files_and_queue_only_metadata(self):
        from posim_slam.native_surface import NativeSurfaceSnapshot,prepare_native_ranges
        from test_delayed_surface_update import physical_case
        from test_native_surface import snapshot,arguments
        _,archive,kwargs,*_=physical_case()
        increment=prepare_native_ranges(NativeSurfaceSnapshot.read(snapshot(archive)),**arguments(kwargs))
        c=self.client();c.surface_transport=True
        with tempfile.TemporaryDirectory() as folder:
            c.surface_directory=Path(folder).resolve()
            for request,suffix in ((1,'.json'),(2,'.tmp')):
                existing=c.surface_directory/('worker-surface-update-'+str(request)+suffix)
                existing.write_bytes(b'preserve existing file')
                with self.assertRaises((ValueError,FileExistsError)):c.surface_update(request,increment)
                self.assertEqual(existing.read_bytes(),b'preserve existing file')
                self.assertEqual(c._surface_files,{});self.assertEqual(c.pending_bytes,0)
            c.surface_update(3,increment)
            packet=c.input.get_nowait();kind,size=struct.unpack('<II',packet[:8])
            self.assertEqual(kind,15);self.assertLess(size,4096)
            metadata=json.loads(packet[8:]);self.assertEqual(metadata['op'],'update')
            self.assertNotIn('jacobian',metadata)
            proposal=c._surface_files[3]
            self.assertEqual(metadata['bytes'],proposal.stat().st_size)
            self.assertNotIn('prior',json.loads(proposal.read_text()))

    def test_sonar_transport_requires_capability_and_preserves_payload(self):
        from posim_slam.sonar_packet import encode_snapshot
        c=self.client()
        tracks=[dict(id=1,observations=[dict(stamp_ns=100,range_m=4.,azimuth_rad=.1)],covariance=[[.01,0],[0,.001]])]
        with self.assertRaisesRegex(ValueError,'unsupported'):c.sonar_snapshot(100,tracks)
        self.assertEqual(c.pending_bytes,0)
        c.sonar_transport=True;c.sonar_snapshot(100,tracks)
        packet=c.input.get_nowait();kind,size=struct.unpack('<II',packet[:8])
        self.assertEqual(kind,7);self.assertEqual(size,len(packet)-8)
        self.assertEqual(packet[8:],encode_snapshot(100,tracks))

    def test_auxiliary_sonar_capability_and_timestamp(self):
        c=self.client()
        with self.assertRaisesRegex(ValueError,'unsupported'):c.sonar_tick(100)
        c.sonar_auxiliary_transport=True
        for bad in (0,-1,True,2**63,1.5):
            with self.assertRaisesRegex(ValueError,'invalid_sonar_auxiliary_stamp'):c.sonar_tick(bad)
        self.assertEqual(c.pending_bytes,0)
        c.sonar_tick(110000000)
        self.assertEqual(c.input.get_nowait(),struct.pack('<IIq',8,8,110000000))

    def test_dead_worker_cleanup_closes_both_pipes_even_with_buffered_input(self):
        class BrokenInput(io.BytesIO):
            def close(self):
                super().close()
                raise BrokenPipeError('dead worker')
        c=self.client();c.writer=threading.Thread();c.reader=threading.Thread()
        c.writer.start();c.reader.start()
        c.process=type('Process',(),dict(stdin=BrokenInput(),stdout=io.BytesIO(),
            poll=lambda self:2,returncode=2))()
        self.assertEqual(c.close(),2)
        self.assertTrue(c.process.stdin.closed);self.assertTrue(c.process.stdout.closed)
        self.assertFalse(c.writer.is_alive() or c.reader.is_alive())
        self.assertEqual(c.close(),2)

    def test_pre_ready_input_is_rejected_without_consuming_budget_or_faulting(self):
        c=self.client();c.ready.clear()
        with self.assertRaises(WorkerNotReady):c.imu(1,[0,0,0],[0,0,9.8])
        self.assertEqual(c.pending_bytes,0);self.assertTrue(c.input.empty())
        self.assertIsNone(c.fault)
        c.ready.set();c.imu(2,[0,0,0],[0,0,9.8])
        self.assertEqual(c.pending_bytes,64)

    def test_reader_rejects_estimates_before_ready_and_duplicate_handshakes(self):
        ready=dict(type='ready',protocol=1,inertial_fused=True)
        for values,reason in (([dict(type='state')],'worker_output_before_ready'),
                              ([ready,ready],'worker_protocol')):
            c=self.client();c.ready.clear()
            c.process=type('Process',(),{'stdout':io.BytesIO(b''.join(
                (json.dumps(v)+'\n').encode() for v in values))})()
            c._read()
            self.assertEqual(c.fault,reason)
            with self.assertRaises(RuntimeError):c.poll()

    def test_rgb_bgr_padded_rows_produce_identical_pixel_values(self):
        rgb=np.zeros((32,100),dtype=np.uint8);rgb[:,:96]=np.tile([255,0,0],32)
        bgr=np.zeros_like(rgb);bgr[:,:96]=np.tile([0,0,255],32)
        a=gray8(32,32,100,'rgb8',rgb.tobytes());b=gray8(32,32,100,'bgr8',bgr.tobytes())
        self.assertTrue(np.array_equal(a,b));self.assertTrue(np.all(a==77))
        with self.assertRaises(ValueError):gray8(32,32,95,'rgb8',rgb.tobytes())

    def test_ready_cannot_silently_ignore_requested_processing_budgets(self):
        ready=dict(type='ready',protocol=1,inertial_fused=True,raw_resolution=[800,600],processed_resolution=[400,300],feature_grid=25)
        for change,reason in (({'feature_grid':50},'worker_feature_budget'),
                              ({'processed_resolution':[800,600]},'worker_pixel_budget'),
                              ({'processed_resolution':None},'worker_processing_resolution'),
                              ({'raw_resolution':[640,480]},'worker_raw_resolution')):
            c=self.client();c.ready.clear()
            c.processing_request=dict(width=800,height=600,max_image_pixels=120000,max_feature_cells=200)
            c.process=type('Process',(),{'stdout':io.BytesIO((json.dumps(ready|change)+'\n').encode())})()
            c._read();self.assertEqual(c.fault,reason);self.assertFalse(c.ready.is_set())
        c=self.client();c.processing_request=dict(width=800,height=600,max_image_pixels=120000,max_feature_cells=200)
        c._check_processing_ready(ready)

    def test_gravity_seed_requires_capability_and_unit_direction(self):
        c=self.client()
        with self.assertRaisesRegex(ValueError,'unsupported'):c.gravity_seed(12,[0,0,1])
        c.gravity_seed_transport=True
        for direction in ([0,0,0],[0,0,2],[False,0,1],[float('nan'),0,1]):
            with self.assertRaisesRegex(ValueError,'invalid_gravity_seed'):c.gravity_seed(12,direction)
        c.gravity_seed(12,[0,0,-1])
        packet=c.input.get_nowait()
        self.assertEqual(struct.unpack('<IIq3d',packet),(6,32,12,0.,0.,-1.))

    def test_reader_rejects_missing_required_gravity_seed_contract(self):
        c=self.client();c.ready.clear();c.processing_request={'require_gravity_seed':True}
        ready=dict(type='ready',protocol=1,inertial_fused=True)
        c.process=type('Process',(),{'stdout':io.BytesIO((json.dumps(ready)+'\n').encode())})()
        c._read();self.assertEqual(c.fault,'worker_gravity_seed_unsupported')
        self.assertFalse(c.ready.is_set())

    def test_inertial_seed_capability_frames_and_validation(self):
        c=self.client()
        with self.assertRaisesRegex(ValueError,'unsupported'):
            c.inertial_seed(12,[0,0,-1],[1,2,3],[.01,.02,.03],[.1,.2,.3])
        c.inertial_seed_transport=True
        c.inertial_seed(12,[0,0,-1],[1,2,3],[.01,.02,.03],[.1,.2,.3])
        packet=c.input.get_nowait()
        self.assertEqual(struct.unpack('<IIq12d',packet),
                         (13,104,12,0.,0.,-1.,1.,2.,3.,.01,.02,.03,.1,.2,.3))
        for velocity in ([0,0],[0,0,float('nan')],[False,0,0]):
            with self.assertRaisesRegex(ValueError,'invalid_inertial_seed'):
                c.inertial_seed(12,[0,0,1],velocity,[0,0,0],[0,0,0])
        with self.assertRaisesRegex(ValueError,'invalid_inertial_seed'):
            c.inertial_seed(12,[0,0,2],[0,0,0],[0,0,0],[0,0,0])
        self.assertTrue(c.input.empty())

    def test_startup_image_requires_capability_and_valid_shape(self):
        c=self.client();pixels=bytes(32*32)
        with self.assertRaisesRegex(ValueError,'unsupported'):c.startup_image(12,32,32,pixels)
        c.startup_tracks_transport=True
        c.startup_image(12,32,32,pixels)
        packet=c.input.get_nowait()
        self.assertEqual(struct.unpack('<IIqII',packet[:24]),(14,1040,12,32,32))
        self.assertEqual(packet[24:],pixels)
        with self.assertRaisesRegex(ValueError,'invalid_image_packet'):c.startup_image(12,32,32,pixels[:-1])

    def test_sonar_clock_never_silently_falls_back_to_camera_worker(self):
        c=self.client();c.ready.clear();c.processing_request={'state_clock':'sonar','require_gravity_seed':True}
        ready=dict(type='ready',protocol=1,inertial_fused=True,
                   gravity_seed_transport='first_image_direction_v1',gravity_seed_required=True)
        c.process=type('Process',(),{'stdout':io.BytesIO((json.dumps(ready)+'\n').encode())})()
        c._read();self.assertEqual(c.fault,'worker_sonar_clock_unsupported')
        self.assertFalse(c.ready.is_set())

    def test_sonar_clock_tick_and_camera_rejection(self):
        c=self.client();c.processing_request={'state_clock':'sonar'};c.sonar_clock_transport=True
        c.sonar_tick(100)
        self.assertEqual(c.input.get_nowait(),struct.pack('<IIq',8,8,100))
        before=c.pending_bytes
        with self.assertRaisesRegex(ValueError,'camera_packet_in_sonar_clock'):
            c.image(101,32,32,bytes(1024))
        self.assertEqual(c.pending_bytes,before);self.assertTrue(c.input.empty())

    def test_imu_vectors_must_have_separate_three_axis_shapes(self):
        c=self.client()
        with self.assertRaises(ValueError):c.imu(1,[1,2],[3,4,5,6])
        with self.assertRaises(ValueError):c.imu(1,[True,0,0],[0,0,9.8])
        c.imu(1,np.zeros(3),np.array([0,0,9.8]))
        self.assertEqual(len(c.input.get_nowait()),64)

    def test_byte_budget_bounds_queued_images_before_the_packet_limit(self):
        c=self.client();pixels=bytes(1920*1080)
        c.image(1,1920,1080,pixels);c.image(2,1920,1080,pixels)
        with self.assertRaisesRegex(RuntimeError,'byte_budget'):c.image(3,1920,1080,pixels)
        self.assertLessEqual(c.pending_bytes,4*1024*1024)

    def test_ping_is_atomic_and_exact_time_without_silent_fallback(self):
        c=self.client();pixels=bytes(32*32)
        with self.assertRaisesRegex(ValueError,'unsupported'):
            c.image(100,32,32,pixels,ping=(100,2.5))
        c.ping_transport=True
        for ping in ((99,2.5),(True,2.5),(100,float('nan')),(100,0),(100,True),(100,1001)):
            with self.assertRaises(ValueError):c.image(100,32,32,pixels,ping=ping)
        self.assertTrue(c.input.empty());self.assertEqual(c.pending_bytes,0)
        c.image(100,32,32,pixels,ping=(100,2.5))
        packet=c.input.get_nowait()
        self.assertEqual(struct.unpack('<IIqIIqd',packet[:40]),(5,1056,100,32,32,100,2.5))
        self.assertEqual(packet[40:],pixels)
        c.image(101,32,32,pixels)
        self.assertEqual(struct.unpack('<I',c.input.get_nowait()[:4]),(2,))

    def test_worker_fault_prevents_old_buffered_output_from_escaping(self):
        c=self.client();c.output.put({'type':'state','stamp_ns':1});c.fault='worker_closed'
        with self.assertRaisesRegex(RuntimeError,'worker_closed'):c.poll()
        with self.assertRaisesRegex(RuntimeError,'worker_closed'):c.imu(2,[0,0,0],[0,0,9.8])


if __name__=='__main__':unittest.main()
