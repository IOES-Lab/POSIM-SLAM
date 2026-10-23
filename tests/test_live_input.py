import io,json,queue,threading,unittest
import numpy as np
from posim_slam.image_input import gray8
from posim_slam.live_client import LiveClient,WorkerNotReady


class LiveInputTests(unittest.TestCase):
    def client(self):
        c=LiveClient.__new__(LiveClient);c.closed=threading.Event();c.fault=None
        c.input=queue.Queue(maxsize=256);c.output=queue.Queue(maxsize=8)
        c.budget_lock=threading.Lock();c.pending_bytes=0
        c.ready=threading.Event();c.ready.set()
        return c

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

    def test_worker_fault_prevents_old_buffered_output_from_escaping(self):
        c=self.client();c.output.put({'type':'state','stamp_ns':1});c.fault='worker_closed'
        with self.assertRaisesRegex(RuntimeError,'worker_closed'):c.poll()
        with self.assertRaisesRegex(RuntimeError,'worker_closed'):c.imu(2,[0,0,0],[0,0,9.8])


if __name__=='__main__':unittest.main()
