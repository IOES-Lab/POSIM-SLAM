import unittest
from posim_slam.image_imu_queue import ImageImuQueue


class ImageImuTests(unittest.TestCase):
    def test_simulation_clock_zero_is_a_real_timestamp(self):
        queue=ImageImuQueue();queue.push(0,'first',1.,1)
        self.assertIsNone(queue.next_ready(1.01));queue.observe_imu(0)
        self.assertEqual(queue.next_ready(1.02).stamp_ns,0)

    def test_camera_before_imu_has_identical_exposure_payload_and_clock(self):
        payload=object();outputs=[]
        for camera_first in (True,False):
            queue=ImageImuQueue()
            if not camera_first:queue.observe_imu(100)
            queue.push(100,payload,1.,50)
            if camera_first:
                self.assertIsNone(queue.next_ready(1.01));queue.observe_imu(99)
                self.assertIsNone(queue.next_ready(1.02));queue.observe_imu(100)
            value=queue.next_ready(1.03);outputs.append(value)
            self.assertIs(value.payload,payload);self.assertEqual(value.stamp_ns,100)
            self.assertEqual(value.receipt_wall,1.);self.assertIsNone(queue.next_ready(1.04))
        self.assertEqual(outputs[0],outputs[1])

    def test_future_imu_releases_ordered_frames_without_inventing_measurements(self):
        queue=ImageImuQueue()
        queue.push(100,'a',1.,50);queue.push(200,'b',1.1,60);queue.push(300,'c',1.2,70)
        queue.observe_imu(250)
        self.assertEqual([queue.next_ready(1.3).payload for _ in range(2)],['a','b'])
        self.assertIsNone(queue.next_ready(1.3));queue.observe_imu(301)
        self.assertEqual(queue.next_ready(1.3).stamp_ns,300)
        self.assertEqual(queue.status()['payload_bytes'],0)

    def test_stale_or_capacity_failure_is_explicit_and_reset_releases_old_context(self):
        queue=ImageImuQueue(max_frames=1,max_payload_bytes=50)
        queue.push(100,'old',1.,50)
        with self.assertRaisesRegex(ValueError,'capacity'):queue.push(200,'extra',1.1,1)
        with self.assertRaisesRegex(ValueError,'imu_stale'):queue.next_ready(3.001)
        generation=queue.generation;queue.reset()
        self.assertGreater(queue.generation,generation);self.assertEqual(queue.status()['discarded_on_reset'],1)
        queue.observe_imu(1000);queue.push(1000,'new',4.,1)
        self.assertEqual(queue.next_ready(4.1).payload,'new')


if __name__=='__main__':unittest.main()
