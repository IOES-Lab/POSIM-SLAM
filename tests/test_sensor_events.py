import itertools
import unittest

from posim_slam.sensor_events import SensorEvents


class SensorEventsTests(unittest.TestCase):
    def test_range_only_clock_preserves_delayed_ping_without_camera(self):
        for order in set(itertools.permutations('ssppi')):
            q=SensorEvents(state_clock='sonar',ping=True)
            times={'sonar':iter([10,30]),'ping':iter([15,30]),'imu':iter([31])};events=[]
            for letter in order:
                kind={'s':'sonar','p':'ping','i':'imu'}[letter];stamp=next(times[kind])
                q.push(kind,stamp,(kind,stamp));events.extend(self.drain(q))
            self.assertEqual([e['stamp_ns'] for e in events],[10,15,30])
            self.assertEqual(events[1]['ping'],('ping',15))
            self.assertIsNone(events[1]['sonar'])
            self.assertTrue(all(e['camera'] is None for e in events))

    def test_pressure_and_dvl_coalesce_only_equal_times_under_delayed_arrival(self):
        for order in set(itertools.permutations('ssddppi')):
            q=SensorEvents(state_clock='sonar',dvl=True,pressure=True)
            times={'sonar':iter([10,30]),'dvl':iter([20,30]),'pressure':iter([15,30]),'imu':iter([31])}
            events=[]
            for letter in order:
                kind={'s':'sonar','d':'dvl','p':'pressure','i':'imu'}[letter];stamp=next(times[kind])
                q.push(kind,stamp,(kind,stamp));events.extend(self.drain(q))
            self.assertEqual([e['stamp_ns'] for e in events],[10,15,20,30])
            self.assertEqual(events[1]['pressure'],('pressure',15))
            self.assertIsNone(events[1]['sonar']);self.assertIsNone(events[1]['dvl'])
            self.assertEqual(events[-1]['pressure'],('pressure',30))

    def test_dvl_and_sonar_keep_original_times_under_all_arrival_orders(self):
        for order in set(itertools.permutations('ssddii')):
            q=SensorEvents(state_clock='sonar',dvl=True)
            times={'sonar':iter([10,30]),'dvl':iter([20,30]),'imu':iter([21,31])}
            events=[]
            for letter in order:
                kind={'s':'sonar','d':'dvl','i':'imu'}[letter];stamp=next(times[kind])
                q.push(kind,stamp,(kind,stamp));events.extend(self.drain(q))
            self.assertEqual([e['stamp_ns'] for e in events],[10,20,30])
            self.assertEqual([e['dvl'] for e in events],[None,('dvl',20),('dvl',30)])
            self.assertEqual(events[-1]['sonar'],('sonar',30))
            self.assertTrue(all(e['camera'] is None for e in events))

    def test_dvl_status_loss_preserves_clock_and_camera_auxiliary_states(self):
        q=SensorEvents(dvl=True);q.push('imu',51)
        for stamp in (10,50):q.push('camera',stamp,'image')
        for stamp in (10,50):q.push('sonar',stamp,'echo')
        for stamp in (10,20,30,40,50):q.push('dvl',stamp,{'valid':stamp!=30})
        events=self.drain(q)
        self.assertEqual([e['stamp_ns'] for e in events],[10,20,30,40,50])
        self.assertEqual(events[2]['dvl'],{'valid':False})
        self.assertEqual(q.auxiliary_skipped,0)
        q=SensorEvents(state_clock='sonar',dvl=True)
        q.push('sonar',10,'echo');q.push('imu',11)
        self.assertIsNone(q.next_event())
        q.push('dvl',10,{'valid':False})
        self.assertIsNotNone(q.next_event())

    def test_sonar_clock_needs_no_camera_and_never_decimates_ticks(self):
        for order in set(itertools.permutations('sssiii')):
            q = SensorEvents(state_clock='sonar')
            times = {'sonar': iter([10, 20, 30]), 'imu': iter([11, 21, 31])}
            events = []
            for letter in order:
                kind = 'sonar' if letter == 's' else 'imu'
                stamp = next(times[kind])
                q.push(kind, stamp, ('echo', stamp) if kind == 'sonar' else None)
                events.extend(self.drain(q))
            self.assertEqual([e['stamp_ns'] for e in events], [10, 20, 30])
            self.assertTrue(all(e['camera'] is None for e in events))
            self.assertEqual([e['sonar'] for e in events], [('echo', t) for t in [10, 20, 30]])
            self.assertEqual((q.before_camera, q.auxiliary_skipped), (0, 0))

    def test_sonar_clock_still_requires_future_imu_and_bounds_backlog(self):
        q = SensorEvents(capacity=2, state_clock='sonar')
        q.push('sonar', 10, 'echo'); q.push('imu', 10)
        self.assertIsNone(q.next_event())
        q.push('sonar', 20, 'echo')
        with self.assertRaisesRegex(ValueError, 'backlog'): q.push('sonar', 30, 'echo')
        self.assertFalse(any(q.pending.values()))
        with self.assertRaises(RuntimeError): q.next_event()
        q = SensorEvents(state_clock='sonar')
        with self.assertRaisesRegex(ValueError, 'input_kind'): q.push('camera', 10, 'image')

    def drain(self, queue):
        result = []
        while (event := queue.next_event()) is not None:
            result.append(event)
            # No second native request can overtake the pending state.
            self.assertIsNone(queue.next_event())
            queue.acknowledge(event['stamp_ns'])
        return result

    def test_every_cross_stream_interleaving_preserves_same_clock_and_payload(self):
        # All 140 interleavings preserve per-stream order, including sonar
        # arriving before/after equal-time camera packets and delayed IMU.
        inputs = {'camera': [10, 20, 30], 'sonar': [10, 15, 30], 'imu': [31]}
        expected = [(10, ('camera', 10), ('sonar', 10)),
                    (15, None, ('sonar', 15)), (20, ('camera', 20), None),
                    (30, ('camera', 30), ('sonar', 30))]
        orders = set(itertools.permutations('cccsssi'))
        self.assertEqual(len(orders), 140)
        for order in orders:
            q = SensorEvents(); positions = dict.fromkeys(inputs, 0); events = []
            for letter in order:
                kind = {'c': 'camera', 's': 'sonar', 'i': 'imu'}[letter]
                stamp = inputs[kind][positions[kind]]; positions[kind] += 1
                q.push(kind, stamp, (kind, stamp))
                events.extend(self.drain(q))
            self.assertEqual([(e['stamp_ns'], e['camera'], e['sonar']) for e in events], expected)

    def test_no_release_until_other_sensor_and_future_imu_arrive(self):
        q = SensorEvents(); q.push('camera', 10, 'image')
        q.push('imu', 10); self.assertIsNone(q.next_event())
        q.push('sonar', 10, 'echo'); self.assertIsNone(q.next_event())
        q.push('imu', 11)
        self.assertEqual(self.drain(q)[0]['sonar'], 'echo')

    def test_dropped_sonar_packet_is_not_replaced_by_previous_echo(self):
        q = SensorEvents(); q.push('imu', 41)
        for stamp in (10, 20, 30, 40): q.push('camera', stamp, stamp)
        for stamp in (10, 30, 40): q.push('sonar', stamp, stamp)
        events = self.drain(q)
        self.assertIsNone(events[1]['sonar'])
        self.assertEqual([e['stamp_ns'] for e in events], [10, 20, 30, 40])

    def test_auxiliary_limit_and_prebootstrap_are_explicit_without_retiming(self):
        q = SensorEvents(); q.push('imu', 41)
        for stamp in (10, 30, 40): q.push('camera', stamp, stamp)
        for stamp in (5, 15, 20, 25, 30, 35, 40): q.push('sonar', stamp, stamp)
        events = self.drain(q)
        self.assertEqual([e['stamp_ns'] for e in events], [10, 15, 30, 35, 40])
        self.assertEqual((q.before_camera, q.auxiliary_skipped), (1, 2))

    def test_missing_stream_or_slow_native_worker_bounds_memory_and_fails_segment(self):
        for pending_ack in (False, True):
            q = SensorEvents(capacity=2)
            if pending_ack:
                q.push('camera', 1, 'first'); q.push('sonar', 1, 'first'); q.push('imu', 10)
                self.assertIsNotNone(q.next_event())
            q.push('camera', 2, 'second'); q.push('camera', 3, 'third')
            with self.assertRaisesRegex(ValueError, 'backlog'): q.push('camera', 4, 'fourth')
            self.assertFalse(any(q.pending.values()))
            with self.assertRaisesRegex(RuntimeError, 'segment_failed'): q.next_event()

    def test_late_duplicate_reset_and_wrong_ack_cannot_continue_segment(self):
        for kind in ('camera', 'sonar', 'imu'):
            for bad in (9, 10, True, 2**63):
                q = SensorEvents(); q.push(kind, 10, 'measurement')
                with self.assertRaisesRegex(ValueError, 'input_clock'): q.push(kind, bad, 'bad')
                with self.assertRaises(RuntimeError): q.push(kind, 11, 'new')
        q = SensorEvents(); q.push('camera', 10, 'image'); q.push('sonar', 10, 'echo'); q.push('imu', 11)
        q.next_event()
        with self.assertRaisesRegex(ValueError, 'unexpected_ack'): q.acknowledge(11)

    def test_finite_input_end_does_not_invent_sensor_or_imu_measurements(self):
        q = SensorEvents(); q.push('camera', 10, 'image'); q.push('imu', 10)
        q.finish('sonar'); q.finish('camera'); self.assertIsNone(q.next_event())
        q.push('imu', 11)
        self.assertIsNone(self.drain(q)[0]['sonar'])
        with self.assertRaisesRegex(ValueError, 'closed'): q.push('camera', 12, 'late')


if __name__ == '__main__': unittest.main()
