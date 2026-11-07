from collections import deque
import unittest

from posim_slam.event_imu import EventImu


def samples(start, end):
    return [(t, [t, 0, 0], [0, 0, t]) for t in range(start, end, 10_000_000)]


class EventImuTests(unittest.TestCase):
    def test_future_arrivals_do_not_run_ahead_of_acknowledged_events(self):
        history = deque(samples(1_000_000_000, 6_000_000_000), maxlen=4096)
        stream = EventImu()
        sent = stream.select(history, 2_000_000_000)
        self.assertEqual(sent, list(history)[80:102])
        self.assertEqual(stream.stamp_ns, 2_010_000_000)
        for t in range(2_100_000_000, 5_900_000_000, 100_000_000):
            block = stream.select(history, t)
            self.assertEqual(block[-1][0], t+10_000_000)
            sent.extend(block)
        expected = [row for row in history if 1_800_000_000 <= row[0] <= 5_810_000_000]
        self.assertEqual(sent, expected)
        self.assertEqual(stream.sent_samples, len(expected))

    def test_auxiliary_event_can_use_the_already_sent_future_sample(self):
        history = samples(1_000_000_000, 2_100_000_000)
        stream = EventImu(); stream.select(history, 2_000_000_000)
        self.assertEqual(stream.select(history, 2_005_000_000), [])
        self.assertEqual(stream.select(history, 2_010_000_000)[0][0], 2_020_000_000)
        with self.assertRaisesRegex(ValueError, 'state_clock'):
            stream.select(history, 2_010_000_000)

    def test_uncovered_event_does_not_advance_or_send_partial_interval(self):
        stream = EventImu()
        with self.assertRaisesRegex(ValueError, 'future_sample_missing'):
            stream.select(samples(1_000_000_000, 2_010_000_000), 2_000_000_000)
        self.assertEqual(stream.status(), dict(stamp_ns=0, event_ns=0, sent_samples=0))
        with self.assertRaisesRegex(ValueError, 'bootstrap_history_missing'):
            stream.select(samples(2_010_000_000, 2_100_000_000), 2_000_000_000)
        self.assertEqual(stream.sent_samples, 0)

    def test_overwritten_history_or_raw_gap_is_not_silently_skipped(self):
        stream = EventImu(); stream.select(samples(1_000_000_000, 2_020_000_000), 2_000_000_000)
        before = stream.status()
        with self.assertRaisesRegex(ValueError, 'history_gap'):
            stream.select(samples(2_100_000_000, 2_200_000_000), 2_150_000_000)
        self.assertEqual(stream.status(), before)
        with self.assertRaisesRegex(ValueError, 'history_gap'):
            EventImu().select([samples(1_800_000_000, 1_810_000_000)[0],
                               samples(2_010_000_000, 2_020_000_000)[0]], 2_000_000_000)


if __name__ == '__main__':
    unittest.main()
