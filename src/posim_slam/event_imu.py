"""Release original raw IMU only through the next acknowledged state event.

The caller owns a bounded, validated sensor history and event acknowledgements.
Future samples stay in that history while the worker processes a state. Each
event needs the first sample strictly after its timestamp for IMU integration.
"""


class EventImu:
    def __init__(self):
        self.stamp_ns = 0
        self.event_ns = 0
        self.sent_samples = 0

    def select(self, samples, event_ns):
        """Return the unsent prefix needed by one increasing state timestamp.

        Sample rows begin with (timestamp_ns, gyro, specific_force). Additional
        metadata is retained. Bootstrap uses the existing 200 ms history. A
        missing future sample or overwritten history fails before transmission;
        this helper never interpolates, repeats or drops an unsent measurement.
        """
        if type(event_ns) is not int or not self.event_ns < event_ns < 2**63:
            raise ValueError('event_imu_state_clock')
        if self.stamp_ns > event_ns:
            self.event_ns = event_ns
            return []
        selected = []
        previous = self.stamp_ns
        start = event_ns-200_000_000 if not previous else previous+1
        for row in samples:
            stamp = row[0]
            if stamp < start:
                continue
            if (type(stamp) is not int or not 0 < stamp < 2**63
                    or (previous and not 0 < stamp-previous <= 20_000_001)):
                raise ValueError('event_imu_history_gap')
            selected.append(row)
            previous = stamp
            if stamp > event_ns:
                break
        if not selected or selected[-1][0] <= event_ns:
            raise ValueError('event_imu_future_sample_missing')
        if not self.stamp_ns and selected[0][0] > event_ns:
            raise ValueError('event_imu_bootstrap_history_missing')
        self.stamp_ns = previous
        self.event_ns = event_ns
        self.sent_samples += len(selected)
        return selected

    def status(self):
        return dict(stamp_ns=self.stamp_ns, event_ns=self.event_ns,
                    sent_samples=self.sent_samples)
