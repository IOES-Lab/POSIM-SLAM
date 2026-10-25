"""Original-time scalar Ping measurement; model uncertainty belongs to calibration."""
import math
import numbers
import struct


def encode_ping(stamp_ns, range_m):
    if (type(stamp_ns) is not int or not 0 < stamp_ns < 2**63 or
            not isinstance(range_m,numbers.Real) or isinstance(range_m,bool) or
            not math.isfinite(range_m) or not 0 < range_m <= 1000):
        raise ValueError('invalid_ping_measurement')
    return struct.pack('<qd',stamp_ns,range_m)
