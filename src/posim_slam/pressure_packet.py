"""Pressure-derived height with one native unknown datum, never first-sample subtraction.

The caller supplies calibrated pressure-to-height scale, independent variance,
submerged validity and original measurement time. Height is positive upward.
"""
import math
import numbers
import struct


def encode_pressure(stamp_ns, height_m, variance_m2):
    if (type(stamp_ns) is not int or not 0<stamp_ns<2**63
            or any(isinstance(v,bool) or not isinstance(v,numbers.Real) or not math.isfinite(v)
                   for v in (height_m,variance_m2))
            or abs(height_m)>20000 or not 0<variance_m2<=1e6):
        raise ValueError('invalid_pressure_observation')
    return struct.pack('<qdd',stamp_ns,height_m,variance_m2)
