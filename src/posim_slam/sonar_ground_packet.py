"""Original-time raw entry packet; fitted plane is initialization only."""
import math
import struct
import numpy as np


def encode_ground(stamp_ns, *, beam_ids, azimuth_rad, range_m, boundary_sign,
                  normal_sonar, distance_sonar_m):
    n=np.asarray(normal_sonar,dtype=float);az=np.asarray(azimuth_rad,dtype=float);ranges=np.asarray(range_m,dtype=float)
    if (type(stamp_ns) is not int or not 0<stamp_ns<2**63 or type(boundary_sign) is not int
            or boundary_sign not in (-1,1) or not 8<=len(beam_ids)<=128
            or any(type(k) is not int or not 0<=k<2**31 for k in beam_ids) or len(set(beam_ids))!=len(beam_ids)
            or az.shape!=(len(beam_ids),) or ranges.shape!=az.shape or not np.isfinite(az).all()
            or not np.isfinite(ranges).all() or np.any(ranges<=0) or np.any(np.diff(az)<=0)
            or np.any(abs(az)>=math.pi/2) or n.shape!=(3,) or not np.isfinite(n).all()
            or not np.isclose(n@n,1.,atol=1e-8,rtol=0) or not math.isfinite(distance_sonar_m)):
        raise ValueError('invalid_ground_packet')
    packet=struct.pack('<qIi4d',stamp_ns,len(beam_ids),boundary_sign,*n,distance_sonar_m)
    return packet+b''.join(struct.pack('<I2d',k,a,r) for k,a,r in zip(beam_ids,az,ranges))
