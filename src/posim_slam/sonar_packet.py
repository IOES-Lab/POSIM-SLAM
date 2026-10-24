"""Bounded acoustic track snapshots; transport does not establish association.

Each track supplies its complete history and covariance in range/azimuth order.
The latest observation must match the image timestamp receiving this snapshot.
"""
import math
import numbers
import struct
import numpy as np

MAX_PAYLOAD = 16 + 16 * (16 + 64 * 24 + 128 * 128 * 8)


def encode_snapshot(stamp_ns, tracks):
    if type(stamp_ns) is not int or not 0 < stamp_ns < 2**63:
        raise ValueError('invalid_sonar_snapshot_stamp')
    if not isinstance(tracks, (list, tuple)) or not 1 <= len(tracks) <= 16:
        raise ValueError('invalid_sonar_snapshot_tracks')
    parts = [struct.pack('<qII', stamp_ns, len(tracks), 0)]
    previous_id = -1
    for track in tracks:
        ident = track['id']
        observations = track['observations']
        if type(ident) is not int or not previous_id < ident < 2**63:
            raise ValueError('unordered_sonar_snapshot_ids')
        if not isinstance(observations, (list, tuple)) or not 1 <= len(observations) <= 64:
            raise ValueError('invalid_sonar_snapshot_observations')
        parts.append(struct.pack('<qII', ident, len(observations), 0))
        previous_stamp = 0
        for observation in observations:
            stamp, distance, azimuth = (observation[k] for k in ('stamp_ns', 'range_m', 'azimuth_rad'))
            if (type(stamp) is not int or not previous_stamp < stamp <= stamp_ns or
                not all(isinstance(v, numbers.Real) and not isinstance(v, bool) and math.isfinite(v)
                        for v in (distance, azimuth)) or not 0 < distance <= 1000 or abs(azimuth) > math.pi/2):
                raise ValueError('invalid_sonar_snapshot_observation')
            parts.append(struct.pack('<qdd', stamp, distance, azimuth))
            previous_stamp = stamp
        if previous_stamp != stamp_ns:
            raise ValueError('sonar_snapshot_image_stamp')
        size = 2 * len(observations)
        noise = np.asarray(track['covariance'], dtype=float)
        if noise.shape != (size, size) or not np.isfinite(noise).all() or not (np.diag(noise) > 0).all():
            raise ValueError('invalid_sonar_snapshot_covariance')
        scales = np.sqrt(np.diag(noise))
        correlation = noise / scales[:, None] / scales[None, :]
        if not np.isfinite(correlation).all() or np.max(np.abs(correlation-correlation.T)) > 1e-10:
            raise ValueError('asymmetric_sonar_snapshot_covariance')
        try:
            np.linalg.cholesky((correlation+correlation.T)*.5)
        except np.linalg.LinAlgError as error:
            raise ValueError('nonpositive_sonar_snapshot_covariance') from error
        parts.append(noise.astype('<f8').tobytes(order='C'))
        previous_id = ident
    payload = b''.join(parts)
    if len(payload) > MAX_PAYLOAD:
        raise ValueError('sonar_snapshot_size')
    return payload
