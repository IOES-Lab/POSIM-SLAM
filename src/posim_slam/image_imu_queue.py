"""Bounded original-time camera delivery after raw IMU covers each exposure.

Receipt order does not imply measurement order. No sample is retimed, predicted
or duplicated. The caller retains IMU values and owns integration/startup.
"""
from collections import deque
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class Exposure:
    stamp_ns: int
    payload: object
    receipt_wall: float
    payload_bytes: int


class ImageImuQueue:
    def __init__(self, *, max_frames=32,max_payload_bytes=64*1024**2,max_age_s=2.):
        if (type(max_frames) is not int or not 1<=max_frames<=32
                or type(max_payload_bytes) is not int or not 1<=max_payload_bytes<=64*1024**2
                or not math.isfinite(max_age_s) or not 0<max_age_s<=2):
            raise ValueError('image_imu_queue_budget')
        self.max_frames=max_frames;self.max_payload_bytes=max_payload_bytes;self.max_age_s=max_age_s
        self.pending=deque();self.bytes=0;self.last_image=None;self.last_imu=None
        self.received=0;self.released=0;self.discarded=0;self.peak=0;self.peak_bytes=0;self.generation=0

    def observe_imu(self,stamp_ns):
        if (type(stamp_ns) is not int or not 0<=stamp_ns<2**63
                or (self.last_imu is not None and stamp_ns<=self.last_imu)):
            raise ValueError('image_imu_queue_imu_clock')
        self.last_imu=stamp_ns

    def push(self,stamp_ns,payload,receipt_wall,payload_bytes):
        if (type(stamp_ns) is not int or not 0<=stamp_ns<2**63
                or (self.last_image is not None and stamp_ns<=self.last_image)
                or not math.isfinite(receipt_wall) or receipt_wall<0
                or type(payload_bytes) is not int or payload_bytes<0):
            raise ValueError('image_imu_queue_image_contract')
        if len(self.pending)>=self.max_frames or self.bytes+payload_bytes>self.max_payload_bytes:
            raise ValueError('image_imu_queue_capacity')
        self.pending.append(Exposure(stamp_ns,payload,receipt_wall,payload_bytes));self.bytes+=payload_bytes
        self.last_image=stamp_ns;self.received+=1
        self.peak=max(self.peak,len(self.pending));self.peak_bytes=max(self.peak_bytes,self.bytes)

    def next_ready(self,now):
        if not self.pending:return None
        value=self.pending[0]
        if not math.isfinite(now) or not 0<=now-value.receipt_wall<=self.max_age_s:
            raise ValueError('imu_stale')
        if self.last_imu is None or self.last_imu<value.stamp_ns:return None
        self.pending.popleft();self.bytes-=value.payload_bytes;self.released+=1
        return value

    def reset(self):
        self.discarded+=len(self.pending);self.pending.clear();self.bytes=0
        self.last_image=None;self.last_imu=None;self.generation+=1

    def status(self):
        return dict(pending=len(self.pending),payload_bytes=self.bytes,received=self.received,
            released=self.released,discarded_on_reset=self.discarded,peak_pending=self.peak,
            peak_payload_bytes=self.peak_bytes,imu_watermark_ns=self.last_imu)
