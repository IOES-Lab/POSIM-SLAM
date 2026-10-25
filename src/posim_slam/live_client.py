"""Bounded, asynchronous pixel/raw-IMU transport to an isolated native worker.

No ROS/application imports. A caller owns each segment and must discard all
old estimates on a fault; restarting a worker never stitches trajectories.
"""
import json
import math
import numbers
from pathlib import Path
import queue
import struct
import subprocess
import threading


class WorkerNotReady(RuntimeError):
    """The process exists but has not accepted the transport contract yet."""


class LiveClient:
    def __init__(self, binary, calibration, log):
        self.processing_request=json.loads(Path(calibration).read_text())
        if not isinstance(self.processing_request,dict):raise ValueError('invalid_worker_calibration')
        for name,low,high in (('max_image_pixels',1024,1920*1080),('max_feature_cells',32,512)):
            if name in self.processing_request:
                value=self.processing_request[name]
                if type(value) is not int or not low<=value<=high:raise ValueError('invalid_'+name)
        self.input=queue.Queue(maxsize=256)
        self.output=queue.Queue(maxsize=8)
        self.fault=None
        self.budget_lock=threading.Lock();self.pending_bytes=0
        self.closed=threading.Event()
        self.ready=threading.Event()
        self.ping_transport=False;self.gravity_seed_transport=False;self.sonar_transport=False;self.sonar_auxiliary_transport=False
        self.sonar_clock_transport=False
        self.process=subprocess.Popen([str(Path(binary).resolve()),str(Path(calibration).resolve())],
            stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=log,bufsize=65536)
        self.writer=threading.Thread(target=self._write,daemon=True)
        self.reader=threading.Thread(target=self._read,daemon=True)
        self.writer.start();self.reader.start()

    def _write(self):
        try:
            while not self.closed.is_set():
                try:data=self.input.get(timeout=.1)
                except queue.Empty:continue
                view=memoryview(data)
                while view:
                    count=self.process.stdin.write(view)
                    if not count:raise EOFError('worker_input_closed')
                    view=view[count:]
                self.process.stdin.flush()
                with self.budget_lock:self.pending_bytes-=len(data)
        except (OSError,ValueError,EOFError) as error:
            if not self.closed.is_set():self.fault=str(error)

    def _read(self):
        try:
            while not self.closed.is_set():
                line=self.process.stdout.readline(256*1024)
                if not line:raise EOFError('worker_closed')
                if len(line)>=256*1024 or not line.endswith(b'\n'):
                    raise ValueError('worker_output_size')
                value=json.loads(line)
                if not isinstance(value,dict):raise ValueError('worker_output_object')
                if value.get('type')=='error':raise ValueError('worker:'+str(value.get('reason')))
                if value.get('type') not in ('ready','state','map','export'):raise ValueError('worker_output_type')
                if value['type']=='ready':
                    if self.ready.is_set() or value.get('protocol')!=1 or value.get('inertial_fused') is not True:
                        raise ValueError('worker_protocol')
                    self._check_processing_ready(value)
                    sonar_clock=self.processing_request.get('state_clock','camera')=='sonar'
                    self.sonar_clock_transport=value.get('sonar_clock_transport')=='ordered_sonar_state_v1'
                    if sonar_clock and (not self.sonar_clock_transport or value.get('state_clock')!='sonar'):
                        raise ValueError('worker_sonar_clock_unsupported')
                    self.gravity_seed_transport=value.get('gravity_seed_transport')==(
                        'first_sensor_direction_v1' if sonar_clock else 'first_image_direction_v1')
                    if self.processing_request.get('require_gravity_seed',False) and (
                        not self.gravity_seed_transport or value.get('gravity_seed_required') is not True):
                        raise ValueError('worker_gravity_seed_unsupported')
                    self.ping_transport=value.get("image_ping_transport")=="exact_stamp_range_v1"
                    self.sonar_auxiliary_transport=value.get("sonar_auxiliary_transport")=="ordered_imu_state_v1"
                    self.sonar_transport=value.get("sonar_snapshot_transport")=="exact_image_track_snapshot_v1"
                elif not self.ready.is_set():
                    raise ValueError('worker_output_before_ready')
                # Never silently drop poses/maps while calling the segment live.
                self.output.put_nowait(value)
                if value['type']=='ready':self.ready.set()
        except (OSError,ValueError,EOFError,queue.Full) as error:
            if not self.closed.is_set():self.fault=str(error) or 'worker_output_backlog'

    def _check_processing_ready(self,value):
        request=self.processing_request
        pixels=request.get('max_image_pixels');cells=request.get('max_feature_cells')
        if pixels is None and cells is None:return
        size=value.get('processed_resolution')
        if not isinstance(size,list) or len(size)!=2 or any(type(n) is not int or n<32 for n in size):
            raise ValueError('worker_processing_resolution')
        if value.get('raw_resolution')!=[request.get('width'),request.get('height')]:
            raise ValueError('worker_raw_resolution')
        if pixels is not None and size[0]*size[1]>pixels:raise ValueError('worker_pixel_budget')
        if cells is not None:
            expected=max(16,math.ceil(math.sqrt(size[0]*size[1]/cells)))
            if value.get('feature_grid')!=expected:raise ValueError('worker_feature_budget')

    def send(self,kind,payload):
        if self.closed.is_set() or self.fault:raise RuntimeError(self.fault or 'worker_closed')
        if not self.ready.is_set():raise WorkerNotReady('worker_not_ready')
        packet=struct.pack('<II',kind,len(payload))+payload
        with self.budget_lock:
            if self.pending_bytes+len(packet)>4*1024*1024:
                self.fault='worker_input_byte_budget';raise RuntimeError(self.fault)
            self.pending_bytes+=len(packet)
        try:self.input.put_nowait(packet)
        except queue.Full:
            with self.budget_lock:self.pending_bytes-=len(packet)
            self.fault='worker_input_backlog'
            raise RuntimeError(self.fault)

    def imu(self,stamp_ns,gyro,accel):
        gyro,accel=list(gyro),list(accel);values=gyro+accel
        if (type(stamp_ns) is not int or not 0<stamp_ns<2**63 or len(gyro)!=3 or len(accel)!=3
                or not all(isinstance(v,numbers.Real) and not isinstance(v,bool) and math.isfinite(v) for v in values)):
            raise ValueError('invalid_imu_packet')
        self.send(1,struct.pack('<q6d',stamp_ns,*values))

    def image(self,stamp_ns,width,height,pixels,*,ping=None):
        if self.processing_request.get('state_clock','camera')=='sonar':
            raise ValueError('camera_packet_in_sonar_clock')
        if (type(stamp_ns) is not int or not 0<stamp_ns<2**63 or
            type(width) is not int or type(height) is not int or
            not 32<=width<=1920 or not 32<=height<=1080 or len(pixels)!=width*height):
            raise ValueError('invalid_image_packet')
        payload=struct.pack('<qII',stamp_ns,width,height)
        if ping is not None:
            if (not isinstance(ping,(tuple,list)) or len(ping)!=2 or
                type(ping[0]) is not int or ping[0]!=stamp_ns or
                not isinstance(ping[1],numbers.Real) or isinstance(ping[1],bool) or
                not math.isfinite(ping[1]) or not 0<ping[1]<=1000):
                raise ValueError('invalid_exact_image_ping')
            if not getattr(self,'ping_transport',False):
                raise ValueError('worker_image_ping_unsupported')
            payload+=struct.pack('<qd',*ping)
        self.send(5 if ping is not None else 2,payload+pixels)

    def sonar_snapshot(self,stamp_ns,tracks):
        """Send before its exact sensor event; active IDs arrive in state replies."""
        if not getattr(self,'sonar_transport',False):
            raise ValueError('worker_sonar_snapshot_unsupported')
        from .sonar_packet import encode_snapshot
        self.send(7,encode_snapshot(stamp_ns,tracks))

    def sonar_tick(self,stamp_ns):
        """Advance at the original sonar time, after its optional snapshot.

        Camera clock allows one auxiliary state between consecutive images.
        Explicit sonar clock permits camera-free startup and consecutive ticks.
        """
        if not (getattr(self,'sonar_auxiliary_transport',False) or getattr(self,'sonar_clock_transport',False)):
            raise ValueError('worker_sonar_auxiliary_unsupported')
        if type(stamp_ns) is not int or not 0<stamp_ns<2**63:
            raise ValueError('invalid_sonar_auxiliary_stamp')
        self.send(8,struct.pack('<q',stamp_ns))

    def gravity_seed(self,stamp_ns,direction):
        values=list(direction)
        if (type(stamp_ns) is not int or not 0<stamp_ns<2**63 or len(values)!=3
            or not all(isinstance(v,numbers.Real) and not isinstance(v,bool) and math.isfinite(v) for v in values)
            or abs(sum(v*v for v in values)-1)>1e-6):raise ValueError('invalid_gravity_seed')
        if not getattr(self,'gravity_seed_transport',False):raise ValueError('worker_gravity_seed_unsupported')
        self.send(6,struct.pack('<q3d',stamp_ns,*values))

    def snapshot(self,request_id):
        if type(request_id) is not int or not 0<request_id<2**63:
            raise ValueError('invalid_snapshot_request_id')
        self.send(4,struct.pack('<Q',request_id))

    def poll(self):
        if self.fault:raise RuntimeError(self.fault)
        result=[]
        while True:
            try:result.append(self.output.get_nowait())
            except queue.Empty:return result

    def close(self):
        self.closed.set()
        if self.process.poll() is None:
            self.process.terminate()
            try:self.process.wait(timeout=2)
            except subprocess.TimeoutExpired:self.process.kill();self.process.wait(timeout=2)
        self.writer.join(timeout=2);self.reader.join(timeout=2)
        try:
            # Buffered stdin may still contain data when a crashed worker exits.
            # Closing flushes that buffer; EPIPE is expected during teardown.
            try:self.process.stdin.close()
            except BrokenPipeError:pass
        finally:self.process.stdout.close()
        while not self.input.empty():self.input.get_nowait()
        if self.writer.is_alive() or self.reader.is_alive():raise RuntimeError('worker_thread_cleanup_failed')
        return self.process.returncode
