"""Bounded, asynchronous pixel/raw-IMU transport to an isolated native worker.

No ROS/application imports. A caller owns each segment and must discard all
old estimates on a fault; restarting a worker never stitches trajectories.
"""
import json
import math
import numbers
import os
from pathlib import Path
import queue
import struct
import subprocess
import shutil
import tempfile
import threading


class WorkerNotReady(RuntimeError):
    """The process exists but has not accepted the transport contract yet."""


class LiveClient:
    def __init__(self, binary, calibration, log):
        self.processing_request=json.loads(Path(calibration).read_text())
        if not isinstance(self.processing_request,dict):raise ValueError('invalid_worker_calibration')
        if ('optical_flow_type' in self.processing_request
                and self.processing_request['optical_flow_type'] not in ('frame_to_frame','patch')):
            raise ValueError('invalid_optical_flow_type')
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
        self.startup_tracks_transport=False;self.inertial_seed_transport=False
        self.ping_transport=False;self.gravity_seed_transport=False;self.sonar_transport=False;self.sonar_auxiliary_transport=False
        self.sonar_clock_transport=False;self.dvl_transport=False;self.sensor_state_transport=False;self.pressure_transport=False;self.terrain_ping_transport=False
        self.ground_transport=False
        self.surface_transport=False;self.surface_directory=None;self._surface_files={};self._surface_request_id=0
        self._surface_lock=threading.Lock();options={}
        if self.processing_request.get('enable_surface_owner',False):
            self.surface_directory=Path(tempfile.mkdtemp(prefix='worker-surface-',dir=Path(calibration).resolve().parent))
            info=self.surface_directory.stat();self._surface_directory_identity=(info.st_dev,info.st_ino)
            options['env']={**os.environ,'POSIM_SLAM_SURFACE_DIRECTORY':str(self.surface_directory)}
        try:
            self.process=subprocess.Popen([str(Path(binary).resolve()),str(Path(calibration).resolve())],
                stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=log,bufsize=65536,**options)
        except BaseException:
            if self.surface_directory is not None:shutil.rmtree(self.surface_directory)
            raise
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
                if value.get('type') not in ('ready','state','map','export','startup_tracks','surface_control'):raise ValueError('worker_output_type')
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
                    self.startup_tracks_transport=value.get('startup_tracks_transport')=='preinitialize_camera_tracks_v1'
                    self.inertial_seed_transport=value.get('inertial_seed_transport')=='first_imu_state_v1'
                    self.ping_transport=value.get("image_ping_transport")=="exact_stamp_range_v1"
                    self.sonar_auxiliary_transport=value.get("sonar_auxiliary_transport")=="ordered_imu_state_v1"
                    self.sonar_transport=value.get("sonar_snapshot_transport")=="exact_image_track_snapshot_v1"
                    self.dvl_transport=value.get('dvl_measurement_transport')=='body_velocity_gyro_covariance_v1'
                    self.terrain_ping_transport=value.get('terrain_ping_transport')=='exact_sensor_range_v1'
                    self.ground_transport=value.get('ground_transport')=='exact_beam_entries_v1'
                    self.surface_transport=value.get('native_surface_transport')=='snapshot_increment_owner_queue_v1'
                    if self.processing_request.get('enable_surface_owner',False) and not self.surface_transport:
                        raise ValueError('worker_surface_owner_unsupported')
                    if self.processing_request.get('enable_sonar_ground',False) and not self.ground_transport:
                        raise ValueError('worker_ground_unsupported')
                    if self.processing_request.get('enable_terrain_ping',False) and not self.terrain_ping_transport:
                        raise ValueError('worker_terrain_ping_unsupported')
                    self.pressure_transport=value.get('pressure_measurement_transport')=='height_unknown_datum_v1'
                    self.sensor_state_transport=value.get('sensor_state_transport')=='ordered_camera_auxiliary_states_v1'
                    if self.processing_request.get('enable_joint_dvl',False) and not self.dvl_transport:
                        raise ValueError('worker_dvl_unsupported')
                    if self.processing_request.get('enable_joint_dvl',False) and not self.sensor_state_transport:
                        raise ValueError('worker_dvl_state_clock_unsupported')
                    if self.processing_request.get('enable_joint_pressure',False) and not (self.pressure_transport and self.sensor_state_transport):
                        raise ValueError('worker_pressure_unsupported')
                elif not self.ready.is_set():
                    raise ValueError('worker_output_before_ready')
                if value['type']=='surface_control':
                    if not self.surface_transport or type(value.get('request_id')) is not int or type(value.get('accepted')) is not bool:
                        raise ValueError('worker_surface_control_response')
                    with self._surface_lock:path=self._surface_files.pop(value['request_id'],None)
                    if path is not None:path.unlink(missing_ok=True)
                # Never silently drop poses/maps while calling the segment live.
                self.output.put_nowait(value)
                if value['type']=='ready':self.ready.set()
        except (OSError,ValueError,EOFError,queue.Full) as error:
            if not self.closed.is_set():self.fault=str(error) or 'worker_output_backlog'

    def _check_processing_ready(self,value):
        request=self.processing_request
        if 'initial_imu_bias_calibration' in request:
            from .imu_bias_calibration import InitialImuBiasCalibration
            calibration=InitialImuBiasCalibration.read(request['initial_imu_bias_calibration']);rows=calibration.rows()
            report=value.get('initial_imu_bias_calibration')
            expected=dict(source=calibration.source,scope='first_state',finite_modes=len(rows['jacobian']),
                exact_modes=len(rows['constraint_jacobian']),stochastic_evolution_unchanged=True,generic_initial_bias_prior_replaced=True)
            if (not isinstance(report,dict) or any(type(report.get(k)) is not type(v) or report[k]!=v for k,v in expected.items())):
                raise ValueError('worker_initial_bias_calibration_contract')
        if 'optical_flow_type' in request and value.get('optical_flow_type')!=request['optical_flow_type']:
            raise ValueError('worker_optical_flow_type')
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

    def _image_header(self,stamp_ns,width,height,pixels):
        if self.processing_request.get('state_clock','camera')=='sonar':
            raise ValueError('camera_packet_in_sonar_clock')
        if (type(stamp_ns) is not int or not 0<stamp_ns<2**63 or
            type(width) is not int or type(height) is not int or
            not 32<=width<=1920 or not 32<=height<=1080 or len(pixels)!=width*height):
            raise ValueError('invalid_image_packet')
        return struct.pack('<qII',stamp_ns,width,height)

    def startup_image(self,stamp_ns,width,height,pixels):
        """Track a bounded startup image without emitting a VIO state/map.

        The caller retains raw IMU locally until a measured first-state seed is
        available. At most 32 monotonically timed frames precede initialization.
        """
        if not getattr(self,'startup_tracks_transport',False):raise ValueError('worker_startup_tracks_unsupported')
        self.send(14,self._image_header(stamp_ns,width,height,pixels)+pixels)

    def image(self,stamp_ns,width,height,pixels,*,ping=None):
        payload=self._image_header(stamp_ns,width,height,pixels)
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

    def dvl(self,stamp_ns,velocity,covariance,*,gyro,gyro_stamp_ns,
            gyro_covariance,bottom_locked,locked_beams):
        """Submit before the exact state event; requires negotiated raw DVL support."""
        if not getattr(self,'dvl_transport',False):raise ValueError('worker_dvl_unsupported')
        from .dvl_packet import encode_dvl
        self.send(9,encode_dvl(stamp_ns,velocity,covariance,gyro=gyro,
            gyro_stamp_ns=gyro_stamp_ns,gyro_covariance=gyro_covariance,
            bottom_locked=bottom_locked,locked_beams=locked_beams))

    def pressure(self,stamp_ns,height_m,variance_m2):
        """Submit calibrated upward height before its original-time state event."""
        if not getattr(self,'pressure_transport',False):raise ValueError('worker_pressure_unsupported')
        from .pressure_packet import encode_pressure
        self.send(10,encode_pressure(stamp_ns,height_m,variance_m2))

    def terrain_ping(self,stamp_ns,range_m):
        """Submit a raw range before its exact state; no camera packet required."""
        if not getattr(self,'terrain_ping_transport',False):raise ValueError('worker_terrain_ping_unsupported')
        from .ping_packet import encode_ping
        self.send(11,encode_ping(stamp_ns,range_m))

    def ground(self,stamp_ns,**observation):
        if not getattr(self,'ground_transport',False):raise ValueError('worker_ground_unsupported')
        from .sonar_ground_packet import encode_ground
        self.send(12,encode_ground(stamp_ns,**observation))

    def sonar_tick(self,stamp_ns):
        """Advance at the original sonar time, after its optional snapshot.

        Camera clock allows one auxiliary state between consecutive images;
        negotiated DVL/pressure support permits consecutive auxiliary states too.
        Explicit sonar clock permits camera-free startup and consecutive ticks,
        including original DVL/pressure times when those transports are enabled.
        """
        if not (getattr(self,'sonar_auxiliary_transport',False) or getattr(self,'sonar_clock_transport',False)
                or getattr(self,'sensor_state_transport',False)):
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

    def inertial_seed(self,stamp_ns,up_direction_imu,velocity_imu,gyro_bias,accel_bias):
        """Numerical startup only, at the exact first sensor state timestamp.

        Up direction and velocity use the first IMU frame; biases use sensor
        axes. This packet carries no covariance or readiness certificate.
        """
        vectors=[list(v) for v in (up_direction_imu,velocity_imu,gyro_bias,accel_bias)]
        if (type(stamp_ns) is not int or not 0<stamp_ns<2**63
            or any(len(v)!=3 for v in vectors)
            or not all(isinstance(x,numbers.Real) and not isinstance(x,bool) and math.isfinite(x)
                       for v in vectors for x in v)
            or abs(sum(x*x for x in vectors[0])-1)>1e-6):
            raise ValueError('invalid_inertial_seed')
        if not getattr(self,'inertial_seed_transport',False):raise ValueError('worker_inertial_seed_unsupported')
        self.send(13,struct.pack('<q12d',stamp_ns,*(x for v in vectors for x in v)))

    def snapshot(self,request_id):
        if type(request_id) is not int or not 0<request_id<2**63:
            raise ValueError('invalid_snapshot_request_id')
        self.send(4,struct.pack('<Q',request_id))

    def _surface_request(self,request_id):
        if not self.ready.is_set():raise WorkerNotReady('worker_not_ready')
        if not self.surface_transport or self.surface_directory is None:raise ValueError('worker_surface_owner_unsupported')
        with self._surface_lock:
            if type(request_id) is not int or not self._surface_request_id<request_id<2**63:
                raise ValueError('ordered_surface_request_id_required')
            self._surface_request_id=request_id

    def surface_snapshot(self,request_id):
        self._surface_request(request_id)
        self.send(15,json.dumps(dict(op='snapshot',request_id=request_id)).encode())

    def read_surface_snapshot(self,response):
        from .native_surface import NativeSurfaceSnapshot
        if response.get('type')!='surface_control' or response.get('op')!='snapshot' or response.get('accepted') is not True:
            raise ValueError('accepted_native_surface_snapshot_required')
        return NativeSurfaceSnapshot.read_file(response['file'],self.surface_directory)

    def surface_update(self,request_id,increment):
        from .native_surface import NativeSurfaceIncrement,MAX_SNAPSHOT_BYTES
        if not isinstance(increment,NativeSurfaceIncrement):raise ValueError('prepared_native_surface_increment_required')
        self._surface_request(request_id)
        data=json.dumps(increment.as_packet(),allow_nan=False,separators=(',',':')).encode()
        if len(data)>MAX_SNAPSHOT_BYTES:raise ValueError('native_surface_update_file_capacity')
        path=self.surface_directory/('worker-surface-update-'+str(request_id)+'.json')
        temporary=path.with_suffix('.tmp')
        with self._surface_lock:
            if self._surface_files:raise RuntimeError('native_surface_update_pending')
            self._surface_files[request_id]=None
        created=False;temporary_created=False
        try:
            with temporary.open('xb') as stream:
                temporary_created=True;stream.write(data)
            if path.exists() or path.is_symlink():raise ValueError('native_surface_update_path_exists')
            temporary.rename(path);created=True;temporary_created=False
            with self._surface_lock:self._surface_files[request_id]=path
            self.send(15,json.dumps(dict(op='update',request_id=request_id,path=str(path),bytes=len(data))).encode())
        except BaseException:
            with self._surface_lock:self._surface_files.pop(request_id,None)
            if created:path.unlink(missing_ok=True)
            if temporary_created:temporary.unlink(missing_ok=True)
            raise

    def surface_expire(self,request_id,closed_through_ns,*,protect=()):
        protect=list(protect)
        if (type(closed_through_ns) is not int or not 0<=closed_through_ns<2**63 or len(protect)>73
                or any(type(t) is not int or not 0<t<2**63 for t in protect) or len(set(protect))!=len(protect)):
            raise ValueError('bounded_native_surface_expiry_required')
        self._surface_request(request_id)
        self.send(15,json.dumps(dict(op='expire',request_id=request_id,closed_through_ns=closed_through_ns,protected_states=protect)).encode())

    def surface_release(self,request_id):
        self._surface_request(request_id);self.send(15,json.dumps(dict(op='release',request_id=request_id)).encode())

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
        if self.surface_directory is not None and self.surface_directory.exists():
            info=self.surface_directory.lstat()
            if self.surface_directory.is_symlink() or (info.st_dev,info.st_ino)!=self._surface_directory_identity:
                raise RuntimeError('native_surface_directory_identity_changed')
            shutil.rmtree(self.surface_directory)
        return self.process.returncode
