from pathlib import Path
import hashlib,json,os,runpy,sys,time

base=Path(__file__).resolve().parent
sys.path[:0]=[str(base/'library/src'),str(base/'replay-driver/scripts')]
import posim_slam.live_client as transport
BaseClient=transport.LiveClient
enabled=os.environ.get('SURFACE_OWNER_REPLAY')=='1'
observations=[]

class ControlledClient(BaseClient):
    def __init__(self,binary,calibration,log):
        if enabled:
            path=Path(calibration);value=json.loads(path.read_text())
            value.update(enable_surface_owner=True,surface_owner_epoch='original-15m-owner-control')
            path.write_text(json.dumps(value))
        super().__init__(binary,calibration,log)
        self.control_request=0;self.state_count=0

    def control(self,method,*args):
        self.control_request+=1;request=self.control_request
        start=time.perf_counter();method(request,*args)
        deadline=time.monotonic()+15
        while time.monotonic()<deadline:
            values=super().poll()
            if values:
                if len(values)!=1 or values[0].get('type')!='surface_control' or values[0]['request_id']!=request:
                    raise ValueError('Unexpected state or clock advance during owner control')
                reply=values[0]
                if not reply['accepted']:raise ValueError('Owner rejected control: '+str(reply))
                row={k:reply[k] for k in ('request_id','op','prior_revision','conditional_nodes','ledger_entries','owner_revision')}
                row['round_trip_ms']=(time.perf_counter()-start)*1000
                if reply['op']=='snapshot':
                    snapshot=self.read_surface_snapshot(reply)
                    row.update(snapshot_bytes=reply['file']['bytes'],stamp_ns=snapshot.stamp_ns,
                        root_columns=sum(v.size for v in snapshot.order))
                    poses=[t for (kind,t) in snapshot._archive.references
                        if kind=='state' and t>snapshot._archive.closed_through_ns]
                    recovered=snapshot.recover([min(poses)])
                    row['recovered_stamp_ns']=min(poses)
                    row['finite_recovered_pose']=bool(__import__('numpy').isfinite(recovered['deltas']).all())
                    if not row['finite_recovered_pose']:raise ValueError('Nonfinite conditional pose')
                observations.append(row);return reply
            time.sleep(.002)
        raise TimeoutError('Original-input owner control timeout')

    def poll(self):
        values=super().poll()
        if enabled:
            for value in values:
                if value.get('type')!='state':continue
                self.state_count+=1
                cutoff=max(0,value['stamp_ns']-30_000_000_000)
                self.control(self.surface_expire,cutoff)
                if self.state_count%5==0:
                    self.control(self.surface_snapshot)
                    self.control(self.surface_release)
        return values

transport.LiveClient=ControlledClient
output=Path(sys.argv[sys.argv.index('--output')+1])
try:
    runpy.run_path(str(base/'replay-driver/scripts/replay_recorded_auxiliary_sonar.py'),run_name='__main__')
finally:
    output.mkdir(parents=True,exist_ok=True)
    (output/'owner-controls.json').write_text(json.dumps(dict(enabled=enabled,
        scope='Original raw IMU/DVL/pressure; owner lifecycle only, no acoustic likelihood or map acceptance',
        controls=observations),indent=2)+'\n')
