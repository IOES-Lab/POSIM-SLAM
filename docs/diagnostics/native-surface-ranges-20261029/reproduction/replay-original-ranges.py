from pathlib import Path
import gzip,importlib.util,json,os,runpy,sys,time
import numpy as np
base=Path(__file__).resolve().parent;prior=base.parent/'native-surface-transport-20261029'
sys.path[:0]=[str(base/'library/src'),str(base/'library/tools'),str(prior/'replay-driver/scripts')]
import posim_slam.live_client as transport
from posim_slam.native_surface_ranges import prepare_native_surface_ranges
BaseClient=transport.LiveClient
spec=importlib.util.spec_from_file_location('original_block',base/'original-block.py')
original=importlib.util.module_from_spec(spec);spec.loader.exec_module(original)
include_sonar=os.environ['INCLUDE_ORIGINAL_FLS']=='1'
output=Path(sys.argv[sys.argv.index('--output')+1]);controls=[];reports=[]

def encode(value):
    if isinstance(value,np.ndarray):return value.tolist()
    if isinstance(value,np.generic):return value.item()
    raise TypeError(type(value).__name__)

class ControlledClient(BaseClient):
    def __init__(self,binary,calibration,log):
        path=Path(calibration);value=json.loads(path.read_text())
        value.update(enable_surface_owner=True,surface_owner_epoch='original-surface-rows-control')
        path.write_text(json.dumps(value));super().__init__(binary,calibration,log)
        self.control_request=0;self.submitted=False

    def control(self,method,*args):
        self.control_request+=1;ident=self.control_request;method(ident,*args)
        deadline=time.monotonic()+15
        while time.monotonic()<deadline:
            values=super().poll()
            if values:
                if len(values)!=1 or values[0]['type']!='surface_control' or values[0]['request_id']!=ident:
                    raise ValueError('Sensor clock changed during diagnostic owner control')
                reply=values[0];controls.append(reply)
                if not reply['accepted']:raise ValueError('Owner rejected original rows: '+str(reply))
                return reply
            time.sleep(.002)
        raise TimeoutError('Original surface owner control timeout')

    def capture(self,name):
        reply=self.control(self.surface_snapshot)
        raw=Path(reply['file']['path']).read_bytes()
        (output/(name+'.json.gz')).write_bytes(gzip.compress(raw,mtime=0))
        return self.read_surface_snapshot(reply)

    def poll(self):
        values=super().poll()
        for value in values:
            if value.get('type')!='state':continue
            # The original history permits lag <= 30 s, including its boundary.
            self.control(self.surface_expire,max(0,value['stamp_ns']-30_000_000_000-1))
            if self.submitted or value['stamp_ns']!=original.first['available_stamp_ns']:continue
            self.submitted=True;start=time.perf_counter();snapshot=self.capture('snapshot-before')
            request,source=original.prepare(snapshot,include_sonar)
            (output/'original-request.json').write_text(json.dumps(request,default=encode,indent=2)+'\n')
            try:block=prepare_native_surface_ranges(snapshot,**request)
            except ValueError as error:
                after=self.capture('snapshot-after');self.control(self.surface_release)
                assert after.prior_revision==snapshot.prior_revision
                assert after.owner_revision==snapshot.owner_revision
                assert after.ledger==snapshot.ledger
                assert not any(v.kind=='plane' for v in after.order)
                reports.append(dict(source=source,rejected_before_native_install=True,reason=str(error),
                    native_prior_ledger_and_coordinates_unchanged=True,
                    total_snapshot_prepare_reject_recover_ms=(time.perf_counter()-start)*1000))
                continue
            (output/'increment.json.gz').write_bytes(gzip.compress(json.dumps(block.increment.as_packet()).encode(),mtime=0))
            reply=self.control(self.surface_update,block.increment)
            if not reply.get('optimized'):raise ValueError('Original range installation optimization failed: '+str(reply))
            after=self.capture('snapshot-after');self.control(self.surface_release)
            stamps=sorted({o['stamp_ns'] for o in request['observations']})
            report=dict(source=source,rows=block.diagnostics,reply=reply,
                before=snapshot.recover(stamps),after=after.recover(stamps),
                total_snapshot_prepare_install_recover_ms=(time.perf_counter()-start)*1000)
            reports.append(report)
        return values

transport.LiveClient=ControlledClient
try:runpy.run_path(str(prior/'replay-driver/scripts/replay_recorded_auxiliary_sonar.py'),run_name='__main__')
finally:
    output.mkdir(parents=True,exist_ok=True)
    (output/'range-owner-controls.json').write_text(json.dumps(dict(include_sonar=include_sonar,
        physical_identity_verified=False,noise_calibrated=False,map_qualified=False,controls=controls,reports=reports),default=encode,indent=2)+'\n')
