"""Read-only calibration/conditional reproduction with public POSIM-SLAM."""
from pathlib import Path
import gzip,json
import numpy as np
from posim_slam.native_surface import NativeSurfaceSnapshot
from posim_slam.native_surface_ranges import prepare_native_surface_ranges,linearize_native_surface_ranges
from posim_slam.surface_prior import Variable
ROOT=Path(__file__).resolve().parent

def load(path):
    data=path.read_bytes();return json.loads(gzip.decompress(data) if path.suffix=='.gz' else data)

checks=0
for name in ('uncalibrated-dvl-only','uncalibrated-dvl-fls','calibrated-dvl-only','calibrated-dvl-fls'):
    folder=ROOT/name;initial=NativeSurfaceSnapshot.read(load(folder/'snapshot-initial.json.gz'))
    first=next(v for v in initial.order if v.kind=='state');g=initial.joint_marginal((first,))
    if name.startswith('calibrated'):
        assert g['root_constraint_rank']==6
        np.testing.assert_allclose(g['observable_covariance'][9:15,9:15],0,atol=1e-13)
    else:assert g['root_constraint_rank']==0
    checks+=1
    request=load(folder/'original-request.json');request['planes']={int(k):v for k,v in request['planes'].items()}
    before=NativeSurfaceSnapshot.read(load(folder/'snapshot-before.json.gz'))
    after=NativeSurfaceSnapshot.read(load(folder/'snapshot-after.json.gz'))
    if name.endswith('dvl-fls'):
        try:prepare_native_surface_ranges(before,**request)
        except ValueError as error:assert str(error)=='dvl_support_not_planar'
        else:raise AssertionError('Invalid original FLS correspondence accepted')
        a=load(folder/'snapshot-before.json.gz');b=load(folder/'snapshot-after.json.gz')
        assert all(a[k]==b[k] for k in ('variables','delta','conditionals','owner_revision','prior_revision','range_ledger','information'))
    else:
        prepared=prepare_native_surface_ranges(before,**request).increment.as_packet();stored=load(folder/'increment.json.gz')
        np.testing.assert_allclose(prepared['jacobian'],stored['jacobian'],atol=2e-9,rtol=2e-9)
        np.testing.assert_allclose(prepared['offset'],stored['offset'],atol=2e-9,rtol=2e-9)
        request['planes']={i:{k:v for k,v in p.items() if not k.startswith('initial_')} for i,p in request['planes'].items()}
        for suffix in ('after','later'):
            s=NativeSurfaceSnapshot.read(load(folder/('snapshot-'+suffix+'.json.gz')))
            assert linearize_native_surface_ranges(s,**request).diagnostics['all_support_checked']
    controls=load(folder/'range-owner-controls.json.gz')
    for report in controls['reports']:
        suffix='later' if 'continued_original_support' in report else 'after'
        s=NativeSurfaceSnapshot.read(load(folder/('snapshot-'+suffix+'.json.gz')))
        recorded=report['joint'];order=tuple(Variable(**v) for v in recorded['order'])
        result=s.joint_marginal(order)
        for key in ('deltas','linearized_mode','observable_covariance'):
            np.testing.assert_allclose(result[key],recorded[key],atol=3e-9,rtol=3e-9)
        assert result['workspace_bound_bytes']<=32*1024*1024
        assert result['root_rank']==recorded['root_rank']
        checks+=1
for path in sorted((ROOT/'owner-traces').glob('*.json.gz')):
    for entry in load(path):
        state=entry['response'].get('state')
        if not state or 'information' not in state:continue
        order=tuple(Variable(**v) for v in state['order']);info=state['information'];n=sum(v.size for v in order)
        j=np.array(state['joint_jacobian']);r=np.array(state['joint_residual'])
        c=np.array(info.get('constraint_jacobian',np.zeros((0,n))))
        d=np.array(info.get('constraint_residual',[]));kkt=np.block([[j.T@j,c.T],[c,np.zeros((len(c),len(c)))]])
        covariance=np.linalg.inv(kkt)[:n,:n]
        expected=np.array(state['delta'])+np.linalg.solve(kkt,np.r_[-j.T@r,-d])[:n]
        packet=dict(schema='posim-slam-native-surface-snapshot-1',snapshot_id=1,owner_revision=state['owner_revision'],
            prior_revision=state['prior']['revision'],epoch=state['prior']['epoch'],stamp_ns=3*10**9,
            variables=state['order'],delta=state['delta'],conditionals=state['conditionals'],
            archive_closed_through_ns=0,protected_states=[],range_ledger=state['prior']['ledger'],information=info)
        got=NativeSurfaceSnapshot.read(packet).joint_marginal(order)
        np.testing.assert_allclose(got['observable_covariance'],covariance,atol=3e-8,rtol=2e-8)
        np.testing.assert_allclose(got['linearized_mode'],expected,atol=3e-8,rtol=2e-8)
        checks+=1
print('PASS:',checks,'initial calibration, nonlinear support and independent KKT controls; no map qualification')
