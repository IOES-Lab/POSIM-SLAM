"""Read-only reproduction with installed POSIM-SLAM and NumPy; no simulator truth."""
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
for mode in ('dvl-only','dvl-fls'):
    folder=ROOT/('guarded-'+mode)
    controls=load(folder/'range-owner-controls.json.gz')
    request=load(folder/'original-request.json');request['planes']={int(k):v for k,v in request['planes'].items()}
    before=NativeSurfaceSnapshot.read(load(folder/'snapshot-before.json.gz'))
    after=NativeSurfaceSnapshot.read(load(folder/'snapshot-after.json.gz'))
    if mode=='dvl-fls':
        try:prepare_native_surface_ranges(before,**request)
        except ValueError as error:assert str(error)=='dvl_support_not_planar'
        else:raise AssertionError('Original invalid correspondence accepted')
        a=load(folder/'snapshot-before.json.gz');b=load(folder/'snapshot-after.json.gz')
        assert all(a[k]==b[k] for k in ('owner_revision','prior_revision','variables','delta','conditionals','range_ledger','information'))
    else:
        prepared=prepare_native_surface_ranges(before,**request).increment.as_packet();stored=load(folder/'increment.json.gz')
        np.testing.assert_allclose(prepared['jacobian'],stored['jacobian'],atol=1e-9,rtol=1e-9)
        np.testing.assert_allclose(prepared['offset'],stored['offset'],atol=1e-9,rtol=1e-9)
        request['planes']={i:{k:v for k,v in p.items() if not k.startswith('initial_')} for i,p in request['planes'].items()}
        for suffix in ('after','later'):
            s=NativeSurfaceSnapshot.read(load(folder/('snapshot-'+suffix+'.json.gz')))
            assert linearize_native_surface_ranges(s,**request).diagnostics['all_support_checked']
    for record in controls['reports']:
        suffix='later' if 'continued_original_support' in record else 'after'
        s=NativeSurfaceSnapshot.read(load(folder/('snapshot-'+suffix+'.json.gz')))
        recorded=record['joint'];order=tuple(Variable(**v) for v in recorded['order'])
        result=s.joint_marginal(order)
        for key in ('deltas','linearized_mode','observable_covariance','root_singular_values'):
            np.testing.assert_allclose(result[key],recorded[key],atol=2e-9,rtol=2e-9)
        assert result['root_rank']==recorded['root_rank']
        assert result['workspace_bound_bytes']<=32*1024*1024
        assert not result['live_coordinates_treated_as_known'] and not result['map_qualified']
        if mode=='dvl-only':
            assert result['root_rank']==result['root_columns']-1
            free=result['unbounded_basis'][-3:]
            assert np.linalg.matrix_rank(free,tol=1e-8)==1
        checks+=1
for path in sorted((ROOT/'owner-traces').glob('*.json.gz')):
    for entry in load(path):
        state=entry['response'].get('state',{})
        if 'information' not in state:continue
        compact=state['information'];j=np.array(compact['jacobian']);r=np.array(compact['residual'])
        raw=np.array(state['joint_jacobian']);residual=np.array(state['joint_residual'])
        for a,b in ((j.T@j,raw.T@raw),(j.T@r,raw.T@residual),(r@r,residual@residual)):
            np.testing.assert_allclose(a,b,atol=3e-8,rtol=2e-9)
        checks+=1
print('PASS:',checks,'recorded joint-information/conditional controls; no map qualification')
