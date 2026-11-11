"""Recompute the recorded joint marginal; no SDK or application imports."""
from pathlib import Path
import gzip
import hashlib
import importlib.util
import json
import numpy as np
from posim_slam.native_surface import NativeSurfaceSnapshot
from posim_slam.native_surface_information import joint_surface_marginal
from posim_slam.surface_prior import Variable

ROOT=Path(__file__).resolve().parent


def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    for name,digest in json.loads((ROOT/'sha256.json').read_text()).items():
        assert sha(ROOT/name)==digest,name
    assessment=json.loads((ROOT/'assessment.json').read_text())
    p=assessment['protocol']
    with gzip.open(ROOT/'native-snapshot.json.gz','rb') as stream:
        raw=stream.read(p['snapshot_source_bytes']+1)
    assert len(raw)==p['snapshot_source_bytes']<=128*2**20
    assert hashlib.sha256(raw).hexdigest()==p['snapshot_sha256']
    snapshot=NativeSurfaceSnapshot.read(json.loads(raw))
    assert snapshot._archive.max_recovery_bytes==32*2**20
    def order(stamps):
        return tuple(Variable('state',t,6,snapshot.reference('state',t).reference[:12]) for t in stamps)
    got=joint_surface_marginal(snapshot,order(p['stamps_ns']))
    with np.load(ROOT/'expected-marginal.npz',allow_pickle=False) as z:
        for name,key in [('covariance','observable_covariance'),('deltas','deltas'),
                ('linearized_mode','linearized_mode')]:
            np.testing.assert_allclose(got[key],z[name],rtol=1e-7,atol=1e-8)
        assert z['unbounded_basis'].shape==got['unbounded_basis'].shape==(24,0)
    assert got['workspace_bound_bytes']<=32*2**20
    assert not got['live_coordinates_treated_as_known'] and not got['independent_pose_observations']
    assert not got['map_qualified']
    spec=importlib.util.spec_from_file_location('posim_slam.recorded_native_information',ROOT/'original_query.py')
    old=importlib.util.module_from_spec(spec);spec.loader.exec_module(old)
    for row in json.loads((ROOT/'equivalence.json').read_text())['tests']:
        query=order(row['stamps'])
        current=joint_surface_marginal(snapshot,query)
        try:previous=old.joint_surface_marginal(snapshot,query)
        except ValueError as error:
            assert row['previous_capacity_rejection'] and str(error)=='joint_recovery_workspace_capacity'
            continue
        assert not row['previous_capacity_rejection']
        for key in ['deltas','linearized_mode','observable_covariance']:
            np.testing.assert_allclose(current[key],previous[key],rtol=2e-10,atol=2e-10)
    before=np.genfromtxt(ROOT/'states-before.csv',delimiter=',',skip_header=1)
    after=np.genfromtxt(ROOT/'states-snapshot.csv',delimiter=',',skip_header=1)
    assert before.shape==after.shape==(271,8)
    np.testing.assert_array_equal(before[:,0],after[:,0])
    np.testing.assert_allclose(before[:,1:],after[:,1:],rtol=1e-9,atol=1e-9)
    old_cases=json.loads((ROOT/'background-original-record.json').read_text())['cases']
    exact_cases=json.loads((ROOT/'background-exact-record.json').read_text())['cases']
    assert len(old_cases)==80 and len(exact_cases)==20
    assert all(c['pixels']==599*256 for c in old_cases+exact_cases)
    assert {c['pose_arm'] for c in old_cases}=={'navigation_mean','interpolated_assessment'}
    assert len({c['frame'] for c in old_cases})==8 and len({c['frame'] for c in exact_cases})==4
    assert all(c['return_mask_disagreements']==0 for c in exact_cases if c['scene_arm']=='full_reference')
    print('PASS: recorded 24D joint marginal, original capacity controls, shared covariance and all 271 states; no map acceptance')


if __name__=='__main__':main()
