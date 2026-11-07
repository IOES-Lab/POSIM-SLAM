"""Recompute delayed joint information, native controls and all timing categories."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import sys

import numpy as np

root=Path(__file__).resolve().parent;library=root.parents[2]
parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--source-tree',type=Path);parser.add_argument('--native-probe',type=Path);parser.add_argument('--owner-probe',type=Path)
args=parser.parse_args();read=lambda path:json.loads(path.read_text())
for name,digest in read(root/'sha256.json').items():assert hashlib.sha256((root/name).read_bytes()).hexdigest()==digest,name
qualification=read(root/'qualification.json')
if args.source_tree:
    for name,digest in qualification['source_sha256'].items():
        assert hashlib.sha256((args.source_tree/name).read_bytes()).hexdigest()==digest,name
sys.path.insert(0,str(library/'src'))
from posim_slam.surface_prior import Variable,RangeLedger,SurfacePrior,marginalize_surface


def prior(value):
    return SurfacePrior(tuple(Variable(**v) for v in value['order']),np.array(value['jacobian']),np.array(value['offset']),
        RangeLedger(value['ledger']['recent'],value['ledger']['closed_through_ns']),value['epoch'],value['revision'])


def recompute(value):
    value=dict(value);value['order']=tuple(Variable(**v) for v in value['order'])
    if 'previous' in value:value['previous']=prior(value['previous'])
    return marginalize_surface(**value)


def compare(actual,expected):
    for a,b in ((actual.jacobian.T@actual.jacobian,expected.jacobian.T@expected.jacobian),
                (actual.jacobian.T@actual.offset,expected.jacobian.T@expected.offset),
                (actual.offset@actual.offset,expected.offset@expected.offset)):
        np.testing.assert_allclose(a,b,atol=2e-8,rtol=2e-10)
    assert actual.ledger==expected.ledger


batch=recompute(read(root/'batch-input.json'))
first=recompute(read(root/'delayed-first-input.json'));second=recompute(read(root/'delayed-next-input.json'))
compare(prior(read(root/'delayed-first-output.json')),first)
compare(prior(read(root/'delayed-next-output.json')),second);compare(second,batch)
owner=read(root/'owner-next-output.json');compare(prior(owner['prior']),batch)
assert owner['resident_pose_ids']==[2,3,4] and owner['maximum_owner_derivative_error']<2e-6
assert len(batch.ledger.recent)==28 and ('state',1) not in [v.key for v in batch.order]
if args.native_probe or args.owner_probe:
    for binary,prefix,is_owner in ((args.native_probe,'delayed',False),(args.owner_probe,'owner',True)):
        if binary is None:continue
        previous=None
        for phase in ('first','next'):
            value=read(root/f'{prefix}-{phase}-input.json')
            if previous is not None:value['previous']=previous
            executed=subprocess.run([str(binary.resolve())],input=json.dumps(value),text=True,capture_output=True,check=True,timeout=30)
            current=json.loads(executed.stdout);previous=current['prior'] if is_owner else current
        compare(prior(previous),batch)
late=read(root/'delayed-source-paths.json');source=library/late['source']
assert hashlib.sha256(source.read_bytes()).hexdigest()==late['source_sha256']
counts={};ages=[]
for row in read(source)['rows']:
    key='sonar_first' if row['sonar_stamp_ns']<row['dvl_stamp_ns'] else 'dvl_first'
    entry=counts.setdefault(key,dict(predictions=0,sonar_pose_retained=0,both_endpoints_retained=0,all_sources_retained=0))
    entry['predictions']+=1;entry['sonar_pose_retained']+=int(row['sonar_stamp_ns'] not in row['missing_pose_stamps_ns'])
    entry['both_endpoints_retained']+=int(row['both_endpoint_poses_retained']);entry['all_sources_retained']+=int(row['all_source_poses_retained'])
    ages.append((row['available_stamp_ns']-min(row['required_pose_stamps_ns']))/1e9)
assert counts==late['directions'] and sum(v['predictions'] for v in counts.values())==664
assert max(ages)==late['maximum_original_support_age_s']==30. and float(np.median(ages))==late['median_original_support_age_s']
wheel=read(root/'wheel-receipt.json');assert (wheel['tests_run'],wheel['failures'],wheel['errors'],len(wheel['skipped']))==(468,0,0,16)
assert not qualification['live_worker_updated'] and not qualification['truth_is_estimator_input'] and not qualification['map_qualified']
print('PASS: persistent joint Gaussian/owner controls and all 664 timing categories; live installation and delayed-state recovery remain open')
