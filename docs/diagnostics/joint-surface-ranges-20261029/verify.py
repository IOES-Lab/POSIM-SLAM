"""Recompute source geometry and complete retention audit; check native receipts."""
import argparse
import gzip
import hashlib
import importlib.util
import json
from pathlib import Path
import sys

import numpy as np

root=Path(__file__).resolve().parent;library=root.parents[2]
parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--source-tree',type=Path)
args=parser.parse_args()
read=lambda path:json.loads(gzip.decompress(path.read_bytes()) if path.suffix=='.gz' else path.read_bytes())
for name,digest in read(root/'sha256.json').items():
    assert hashlib.sha256((root/name).read_bytes()).hexdigest()==digest,name
qualification=read(root/'qualification.json')
if args.source_tree:
    for name,digest in qualification['source_sha256'].items():
        assert hashlib.sha256((args.source_tree/name).read_bytes()).hexdigest()==digest,name
sys.path.insert(0,str(library/'src'))
from posim_slam.range_surface_joint import linearize_surface_ranges
from posim_slam.ping_joint import eliminate_patch
for name in ('accumulated-two-plane','current-raw-support'):
    submitted=read(root/f'{name}-input.json');native=read(root/f'{name}-owner.json')
    submitted['states']={int(k):v for k,v in submitted['states'].items()}
    submitted['planes']={int(k):v for k,v in submitted['planes'].items()}
    submitted.pop('prior_sqrt',None);reference=linearize_surface_ranges(**submitted)
    np.testing.assert_allclose(reference['jacobian'],native['jacobian'],atol=2e-10,rtol=1e-10)
    np.testing.assert_allclose(reference['residual'],native['residual'],atol=2e-10,rtol=1e-10)
    n=6*len(submitted['states']);reduced=eliminate_patch(reference['jacobian'],reference['residual'],retained_columns=n)
    np.testing.assert_allclose(reduced['jacobian'].T@reduced['jacobian'],native['reduced_H'],atol=2e-8,rtol=1e-9)
    assert native['maximum_derivative_error']<2e-6
spec=importlib.util.spec_from_file_location('surface_retention_audit',library/'tools/audit_surface_pose_retention.py')
tool=importlib.util.module_from_spec(spec);spec.loader.exec_module(tool)
source=library/'docs/diagnostics/sonar-low-altitude-15m-20261029'
computed=tool.audit(read(source/'dvl-sonar-prediction.json'),read(source/'dvl-spatial-no-acoustic.json'),read(source/'no-acoustic/events.json.gz'))
retained=read(root/'surface-pose-retention.json')
assert computed['rows']==retained['rows'] and computed['counts']==retained['counts']
for path,digest in retained['source_sha256'].items():
    assert hashlib.sha256((library/path).read_bytes()).hexdigest()==digest,path
assert retained['counts']==dict(predictions=664,with_published_state=664,
    both_endpoint_poses_retained=212,all_source_poses_retained=0,no_published_state=0)
wheel=read(root/'wheel-receipt.json')
assert (wheel['tests_run'],wheel['failures'],wheel['errors'],len(wheel['skipped']))==(461,0,0,16)
assert not qualification['live_surface_fusion_enabled'] and not qualification['native_stream_binary_updated']
assert not qualification['truth_is_estimator_input'] and not qualification['map_qualified']
print('PASS: source/native retained controls and all 664 source-pose audits; persistent surface ownership remains open')
