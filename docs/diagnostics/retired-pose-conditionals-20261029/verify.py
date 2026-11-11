"""Recompute every retained delayed-pose recovery, without raw sensor copies."""
import argparse
import gzip
import hashlib
import json
from pathlib import Path
import sys
import numpy as np

root=Path(__file__).resolve().parent;library=root.parents[2]
parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--source-tree',type=Path)
args=parser.parse_args();read=lambda name:json.loads((root/name).read_text())
for name,digest in read('sha256.json').items():
    assert hashlib.sha256((root/name).read_bytes()).hexdigest()==digest,name
q=read('qualification.json')
if args.source_tree:
    for name,digest in q['source_sha256'].items():assert hashlib.sha256((args.source_tree/name).read_bytes()).hexdigest()==digest,name
sys.path.insert(0,str((args.source_tree or library)/'src'))
from posim_slam.retired_pose_archive import RetiredPoseArchive

packets=json.loads(gzip.decompress((root/'archive-packets.json.gz').read_bytes()))
expected=json.loads(gzip.decompress((root/'recovery.json.gz').read_bytes()))
timing=json.loads((library/'docs/diagnostics/joint-surface-ranges-20261029/surface-pose-retention.json').read_text())['rows']
assert hashlib.sha256(gzip.decompress((root/'states.csv.gz').read_bytes())).hexdigest()==q['states_sha256']
rows=expected['rows'];by_stamp={}
for row in rows:by_stamp.setdefault(row['stamp_ns'],[]).append(row)
archive=RetiredPoseArchive(packets[0]['epoch']);count=retired=0
assert len(packets)==q['archive_packets']==178
assert sum(len(p['conditionals']) for p in packets)==q['conditional_nodes']==163
for packet in packets:
    archive.ingest(packet)
    for row in by_stamp.get(packet['stamp_ns'],[]):
        source=timing[row['row']];assert row['pose_stamps']==source['required_pose_stamps_ns']
        got=archive.recover(row['pose_stamps']);cov=got['observable_conditional_covariance'];free=got['unbounded_basis']
        values=(np.sqrt(max(0,np.max(np.diag(cov).reshape(-1,6)[:,:3]))),
                np.sqrt(max(0,np.max(np.diag(cov).reshape(-1,6)[:,3:]))),np.linalg.norm(cov-np.diag(np.diag(cov))),np.linalg.norm(free))
        for name,value in zip(('maximum_conditional_position_sigma_m','maximum_conditional_rotation_sigma_rad',
                               'conditional_covariance_cross_norm','unbounded_pose_norm'),values):
            np.testing.assert_allclose(value,row[name],rtol=2e-10,atol=2e-12)
        assert free.shape[1]==row['unbounded_direction_columns']
        assert got['live_coordinates_treated_as_known'] and not got['independent_pose_observations'] and not got['map_qualified']
        count+=1;retired+=row['retired_sonar_pose']
    cutoff=max(archive.closed_through_ns,packet['stamp_ns']-30_000_000_000)
    protect={('state',t) for row in by_stamp.get(packet['stamp_ns'],[]) for t in row['pose_stamps'] if t<=cutoff}
    archive.expire_through(cutoff,protect=protect)
assert count==len(rows)==len(timing)==664 and retired==356
assert not expected['failures'] and not q['live_factor_installed'] and not q['map_qualified']
print('PASS: 178 snapshots, 163 native conditionals, 664 joint recoveries including 356 retired sonar poses; live-state uncertainty excluded')
