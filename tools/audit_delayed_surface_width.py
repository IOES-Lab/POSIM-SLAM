"""Audit recorded conditional widths and optional null-information controls.

This reads independent library records, without an application import. Null
controls use an explicit zero-information live root and zero synthetic rows;
they measure conditional algebra, not a navigation posterior or real fusion.
"""
import argparse
import gzip
import hashlib
import json
from pathlib import Path
import time

import numpy as np

from posim_slam.delayed_surface_update import _closure,prepare_delayed_ranges
from posim_slam.retired_pose_archive import RetiredPoseArchive
from posim_slam.surface_prior import SurfacePrior,Variable,RangeLedger


def read(path):
    return json.loads(gzip.decompress(path.read_bytes()) if path.suffix=='.gz' else path.read_bytes())


def null_control(archive,row,index,dense):
    before=archive.recover(row['required_pose_stamps_ns'])
    order=tuple(Variable('state',t,6,archive.references[('state',t)].reference[:12]) for t in row['required_pose_stamps_ns'])
    order+=(Variable('plane',987,3,(0,0,1,0)),)
    delta=np.r_[before['deltas'],np.zeros(3)];live_columns=len(archive.root)
    prior=SurfacePrior(archive.root_variables,np.zeros((1,live_columns)),np.zeros(1),RangeLedger(),archive.epoch)
    start=time.perf_counter()
    result=prepare_delayed_ranges(prior,archive,order=order,delta=delta,jacobian=np.zeros((1,len(delta))),residual=[0.],
        range_keys=[(row['sonar_stamp_ns'],900000000+index,row['beam_id'])],source_stamp_ns=archive.stamp_ns,expected_revision=0)
    after=result.archive.recover(row['required_pose_stamps_ns'])
    means=float(np.max(np.abs(before['deltas']-after['deltas'])))
    cov=float(np.linalg.norm(before['observable_conditional_covariance']-after['observable_conditional_covariance']))
    information=float(np.linalg.norm(result.new_live_rows.jacobian));offset=float(np.linalg.norm(result.new_live_rows.offset))
    if max(means,cov,information,offset)>=1e-8:raise ValueError('null_information_changed_conditional_problem')
    return dict(row=index,dense_columns=dense,peak_front_columns=result.peak_front_columns,
        changed_conditionals=len(result.changed_conditionals),workspace_bound_bytes=result.workspace_bound_bytes,
        maximum_mean_change=means,covariance_difference_norm=cov,new_information_rows_norm=information,
        new_information_offset_norm=offset,elapsed_s=time.perf_counter()-start)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--packets',type=Path,required=True)
    parser.add_argument('--requests',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--null-rows',type=int,nargs='*',default=[])
    args=parser.parse_args()
    packets=read(args.packets);requests=read(args.requests)['rows'];selected=set(args.null_rows)
    if not packets or any(i<0 or i>=len(requests) for i in selected):raise ValueError('known_request_rows_required')
    grouped={}
    for i,row in enumerate(requests):grouped.setdefault(row['snapshot_stamp_ns'],[]).append((i,row))
    archive=RetiredPoseArchive(packets[0]['epoch']);results=[];controls=[];start=time.perf_counter()
    for packet in packets:
        archive.ingest(packet)
        for index,row in grouped.get(packet['stamp_ns'],[]):
            query={('state',t,k) for t in row['required_pose_stamps_ns'] for k in range(6)}
            nodes=_closure(archive,query);plane={('plane',987,k) for k in range(3)}
            live=set(archive.root)|plane;all_coords=set(live)|query
            for ident in nodes:all_coords.update(archive.nodes[ident].source_coordinates)
            buckets={};peak=len(query|plane);processed=[]
            def dispatch(coords):
                parents=[archive.producers[key][0] for key in coords if key not in live]
                if parents:buckets.setdefault(min(parents),set()).update(coords)
            dispatch(query|plane)
            for ident in nodes:
                if ident not in buckets:continue
                node=archive.nodes[ident];cols=node.source_coordinates
                front=buckets.pop(ident)|set(cols);peak=max(peak,len(front));processed.append(ident)
                dispatch(front-{cols[i] for i in node.removed})
            if buckets:raise ValueError('unprocessed_symbolic_parent')
            dense=len(all_coords)
            results.append(dict(row=index,source_stamp_ns=packet['stamp_ns'],closure_nodes=len(nodes),
                                dense_columns=dense,symbolic_front_columns=peak,processed_nodes=len(processed)))
            if index in selected:controls.append(null_control(archive,row,index,dense))
        cutoff=max(archive.closed_through_ns,packet['stamp_ns']-30_000_000_000)
        protection={('state',t) for _,r in grouped.get(packet['stamp_ns'],[]) for t in r['required_pose_stamps_ns'] if t<=cutoff}
        archive.expire_through(cutoff,protect=protection)
    if len(results)!=len(requests) or {r['row'] for r in controls}!=selected:raise ValueError('incomplete_original_request_coverage')
    result=dict(scope=__doc__,requests=len(results),dense_over_1024=sum(r['dense_columns']>1024 for r in results),
        max_dense_columns=max(r['dense_columns'] for r in results),max_symbolic_front_columns=max(r['symbolic_front_columns'] for r in results),
        live_complete_prior_recorded=False,packet_keys=sorted(packets[0]),elapsed_s=time.perf_counter()-start,
        fixed_diagnostic_query_horizon_ns=30_000_000_000,source_sha256={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in (args.packets,args.requests)},
        null_controls=controls,synthetic_zero_information_root=True,new_physical_range_measurements=0,
        native_worker_updated=False,navigation_covariance_qualified=False,map_qualified=False,rows=results)
    args.output.write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps({k:v for k,v in result.items() if k not in ('rows','source_sha256','scope')},indent=2))


if __name__=='__main__':main()
