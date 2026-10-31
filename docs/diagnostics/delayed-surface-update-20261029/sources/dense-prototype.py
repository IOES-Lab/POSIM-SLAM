"""Prepare one joint late-range update of live information and conditionals.

The live prior must be the sole, complete information consumed by this update.
It is replaced together with the affected retiring-state conditionals. Modes
are linearization coordinates, never independent pose observations. The caller
owns native factor retirement, nonlinear relinearization and publication of the
whole result; this reference operation does not update a native worker.
"""
from dataclasses import dataclass
import copy
import hashlib
import json

import numpy as np

from .ping_joint import split_patch_information
from .retired_pose_archive import (Conditional,RetiredPoseArchive,coordinates,
                                    merge_reference)
from .surface_prior import SurfacePrior,Variable,layout,_keys


@dataclass(frozen=True)
class DelayedSurfaceUpdate:
    prior: SurfacePrior
    archive: RetiredPoseArchive
    changed_conditionals: tuple
    joint_columns: int
    workspace_bound_bytes: int


def _prior_digest(prior):
    fields=dict(epoch=prior.epoch,revision=prior.revision,
        order=[dict(kind=v.kind,ident=v.ident,size=v.size,reference=v.reference) for v in prior.order],
        keys=sorted(prior.ledger.recent),cutoff=prior.ledger.closed_through_ns,
        shape=prior.jacobian.shape)
    digest=hashlib.sha256(json.dumps(fields,sort_keys=True,separators=(',',':')).encode())
    for value in (prior.jacobian,prior.offset):digest.update(np.asarray(value,dtype='<f8').tobytes())
    return digest.hexdigest()


def _closure(archive,query):
    needed=set();pending=[]
    for key in query:
        if key in archive.root:continue
        if key not in archive.producers:raise ValueError('late_range_coordinate_not_owned')
        pending.append(archive.producers[key][0])
    while pending:
        ident=pending.pop()
        if ident in needed:continue
        needed.add(ident);node=archive.nodes[ident];cols=node.source_coordinates
        pending.extend(archive.producers[cols[i]][0] for i in node.kept if cols[i] not in archive.root)
    return tuple(sorted(needed))


def _remaining_order(order,remaining):
    present=set(remaining);out=[]
    for v in order:
        parts=[i for i in range(v.size) if (v.kind,v.ident,i) in present]
        if not parts:continue
        if parts==list(range(v.size)):out.append(v)
        elif v.kind=='state' and v.size==15 and parts==list(range(6)):
            out.append(Variable(v.kind,v.ident,6,v.reference[:12]))
        else:raise ValueError('unsupported_partial_late_range_variable')
    if coordinates(out)!=tuple(remaining):raise ValueError('late_range_coordinate_order')
    return tuple(out)


def prepare_delayed_ranges(prior,archive,*,order,delta,jacobian,residual,range_keys,
                           source_stamp_ns,expected_revision,max_workspace_bytes=32*1024*1024):
    """Replace live marginal AND affected conditional rows in one pure result.

    New J/r are jointly whitened raw-range rows, evaluated at delta in immutable
    owner references. Correlations within that block must already be present.
    The rows must exclude old prior/conditional rows, damping and fitted-pose or
    fitted-plane priors. New unknown planes may be appended with zero prior
    information. Original range identities enter the persistent ledger once.

    The archive's current coordinates and complete live prior must describe the
    same snapshot. The first consumed live prior has revision zero and no
    previously fused raw ranges. Later calls consume the paired result, whose
    prior identity is bound to its updated conditionals. All input objects
    remain unchanged on success or failure.
    Publish neither returned part without its companion. A native owner still
    needs to retire/replace the corresponding factors on its owning thread.
    """
    if not isinstance(prior,SurfacePrior) or not isinstance(archive,RetiredPoseArchive):
        raise ValueError('owned_live_prior_and_conditionals_required')
    if (prior.epoch!=archive.epoch or type(source_stamp_ns) is not int
            or source_stamp_ns!=archive.stamp_ns or source_stamp_ns<=0
            or type(expected_revision) is not int or expected_revision!=prior.revision):
        raise ValueError('late_range_snapshot_or_revision_mismatch')
    if prior.order!=archive.root_variables:raise ValueError('complete_matching_live_prior_order_required')
    bound=getattr(archive,'_delayed_live_prior_digest',None)
    if ((bound is None and (prior.revision!=0 or prior.ledger.recent or prior.ledger.closed_through_ns))
            or (bound is not None and bound!=_prior_digest(prior))):
        raise ValueError('late_range_prior_and_conditionals_must_be_published_together')
    if (type(max_workspace_bytes) is not int or not 1024<=max_workspace_bytes<=128*1024*1024):
        raise ValueError('bounded_late_range_workspace_required')
    order,_,count=layout(order,retirement_source=True)
    delta=np.asarray(delta,dtype=float);j=np.asarray(jacobian,dtype=float);r=np.asarray(residual,dtype=float)
    keys=_keys(range_keys)
    if (delta.shape!=(count,) or j.ndim!=2 or j.shape[1]!=count or not 1<=len(j)<=4096
            or r.shape!=(len(j),) or not 1<=len(keys)<=len(j)
            or not all(np.isfinite(a).all() for a in (delta,j,r))):
        raise ValueError('finite_new_delayed_raw_range_rows_required')
    if any(key[0]>source_stamp_ns or ('state',key[0]) not in {v.key for v in order} for key in keys):
        raise ValueError('late_range_original_measurement_pose_required')
    ledger=prior.ledger.admit(keys)
    new_planes=[];block_cols=coordinates(order);block_values=dict(zip(block_cols,delta))
    for v in order:
        if v.key not in archive.references:
            if v.kind!='plane':raise ValueError('late_range_variable_not_owned')
            new_planes.append(v)
        else:
            merge_reference(archive.references[v.key],v)
            if (v.kind=='state' and v.ident<=archive.closed_through_ns
                    and v.key not in archive.protected and (v.kind,v.ident,0) not in archive.root):
                raise ValueError('late_range_pose_admission_closed')
    for key,value in block_values.items():
        if key in archive.root and not np.isclose(value,archive.root[key],rtol=0,atol=1e-12):
            raise ValueError('late_range_live_linearization_snapshot_mismatch')
    live_order=prior.order+tuple(new_planes);_,_,live_count=layout(live_order)
    added=set(coordinates(new_planes))
    changed=_closure(archive,[key for key in block_cols if key not in added])
    descriptors={v.key:v for v in live_order}
    for ident in changed:
        for v in archive.nodes[ident].variables:
            descriptors[v.key]=merge_reference(descriptors[v.key],v) if v.key in descriptors else v
    # Do not pull an already eliminated inertial tail into a pose-only closure.
    # The selected nodes, not the archive's largest historical descriptor, own
    # the coordinates required by this transaction.
    for v in order:
        descriptors[v.key]=merge_reference(descriptors[v.key],v) if v.key in descriptors else v
    joint_order=tuple(descriptors.values());_,_,columns=layout(joint_order,retirement_source=True)
    cols=coordinates(joint_order);lookup={key:i for i,key in enumerate(cols)}
    rows=len(prior.jacobian)+len(j)+sum(archive.nodes[i].noise.shape[1] for i in changed)
    if rows>4096:raise ValueError('late_range_joint_row_budget')
    # Reserve numeric arrays for assembly/copies and compact QR/SVD. Library
    # allocator/LAPACK scratch is separate from this matrix budget. The live
    # estimator's state/window and archive limits remain unchanged.
    workspace=3*(rows+1)*(columns+1)*8+6*(columns+1)**2*8
    if workspace>min(max_workspace_bytes,archive.max_recovery_bytes):raise ValueError('late_range_workspace_capacity')
    assembled=np.zeros((rows,columns));offset=np.zeros(rows);start=0
    size=len(prior.jacobian)
    assembled[:size,[lookup[k] for k in coordinates(prior.order)]]=prior.jacobian
    offset[:size]=prior.offset;start+=size
    for ident in changed:
        node=archive.nodes[ident];n=node.noise.shape[1]
        if not n:continue
        # Columns of the emitted noise factor are orthogonal, with their
        # original variances. Free directions contribute no information rows.
        white=node.noise.T/np.sum(node.noise**2,axis=0)[:,None]
        source=node.source_coordinates
        assembled[start:start+n,[lookup[source[i]] for i in node.removed]]=white
        assembled[start:start+n,[lookup[source[i]] for i in node.kept]]=-white@node.regression
        offset[start:start+n]=-white@node.offset;start+=n
    assembled[start:,[lookup[k] for k in block_cols]]=j
    offset[start:]=r-j@delta
    if rows>columns+1:
        compact=np.linalg.qr(np.column_stack((assembled,offset)),mode='r')
        assembled=compact[:,:columns];offset=compact[:,columns]
    nodes=dict(archive.nodes);remaining=cols
    for ident in changed:
        old=archive.nodes[ident];removed_keys=[old.source_coordinates[i] for i in old.removed]
        positions={key:i for i,key in enumerate(remaining)}
        removed=[positions[key] for key in removed_keys];removed_set=set(removed)
        kept=[i for i in range(len(remaining)) if i not in removed_set]
        current=_remaining_order(joint_order,remaining)
        split=split_patch_information(assembled[:,kept+removed],offset,retained_columns=len(kept))
        nodes[ident]=Conditional.read(dict(id=ident,
            variables=[dict(kind=v.kind,ident=v.ident,size=v.size,reference=v.reference) for v in current],
            kept_indices=kept,removed_indices=removed,regression=split['backsub_matrix'],
            offset=split['backsub_offset'],noise_factor=split['observable_noise_factor'],
            unbounded_basis=split['unobservable_basis']))
        assembled=split['jacobian'];offset=split['residual']
        remaining=tuple(remaining[i] for i in kept)
    if remaining!=coordinates(live_order):raise ValueError('late_range_final_live_order_mismatch')
    if not len(assembled):assembled=np.zeros((1,live_count));offset=np.zeros(1)
    if len(assembled)>live_count+1:
        compact=np.linalg.qr(np.column_stack((assembled,offset)),mode='r')
        assembled=compact[:,:live_count];offset=compact[:,live_count]
    result_prior=SurfacePrior(live_order,assembled,offset,ledger,prior.epoch,prior.revision+1)
    if sum(n.matrix_bytes for n in nodes.values())>archive.max_matrix_bytes:
        raise ValueError('updated_conditional_archive_capacity')
    live=dict(archive.root)
    live.update({key:block_values[key] for key in added})
    producers,references=archive._validate(nodes,live,live_order)
    result_archive=copy.copy(archive)
    result_archive.nodes=nodes;result_archive.root=live;result_archive.root_variables=live_order
    result_archive.producers=producers;result_archive.references=references;result_archive._memo={}
    result_archive._delayed_live_prior_digest=_prior_digest(result_prior)
    return DelayedSurfaceUpdate(result_prior,result_archive,changed,columns,workspace)
