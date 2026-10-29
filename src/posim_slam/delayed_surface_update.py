"""Prepare one joint late-range update of live information and conditionals.

The live prior must be the sole, complete information consumed by this update.
It is replaced together with the affected retiring-state conditionals. Modes
are linearization coordinates, never independent pose observations. The caller
owns native factor retirement, nonlinear relinearization and publication of the
whole result. A native owner that retains its active rows installs only the new
live rows, never the folded verification prior as well. This reference operation
does not update a native worker.
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
class LiveRangeRows:
    order: tuple
    jacobian: np.ndarray
    offset: np.ndarray

    def __post_init__(self):
        order,_,columns=layout(self.order)
        j=np.asarray(self.jacobian,dtype=float);b=np.asarray(self.offset,dtype=float)
        if (j.ndim!=2 or j.shape[1]!=columns or len(j)>columns+1 or b.shape!=(len(j),)
                or not np.isfinite(j).all() or not np.isfinite(b).all()):
            raise ValueError('finite_bounded_live_range_rows_required')
        object.__setattr__(self,'order',order)
        for name in ('jacobian','offset'):
            value=(j if name=='jacobian' else b).copy();value.flags.writeable=False
            object.__setattr__(self,name,value)


@dataclass(frozen=True)
class DelayedSurfaceUpdate:
    prior: SurfacePrior
    archive: RetiredPoseArchive
    new_live_rows: LiveRangeRows
    changed_conditionals: tuple
    peak_front_columns: int
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


@dataclass(frozen=True)
class _Factor:
    order: tuple
    jacobian: np.ndarray
    offset: np.ndarray


def _conditional_rows(node,reserve):
    reserve(node.noise.shape[1],len(node.source_coordinates))
    white=node.noise.T/np.sum(node.noise**2,axis=0)[:,None]
    j=np.zeros((len(white),len(node.source_coordinates)))
    j[:,node.removed]=white;j[:,node.kept]=-white@node.regression
    return _Factor(node.variables,j,-white@node.offset)


def _stack(factors,reserve):
    descriptors={}
    for factor in factors:
        for v in factor.order:
            descriptors[v.key]=merge_reference(descriptors[v.key],v) if v.key in descriptors else v
    order=tuple(descriptors.values());_,_,columns=layout(order,retirement_source=True)
    cols=coordinates(order);lookup={key:i for i,key in enumerate(cols)}
    rows=sum(len(f.jacobian) for f in factors)
    if rows>4096:raise ValueError('late_range_joint_row_budget')
    reserve(rows,columns)
    j=np.zeros((rows,columns));b=np.zeros(rows);start=0
    for factor in factors:
        n=len(factor.jacobian)
        j[start:start+n,[lookup[k] for k in coordinates(factor.order)]]=factor.jacobian
        b[start:start+n]=factor.offset;start+=n
    if rows>columns+1:
        compact=np.linalg.qr(np.column_stack((j,b)),mode='r')
        j=compact[:,:columns];b=compact[:,columns]
    return _Factor(order,j,b)


def _propagate_incoming(incoming,node,front,kept,removed,reserve):
    """Propagate only the new likelihood through the existing conditional.

    x_removed = M x_kept + d + L noise + U free. Integrating unit conditional
    noise gives I + A L L.T A.T. Project free directions afterward. Unlike a
    subtraction of large old normal matrices or their residual projections,
    zero incoming information yields exactly zero live information.
    """
    cols=coordinates(front);positions={key:i for i,key in enumerate(cols)}
    reserve(len(incoming.jacobian),len(cols))
    a=np.zeros((len(incoming.jacobian),len(cols)))
    a[:,[positions[k] for k in coordinates(incoming.order)]]=incoming.jacobian
    old_cols=node.source_coordinates;where={index:i for i,index in enumerate(kept)}
    regression=np.zeros((len(removed),len(kept)))
    regression[:,[where[positions[old_cols[i]]] for i in node.kept]]=node.regression
    removed_rows=a[:,removed]
    j=a[:,kept]+removed_rows@regression;b=incoming.offset+removed_rows@node.offset
    noise=removed_rows@node.noise;free=removed_rows@node.unbounded
    chol=np.linalg.cholesky(np.eye(len(j))+noise@noise.T)
    j=np.linalg.solve(chol,j);b=np.linalg.solve(chol,b)
    if free.shape[1]:
        free=np.linalg.solve(chol,free)
        split=split_patch_information(np.column_stack((j,free)),b,retained_columns=len(kept),include_conditional=False)
        j=split['jacobian'];b=split['residual']
    remaining=tuple(cols[i] for i in kept)
    return _Factor(_remaining_order(front,remaining),j,b)


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
    Publish neither returned part without its companion. A native owner retains
    its active nonlinear rows and installs only new_live_rows, with the updated
    conditionals and ledger. Alternatively replace the consumed whole linear
    system by prior. Installing both duplicates the late information.
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
    needed=_closure(archive,[key for key in block_cols if key not in added])
    live_keys=set(archive.root)|added;pending={};root_factors=[];nodes=dict(archive.nodes)
    changed=[];peak_columns=0;workspace=0;new_node_bytes=0
    def reserve(rows,columns):
        nonlocal workspace,peak_columns
        # Numeric-array reservation; NumPy/LAPACK allocator scratch is separate.
        held=sum(f.jacobian.nbytes+f.offset.nbytes for f in pending.values())
        held+=sum(f.jacobian.nbytes+f.offset.nbytes for f in root_factors)
        amount=held+new_node_bytes+3*(rows+1)*(columns+1)*8+6*(columns+1)**2*8
        if amount>min(max_workspace_bytes,archive.max_recovery_bytes):
            raise ValueError('late_range_workspace_capacity')
        workspace=max(workspace,amount);peak_columns=max(peak_columns,columns)
    def dispatch(factor):
        parents=[archive.producers[key][0] for key in coordinates(factor.order) if key not in live_keys]
        if not parents:root_factors.append(factor);return
        ident=min(parents)
        if ident in pending:factor=_stack((pending.pop(ident),factor),reserve)
        pending[ident]=factor
    reserve(len(j),count)
    dispatch(_Factor(order,j,r-j@delta))
    for ident in needed:
        if ident not in pending:continue
        old=archive.nodes[ident]
        # Re-eliminate one frontal block at a time. Long temporal closures do
        # not become a single dense matrix or enlarge the live state window.
        incoming=pending.pop(ident)
        factor=_stack((incoming,_conditional_rows(old,reserve)),reserve)
        cols=coordinates(factor.order);positions={key:i for i,key in enumerate(cols)}
        removed=[positions[old.source_coordinates[i]] for i in old.removed];removed_set=set(removed)
        kept=[i for i in range(len(cols)) if i not in removed_set]
        split=split_patch_information(factor.jacobian[:,kept+removed],factor.offset,retained_columns=len(kept))
        nodes[ident]=Conditional.read(dict(id=ident,
            variables=[dict(kind=v.kind,ident=v.ident,size=v.size,reference=v.reference) for v in factor.order],
            kept_indices=kept,removed_indices=removed,regression=split['backsub_matrix'],
            offset=split['backsub_offset'],noise_factor=split['observable_noise_factor'],
            unbounded_basis=split['unobservable_basis']))
        new_node_bytes+=nodes[ident].matrix_bytes;changed.append(ident)
        dispatch(_propagate_incoming(incoming,old,factor.order,kept,removed,reserve))
    if pending:raise ValueError('late_range_unprocessed_conditional_parent')
    increment=_stack(tuple(root_factors),reserve)
    live_positions={key:i for i,key in enumerate(coordinates(live_order))}
    reserve(len(increment.jacobian),live_count)
    increment_j=np.zeros((len(increment.jacobian),live_count))
    increment_j[:,[live_positions[k] for k in coordinates(increment.order)]]=increment.jacobian
    live_rows=LiveRangeRows(live_order,increment_j,increment.offset)
    result=_stack((_Factor(prior.order,prior.jacobian,prior.offset),
                   _Factor(live_order,live_rows.jacobian,live_rows.offset)),reserve)
    final_cols=coordinates(result.order);positions={key:i for i,key in enumerate(final_cols)}
    if set(final_cols)!=set(coordinates(live_order)):raise ValueError('late_range_final_live_order_mismatch')
    assembled=result.jacobian[:,[positions[k] for k in coordinates(live_order)]];offset=result.offset
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
    return DelayedSurfaceUpdate(result_prior,result_archive,live_rows,tuple(changed),peak_columns,workspace)
