"""Persistent square-root information for jointly estimated poses and planes.

An unknown surface remains a variable when old source poses are marginalized.
The prior includes pose/plane cross terms and immutable owner references. It
must replace the consumed rows, never accompany an independently fitted plane
prior or a second copy of the same state prior. This module does not establish
echo identity, reconstruct retired support geometry or enable live fusion.
"""
from __future__ import annotations

from dataclasses import dataclass
import numpy as np

from .dvl_scale import rotation
from .ping_joint import eliminate_patch

MAX_COLUMNS=265  # Sixteen inertial states, eight planes and one pressure datum.
MAX_RECENT_KEYS=32768


@dataclass(frozen=True)
class Variable:
    kind: str
    ident: int
    size: int
    reference: tuple

    def __post_init__(self):
        if (self.kind not in ('state','plane','pressure') or type(self.ident) is not int
                or self.ident<(1 if self.kind=='state' else 0)
                or type(self.size) is not int
                or self.size not in ((6,15) if self.kind=='state' else ((3,) if self.kind=='plane' else (1,)))):
            raise ValueError('invalid_persistent_surface_variable')
        value=np.asarray(self.reference,dtype=float)
        count=(12 if self.size==6 else 21) if self.kind=='state' else (4 if self.kind=='plane' else 1)
        if value.shape!=(count,) or not np.isfinite(value).all():
            raise ValueError('complete_persistent_owner_reference_required')
        if self.kind=='state':rotation(value[3:12].reshape(3,3))
        elif self.kind=='plane' and abs(value[:3]@value[:3]-1)>1e-8:raise ValueError('unit_surface_normal_required')
        object.__setattr__(self,'reference',tuple(float(x) for x in value))

    @property
    def key(self):return self.kind,self.ident


def layout(order):
    order=tuple(order)
    if (not order or len(order)>25 or any(not isinstance(v,Variable) for v in order)
            or sum(v.kind=='state' for v in order)>16 or sum(v.kind=='plane' for v in order)>8
            or sum(v.kind=='pressure' for v in order)>1
            or len({v.key for v in order})!=len(order)):
        raise ValueError('bounded_unique_surface_prior_order_required')
    columns={};offset=0
    for variable in order:columns[variable.key]=(offset,variable);offset+=variable.size
    if offset>MAX_COLUMNS:raise ValueError('surface_prior_column_budget')
    return order,columns,offset


def _keys(values):
    result=[]
    for value in values:
        key=tuple(value)
        if (len(key)!=3 or any(type(i) is not int for i in key)
                or key[0]<=0 or min(key[1:])<0):
            raise ValueError('original_integer_range_key_required')
        result.append(key)
    if len(set(result))!=len(result):raise ValueError('reused_surface_range_measurement')
    return frozenset(result)


@dataclass(frozen=True)
class RangeLedger:
    recent: frozenset=frozenset()
    closed_through_ns: int=0

    def __post_init__(self):
        keys=_keys(self.recent)
        if (type(self.closed_through_ns) is not int or self.closed_through_ns<0
                or len(keys)>MAX_RECENT_KEYS or any(k[0]<=self.closed_through_ns for k in keys)):
            raise ValueError('bounded_recent_range_ledger_required')
        object.__setattr__(self,'recent',keys)

    def admit(self,keys,*,close_through_ns=None):
        added=_keys(keys)
        if any(k[0]<=self.closed_through_ns or k in self.recent for k in added):
            raise ValueError('reused_or_closed_surface_range_measurement')
        cutoff=self.closed_through_ns if close_through_ns is None else close_through_ns
        if type(cutoff) is not int or cutoff<self.closed_through_ns:
            raise ValueError('monotonic_range_admission_cutoff_required')
        # The caller closes admission to ALL earlier ranges before pruning.
        # A discarded identity can therefore never become admissible again.
        return RangeLedger(frozenset(k for k in self.recent|added if k[0]>cutoff),cutoff)


@dataclass(frozen=True)
class SurfacePrior:
    order: tuple
    jacobian: np.ndarray
    offset: np.ndarray
    ledger: RangeLedger
    epoch: str
    revision: int=0

    def __post_init__(self):
        order,_,columns=layout(self.order);j=np.asarray(self.jacobian,dtype=float);b=np.asarray(self.offset,dtype=float)
        if (j.ndim!=2 or j.shape[1]!=columns or not 1<=j.shape[0]<=columns+1
                or b.shape!=(len(j),) or not np.isfinite(j).all() or not np.isfinite(b).all()
                or not isinstance(self.ledger,RangeLedger) or not isinstance(self.epoch,str) or not self.epoch.strip()
                or type(self.revision) is not int or self.revision<0):
            raise ValueError('finite_bounded_surface_prior_required')
        j=j.copy();b=b.copy();j.flags.writeable=False;b.flags.writeable=False
        object.__setattr__(self,'order',order);object.__setattr__(self,'jacobian',j);object.__setattr__(self,'offset',b)


def linearize_prior(prior,*,order,delta,epoch):
    """Map one consumed prior into the current order, preserving fixed references.

    delta contains the owner's accumulated reference coordinates. Pose and
    plane rotation mappings must match those used by the nonlinear range rows.
    Missing variables, a changed reference or a different session are errors.
    """
    _,columns,count=layout(order);delta=np.asarray(delta,dtype=float)
    if epoch!=prior.epoch:raise ValueError('surface_prior_epoch_mismatch')
    if delta.shape!=(count,) or not np.isfinite(delta).all():raise ValueError('finite_owner_delta_required')
    j=np.zeros((len(prior.offset),count));source=0
    for variable in prior.order:
        found=columns.get(variable.key)
        if found is None or found[1]!=variable:raise ValueError('changed_or_missing_surface_prior_reference')
        target=found[0];j[:,target:target+variable.size]=prior.jacobian[:,source:source+variable.size];source+=variable.size
    return dict(jacobian=j,residual=j@delta+prior.offset)


def marginalize_surface(*,order,delta,jacobian,residual,retain,epoch,
                       range_keys=(),previous=None,close_through_ns=None):
    """Consume ONE previous prior plus new rows, retaining poses AND surfaces.

    retain lists (kind, ident, size) blocks in the desired output order. An
    inertial state may retain its first six pose coordinates. Every other
    retained block keeps its size and immutable reference. New Jacobians and
    residuals are evaluated at delta. They must contain neither previous prior
    rows nor trial damping. All correlated raw rows belong in this transaction.
    range_keys identify only newly fused original ranges, not support guards.

    Closing an admission cutoff is explicit; the caller must forbid every
    earlier future range factor, including delayed associations. It bounds the
    deduplication ledger without forgetting already consumed measurements.
    """
    order,columns,count=layout(order);delta=np.asarray(delta,dtype=float)
    j=np.asarray(jacobian,dtype=float);r=np.asarray(residual,dtype=float)
    keys=_keys(range_keys)
    if (delta.shape!=(count,) or not np.isfinite(delta).all() or j.ndim!=2 or j.shape[1]!=count
            or not 0<=j.shape[0]<=4096 or r.shape!=(len(j),) or len(keys)>len(j)
            or not np.isfinite(j).all() or not np.isfinite(r).all()
            or not isinstance(epoch,str) or not epoch.strip()):
        raise ValueError('finite_new_surface_rows_required')
    if any(('state',key[0]) not in columns for key in keys):
        raise ValueError('new_range_requires_resident_measurement_pose')
    if previous is None:
        ledger=RangeLedger();revision=0
    else:
        old=linearize_prior(previous,order=order,delta=delta,epoch=epoch)
        j=np.vstack((j,old['jacobian']));r=np.r_[r,old['residual']]
        ledger=previous.ledger;revision=previous.revision+1
    ledger=ledger.admit(keys,close_through_ns=close_through_ns)
    keep=[];retained=[];seen=set()
    for kind,ident,size in retain:
        key=(kind,ident)
        if key in seen or key not in columns:raise ValueError('unique_known_retained_surface_blocks_required')
        seen.add(key);start,v=columns[key]
        if type(size) is not int or not (size==v.size or (kind=='state' and v.size==15 and size==6)):
            raise ValueError('invalid_surface_prior_block_downgrade')
        reference=v.reference[:12] if v.size==15 and size==6 else v.reference
        retained.append(Variable(kind,ident,size,reference));keep.extend(range(start,start+size))
    layout(retained)
    if not len(j):raise ValueError('surface_prior_requires_consumed_information_rows')
    remaining=[i for i in range(count) if i not in set(keep)]
    ordered=j[:,keep+remaining]
    # QR first bounds the full-U elimination by variable count, not history.
    if len(j)>count+1:
        _,compact=np.linalg.qr(np.column_stack((ordered,r)),mode='reduced')
        ordered=compact[:,:count];r=compact[:,count]
    if remaining:
        reduced=eliminate_patch(ordered,r,retained_columns=len(keep));a=reduced['jacobian'];b=reduced['residual']
    else:a=ordered;b=r
    if not len(a):a=np.zeros((1,len(keep)));b=np.zeros(1)
    offset=b-a@delta[keep]
    # Preserve the residual-only constant in one bounded extra row.
    if len(a)>len(keep)+1:
        _,compact=np.linalg.qr(np.column_stack((a,offset)),mode='reduced')
        a=compact[:,:len(keep)];offset=compact[:,len(keep)]
    return SurfacePrior(tuple(retained),a,offset,ledger,epoch,revision)
