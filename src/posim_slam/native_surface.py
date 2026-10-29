"""Snapshot-bound delayed likelihoods for the native marginal owner.

Only new live rows and the matching conditional replacements cross back to the
owner. Active native measurements and its marginal prior are never returned as
an extra measurement. A fresh native snapshot is required after state changes.
"""
from dataclasses import dataclass
import json
import os
from pathlib import Path
import stat

from .delayed_surface_update import _prepare_range_likelihood
from .retired_pose_archive import RetiredPoseArchive
from .surface_prior import RangeLedger,Variable,layout

MAX_SNAPSHOT_BYTES=128*1024*1024


def _integer(value,minimum=0):
    if type(value) is not int or not minimum<=value<2**63:raise ValueError('bounded_native_surface_integer_required')
    return value


def _variable(v):
    return dict(kind=v.kind,ident=v.ident,size=v.size,reference=list(v.reference))


@dataclass(frozen=True)
class NativeSurfaceSnapshot:
    snapshot_id: int
    owner_revision: int
    prior_revision: int
    _archive: RetiredPoseArchive
    ledger: RangeLedger

    @property
    def epoch(self):return self._archive.epoch

    @property
    def stamp_ns(self):return self._archive.stamp_ns

    @property
    def order(self):return self._archive.root_variables

    def reference(self,kind,ident):return self._archive.references[(kind,ident)]

    def recover(self,stamps):return self._archive.recover(stamps)

    @classmethod
    def read(cls,packet):
        if not isinstance(packet,dict) or packet.get('schema')!='posim-slam-native-surface-snapshot-1':
            raise ValueError('native_surface_snapshot_schema_required')
        identity=_integer(packet['snapshot_id'],1);owner=_integer(packet['owner_revision']);prior=_integer(packet['prior_revision'])
        stamp=_integer(packet['stamp_ns'],1);order=tuple(Variable(**v) for v in packet['variables']);layout(order)
        if sum(v.kind=='pressure' for v in order)!=1:raise ValueError('native_surface_pressure_coordinate_required')
        epoch=packet['epoch']
        if not isinstance(epoch,str) or not 1<=len(epoch)<=128:raise ValueError('native_surface_epoch_required')
        archive=RetiredPoseArchive(epoch)
        archive.ingest(dict(epoch=epoch,stamp_ns=stamp,variables=packet['variables'],delta=packet['delta'],conditionals=packet['conditionals']))
        cutoff=_integer(packet['archive_closed_through_ns']);protected=tuple(_integer(t,1) for t in packet['protected_states'])
        if len(set(protected))!=len(protected) or any(('state',t) not in archive.references for t in protected):
            raise ValueError('native_surface_protected_pose_not_owned')
        archive.expire_through(cutoff,protect=[('state',t) for t in protected])
        data=packet['range_ledger'];ledger=RangeLedger(data['recent'],data['closed_through_ns'])
        if ledger.closed_through_ns>stamp or any(k[0]>stamp for k in ledger.recent):
            raise ValueError('native_surface_ledger_clock_mismatch')
        return cls(identity,owner,prior,archive,ledger)

    @classmethod
    def read_file(cls,metadata,directory):
        if not isinstance(metadata,dict) or metadata.get('schema')!='posim-slam-native-surface-file-1':
            raise ValueError('native_surface_file_metadata_required')
        identity=_integer(metadata['snapshot_id'],1);size=_integer(metadata['bytes'],1);_integer(metadata['stamp_ns'],1)
        if size>MAX_SNAPSHOT_BYTES:raise ValueError('native_surface_file_capacity')
        root=Path(directory).resolve();path=Path(metadata['path'])
        if (not path.is_absolute() or path.name!='worker-surface-snapshot-'+str(identity)+'.json'
                or path.parent.resolve()!=root or path.is_symlink()):
            raise ValueError('native_surface_file_path')
        with os.fdopen(os.open(path,os.O_RDONLY|getattr(os,'O_NOFOLLOW',0)),'rb') as stream:
            before=os.fstat(stream.fileno())
            if not stat.S_ISREG(before.st_mode) or before.st_size!=size:raise ValueError('native_surface_file_identity')
            data=stream.read(size+1);after=os.fstat(stream.fileno())
        if len(data)!=size or (before.st_size,before.st_mtime_ns,before.st_ino)!=(after.st_size,after.st_mtime_ns,after.st_ino):
            raise ValueError('native_surface_file_changed')
        result=cls.read(json.loads(data))
        if result.snapshot_id!=identity or result.stamp_ns!=metadata['stamp_ns'] or result.epoch!=metadata['epoch']:
            raise ValueError('native_surface_file_snapshot_mismatch')
        return result


@dataclass(frozen=True)
class NativeSurfaceIncrement:
    snapshot: NativeSurfaceSnapshot
    source_order: tuple
    original_keys: tuple
    _likelihood: object

    @property
    def new_live_rows(self):return self._likelihood.new_live_rows

    @property
    def peak_front_columns(self):return self._likelihood.peak_front_columns

    @property
    def workspace_bound_bytes(self):return self._likelihood.workspace_bound_bytes

    def as_packet(self):
        s=self.snapshot;update=self._likelihood
        nodes=[]
        for ident in update.changed_conditionals:
            n=update.archive.nodes[ident]
            nodes.append(dict(id=n.ident,variables=[_variable(v) for v in n.variables],kept_indices=list(n.kept),removed_indices=list(n.removed),
                regression=n.regression.tolist(),offset=n.offset.tolist(),noise_factor=n.noise.tolist(),unbounded_basis=n.unbounded.tolist()))
        return dict(schema='posim-slam-native-surface-increment-1',epoch=s.epoch,snapshot_id=s.snapshot_id,
            owner_revision=s.owner_revision,prior_revision=s.prior_revision,stamp_ns=s.stamp_ns,
            source_order=[_variable(v) for v in self.source_order],
            new_planes=[_variable(v) for v in update.new_live_rows.order[len(s.order):]],
            jacobian=update.new_live_rows.jacobian.tolist(),offset=update.new_live_rows.offset.tolist(),
            conditionals=nodes,range_keys=[list(k) for k in self.original_keys])


def prepare_native_ranges(snapshot,*,order,delta,jacobian,residual,range_keys,max_workspace_bytes=32*1024*1024):
    """Prepare a pure increment from one explicit native snapshot.

    Caller supplies jointly whitened original range rows in fixed owner
    coordinates, excluding old prior/conditional rows, fitted-pose priors and
    numerical damping. Snapshot IDs/coordinates are rechecked by the native
    owner; constructing an update never mutates an estimator or the snapshot.
    """
    if not isinstance(snapshot,NativeSurfaceSnapshot):raise ValueError('native_owned_surface_snapshot_required')
    order=tuple(order);keys=tuple(tuple(k) for k in range_keys)
    result=_prepare_range_likelihood(snapshot._archive,snapshot.ledger,order=order,delta=delta,
        jacobian=jacobian,residual=residual,range_keys=keys,source_stamp_ns=snapshot.stamp_ns,max_workspace_bytes=max_workspace_bytes)
    return NativeSurfaceIncrement(snapshot,order,tuple(sorted(keys)),result)
