"""Bounded, correlated recovery from actual retiring-state conditionals.

Modes are linearization points, not fixed observations. Covariance is conditional
on the supplied live coordinates; it excludes uncertainty in those coordinates.
Shared ancestor noise and unbounded null directions retain their identities.
This read-only archive never installs late factors or changes a live estimate.
"""
from dataclasses import dataclass
import numpy as np

from .inertial import exp_so3
from .surface_prior import Variable,layout


def coordinates(variables):
    return tuple((v.kind,v.ident,i) for v in variables for i in range(v.size))


def merge_reference(previous,current):
    if previous.key!=current.key:raise ValueError('conditional_variable_identity_mismatch')
    small,large=sorted((previous,current),key=lambda v:v.size)
    if small.kind=='state' and small.size==6 and large.size==15:
        if small.reference!=large.reference[:12]:raise ValueError('changed_conditional_owner_reference')
        return large
    if previous!=current:raise ValueError('changed_conditional_owner_reference')
    return previous


@dataclass(frozen=True)
class Conditional:
    ident: int
    variables: tuple
    kept: tuple
    removed: tuple
    regression: np.ndarray
    offset: np.ndarray
    noise: np.ndarray
    unbounded: np.ndarray
    deterministic: object=None

    @classmethod
    def read(cls,value):
        ident=value['id'];variables=tuple(Variable(**v) for v in value['variables'])
        _,_,columns=layout(variables,retirement_source=True)
        keep=tuple(value['kept_indices']);removed=tuple(value['removed_indices'])
        if (type(ident) is not int or ident<1 or not removed
                or any(type(i) is not int for i in keep+removed)
                or sorted(keep+removed)!=list(range(columns))):
            raise ValueError('unique_complete_conditional_partition_required')
        m=np.asarray(value['regression'],dtype=float);offset=np.asarray(value['offset'],dtype=float)
        noise=np.asarray(value['noise_factor'],dtype=float);free=np.asarray(value['unbounded_basis'],dtype=float);n=len(removed)
        exact=np.asarray(value.get('deterministic_basis',np.zeros((n,0))),dtype=float)
        if (m.shape!=(n,len(keep)) or offset.shape!=(n,) or noise.ndim!=2 or free.ndim!=2
                or noise.shape[0]!=n or free.shape[0]!=n or exact.ndim!=2 or exact.shape[0]!=n
                or noise.shape[1]+free.shape[1]+exact.shape[1]!=n
                or any(not x.all() for x in (np.isfinite(m),np.isfinite(offset),np.isfinite(noise),np.isfinite(free),np.isfinite(exact)))):
            raise ValueError('finite_complete_conditional_geometry_required')
        # Normalize only to validate the observable/null subspaces. Noise scales
        # themselves stay exactly as emitted; no variance is guessed or changed.
        norms=np.linalg.norm(noise,axis=0)
        if np.any(norms<=0):raise ValueError('nonzero_conditional_noise_columns_required')
        basis=np.column_stack((noise/norms,free,exact))
        if not np.allclose(basis.T@basis,np.eye(n),atol=2e-7,rtol=0):
            raise ValueError('conditional_observable_and_null_spaces_required')
        for name in ('marginal_information_relative_error','marginal_gradient_relative_error'):
            if name in value and (not np.isfinite(value[name]) or value[name]>1e-7):
                raise ValueError('native_conditional_marginal_mismatch')
        arrays=[]
        for a in (m,offset,noise,free,exact):
            a=a.copy();a.flags.writeable=False;arrays.append(a)
        return cls(ident,variables,keep,removed,*arrays)

    @property
    def source_coordinates(self):return coordinates(self.variables)

    @property
    def matrix_bytes(self):return sum(x.nbytes for x in (self.regression,self.offset,self.noise,self.unbounded,self.deterministic))


class RetiredPoseArchive:
    def __init__(self,epoch,*,max_nodes=256,max_matrix_bytes=32*1024*1024,max_recovery_bytes=32*1024*1024):
        if (not isinstance(epoch,str) or not epoch.strip() or type(max_nodes) is not int or not 1<=max_nodes<=1024
                or any(type(x) is not int or not 1024<=x<=128*1024*1024 for x in (max_matrix_bytes,max_recovery_bytes))):
            raise ValueError('explicit_bounded_conditional_archive_required')
        self.epoch=epoch;self.max_nodes=max_nodes;self.max_matrix_bytes=max_matrix_bytes;self.max_recovery_bytes=max_recovery_bytes
        self.nodes={};self.producers={};self.references={};self.root={};self.root_variables=();self.stamp_ns=0
        self.highest_id=0;self.closed_through_ns=0;self.protected=frozenset();self._memo={}

    @staticmethod
    def _validate(nodes,root,root_variables):
        producers={};references={}
        for node in nodes.values():
            for v in node.variables:
                references[v.key]=merge_reference(references[v.key],v) if v.key in references else v
            cols=node.source_coordinates
            for i,index in enumerate(node.removed):
                key=cols[index]
                if key in producers or key in root:raise ValueError('duplicate_retired_coordinate_owner')
                producers[key]=(node.ident,i)
        for v in root_variables:
            references[v.key]=merge_reference(references[v.key],v) if v.key in references else v
        for node in nodes.values():
            for index in node.kept:
                key=node.source_coordinates[index]
                if key in root:continue
                if key not in producers:raise ValueError('missing_conditional_separator')
                if producers[key][0]<=node.ident:raise ValueError('cyclic_or_reordered_conditional_ownership')
        return producers,references

    def ingest(self,packet):
        if packet['epoch']!=self.epoch:raise ValueError('conditional_archive_epoch_mismatch')
        stamp=packet['stamp_ns'];variables=tuple(Variable(**v) for v in packet['variables']);_,_,count=layout(variables,retirement_source=True)
        values=np.asarray(packet['delta'],dtype=float)
        if type(stamp) is not int or stamp<=self.stamp_ns or values.shape!=(count,) or not np.isfinite(values).all():
            raise ValueError('ordered_finite_live_coordinates_required')
        nodes=dict(self.nodes);highest=self.highest_id
        for value in packet['conditionals']:
            node=Conditional.read(value)
            if node.ident<=highest or node.ident in nodes:raise ValueError('replayed_conditional_node')
            highest=node.ident;nodes[node.ident]=node
        if len(nodes)>self.max_nodes or sum(n.matrix_bytes for n in nodes.values())>self.max_matrix_bytes:
            raise ValueError('conditional_archive_capacity_requires_explicit_expiry')
        root=dict(zip(coordinates(variables),values));producers,references=self._validate(nodes,root,variables)
        # All validation/allocation precedes publication of a new live snapshot.
        self.nodes=nodes;self.producers=producers;self.references=references;self.root=root;self.root_variables=variables
        self.highest_id=highest;self.stamp_ns=stamp;self._memo={}

    def _resolve(self,ident):
        if ident in self._memo:return self._memo[ident]
        # Parent IDs increase strictly. Resolve the ancestor closure iteratively,
        # so the supported node budget cannot exceed Python's recursion limit.
        pending=[ident];needed=set()
        while pending:
            current=pending.pop()
            if current in needed or current in self._memo:continue
            needed.add(current);node=self.nodes[current];columns=node.source_coordinates
            pending.extend(self.producers[columns[i]][0] for i in node.kept if columns[i] not in self.root)
        for current in sorted(needed,reverse=True):self._resolve_one(current)
        return self._memo[ident]

    def _resolve_one(self,ident):
        node=self.nodes[ident];n=len(node.removed);columns=node.source_coordinates
        if self._workspace_bytes()+sum(x.nbytes for x in (node.offset,node.noise,node.unbounded))>self.max_recovery_bytes:
            self._memo={};raise ValueError('conditional_recovery_workspace_capacity')
        mean=node.offset.copy();noise={ident:node.noise.copy()} if node.noise.shape[1] else {}
        free={ident:node.unbounded.copy()} if node.unbounded.shape[1] else {};groups={};root_cols=[];root_values=[]
        for i,index in enumerate(node.kept):
            key=columns[index]
            if key in self.root:root_cols.append(i);root_values.append(self.root[key])
            else:
                parent,row=self.producers[key];groups.setdefault(parent,[]).append((i,row))
        if root_cols:mean+=node.regression[:,root_cols]@root_values
        for parent,items in groups.items():
            resolved=self._memo[parent];local,rows=map(list,zip(*items));m=node.regression[:,local]
            mean+=m@resolved['mean'][rows]
            for target,source in ((noise,resolved['noise']),(free,resolved['free'])):
                for key,coefficient in source.items():
                    if key not in target:
                        if coefficient.shape[1]*n*8+self._workspace_bytes()+sum(x.nbytes for x in (mean,*noise.values(),*free.values()))>self.max_recovery_bytes:
                            self._memo={};raise ValueError('conditional_recovery_workspace_capacity')
                        target[key]=np.zeros((n,coefficient.shape[1]))
                    target[key]+=m@coefficient[rows,:]
        value=dict(mean=mean,noise=noise,free=free);self._memo[ident]=value
        if self._workspace_bytes()>self.max_recovery_bytes:
            self._memo={};raise ValueError('conditional_recovery_workspace_capacity')
        return value

    def _workspace_bytes(self):
        return sum(x.nbytes for r in self._memo.values() for x in (r['mean'],*r['noise'].values(),*r['free'].values()))

    def recover(self,stamps):
        stamps=list(stamps)
        if (not 1<=len(stamps)<=64 or len(set(stamps))!=len(stamps)
                or any(type(t) is not int or t<=0 or (t<=self.closed_through_ns and ('state',t) not in self.protected) for t in stamps)):
            raise ValueError('distinct_admissible_pose_queries_required')
        if (6*len(stamps))**2*8+6*len(stamps)*8>self.max_recovery_bytes:
            raise ValueError('conditional_recovery_workspace_capacity')
        mean=np.zeros(6*len(stamps));noise={};free={}
        for offset,stamp in enumerate(stamps):
            if ('state',stamp) not in self.references:raise ValueError('pose_not_owned_by_conditional_archive')
            for component in range(6):
                key=('state',stamp,component);row=6*offset+component
                if key in self.root:mean[row]=self.root[key];continue
                if key not in self.producers:raise ValueError('pose_coordinate_expired_or_never_retained')
                ident,local=self.producers[key];resolved=self._resolve(ident);mean[row]=resolved['mean'][local]
                for target,source in ((noise,resolved['noise']),(free,resolved['free'])):
                    for ident,coefficients in source.items():
                        if ident not in target:
                            used=self._workspace_bytes()+mean.nbytes+sum(x.nbytes for x in noise.values())+2*sum(x.nbytes for x in free.values())
                            # Reserve both the covariance and the final copied
                            # null-direction matrix before allocating more rows.
                            required=coefficients.shape[1]*len(mean)*8+len(mean)**2*8
                            if used+2*required>self.max_recovery_bytes:
                                self._memo={};raise ValueError('conditional_recovery_workspace_capacity')
                            target[ident]=np.zeros((len(mean),coefficients.shape[1]))
                        target[ident][row]=coefficients[local]
        used=self._workspace_bytes()+mean.nbytes+sum(x.nbytes for x in noise.values())+2*sum(x.nbytes for x in free.values())+len(mean)**2*8
        if used>self.max_recovery_bytes:
            self._memo={};raise ValueError('conditional_recovery_workspace_capacity')
        covariance=np.zeros((len(mean),len(mean)))
        for coefficient in noise.values():covariance+=coefficient@coefficient.T
        unbounded=np.column_stack(list(free.values())) if free else np.zeros((len(mean),0))
        poses=[]
        for i,stamp in enumerate(stamps):
            reference=np.array(self.references[('state',stamp)].reference);delta=mean[6*i:6*i+6]
            poses.append(dict(stamp_ns=stamp,position=reference[:3]+delta[:3],
                attitude=exp_so3(delta[3:])@reference[3:12].reshape(3,3)))
        return dict(poses=poses,deltas=mean,observable_conditional_covariance=covariance,unbounded_basis=unbounded,
            live_coordinates_treated_as_known=True,independent_pose_observations=False,map_qualified=False,
            source_epoch=self.epoch,source_stamp_ns=self.stamp_ns)

    def expire_through(self,stamp_ns,*,protect=()):
        if type(stamp_ns) is not int or not self.closed_through_ns<=stamp_ns<=self.stamp_ns:
            raise ValueError('monotonic_past_conditional_cutoff_required')
        protect=frozenset(tuple(key) for key in protect)
        if any(len(key)!=2 or key[0]!='state' or type(key[1]) is not int or key[1]<=0 for key in protect):
            raise ValueError('explicit_original_pose_protection_required')
        needed=set()
        for ident,node in self.nodes.items():
            if any((key[0]=='state' and key[1]>stamp_ns) or (key[0],key[1]) in protect
                   for key in (node.source_coordinates[i] for i in node.removed)):needed.add(ident)
        pending=list(needed)
        while pending:
            node=self.nodes[pending.pop()]
            for index in node.kept:
                key=node.source_coordinates[index]
                if key in self.root:continue
                parent=self.producers[key][0]
                if parent not in needed:needed.add(parent);pending.append(parent)
        nodes={i:n for i,n in self.nodes.items() if i in needed};producers,references=self._validate(nodes,self.root,self.root_variables)
        self.nodes=nodes;self.producers=producers;self.references=references;self.closed_through_ns=stamp_ns;self.protected=protect;self._memo={}
