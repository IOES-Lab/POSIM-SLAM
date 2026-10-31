"""Joint local uncertainty from the native root and its correlated archive.

All active native IMU/DVL/pressure rows and the single owned marginal contribute
to the root information. Frozen robust weights and declared sensor covariances
remain approximations. An observable covariance is accompanied by explicit free
directions; a pseudoinverse alone must never be used as map admission evidence.
"""
from dataclasses import dataclass
import numpy as np

from .retired_pose_archive import coordinates,merge_reference
from .surface_prior import MAX_COLUMNS,layout
from .constrained_information import constraint_support


@dataclass(frozen=True)
class NativeSurfaceInformation:
    jacobian: np.ndarray
    residual: np.ndarray
    counts: tuple
    constraint_jacobian: object=None
    constraint_residual: object=None

    @classmethod
    def read(cls,value,columns):
        if (not isinstance(value,dict) or value.get('schema') not in ('posim-slam-native-surface-information-1','posim-slam-native-surface-information-2')
                or value.get('scope')!='local_frozen_weight_gauss_newton'):
            raise ValueError('native_joint_information_schema_required')
        if not 1<=columns<=MAX_COLUMNS:raise ValueError('native_joint_information_column_budget')
        j=np.asarray(value['jacobian'],dtype=float);r=np.asarray(value['residual'],dtype=float)
        counts=tuple(value[k] for k in ('imu_intervals','dvl_observations','pressure_observations'))
        if (j.ndim!=2 or j.shape[1]!=columns or not 1<=len(j)<=columns+1 or r.shape!=(len(j),)
                or not np.isfinite(j).all() or not np.isfinite(r).all()
                or any(type(n) is not int or not 0<=n<=73 for n in counts)):
            raise ValueError('finite_bounded_native_joint_information_required')
        j=j.copy();r=r.copy();j.flags.writeable=False;r.flags.writeable=False
        if ('constraint_jacobian' in value)!=('constraint_residual' in value):raise ValueError('complete_native_exact_support_required')
        if (('constraint_jacobian' in value)!=(value['schema']=='posim-slam-native-surface-information-2')):
            raise ValueError('native_exact_support_schema_required')
        c=np.asarray(value.get('constraint_jacobian',np.zeros((0,columns))),dtype=float)
        d=np.asarray(value.get('constraint_residual',np.zeros(0)),dtype=float)
        constraint_support(c,-d,columns)
        c=c.copy();d=d.copy();c.flags.writeable=False;d.flags.writeable=False
        return cls(j,r,counts,c,d)


def joint_surface_marginal(snapshot,order,*,relative_tolerance=1e-10):
    """Recover arbitrary owned blocks with root and shared conditional noise.

    The returned deltas are the owner's nominal coordinates. linearized_mode
    includes the undamped linear solve's shift and does not replace that owner.
    Free roots and shared conditional free directions stay explicit. No factors
    are installed, no references are changed, and no map is qualified here.
    """
    information=snapshot._information
    if information is None:raise ValueError('complete_native_joint_information_required')
    if (isinstance(relative_tolerance,bool) or not np.isfinite(relative_tolerance)
            or not 0<relative_tolerance<=1e-6):raise ValueError('explicit_joint_rank_tolerance_required')
    order,_,count=layout(order,retirement_source=True);archive=snapshot._archive
    query=coordinates(order);root_keys=coordinates(snapshot.order);root_columns={k:i for i,k in enumerate(root_keys)}
    for variable in order:
        key=variable.key
        if key not in archive.references:raise ValueError('joint_query_variable_not_owned')
        merged=merge_reference(archive.references[key],variable)
        if variable.size>archive.references[key].size or merged!=archive.references[key]:
            raise ValueError('joint_query_reference_not_owned')
        if variable.kind=='state' and variable.ident<=archive.closed_through_ns and key not in archive.protected:
            raise ValueError('joint_query_expired')
    needed=set();pending=[]
    for key in query:
        if key not in root_columns:
            if key not in archive.producers:raise ValueError('joint_query_coordinate_not_owned')
            pending.append(archive.producers[key][0])
    while pending:
        ident=pending.pop()
        if ident in needed:continue
        needed.add(ident);node=archive.nodes[ident];cols=node.source_coordinates
        pending.extend(archive.producers[cols[i]][0] for i in node.kept if cols[i] not in root_columns)
    # Bound the entire linear recovery workspace before SVD/propagation. Reserve
    # root factors, all dependency rows, final factors/covariance and SVD copies.
    n=len(root_keys)
    noise_columns=n+sum(archive.nodes[i].noise.shape[1]+archive.nodes[i].unbounded.shape[1] for i in needed)
    dependency_rows=sum(len(archive.nodes[i].removed) for i in needed)
    max_separator=max((len(archive.nodes[i].kept) for i in needed),default=0)
    bound=8*((dependency_rows+3*count+4*max_separator)*noise_columns+count*count+12*n*n+4*n)
    if bound>archive.max_recovery_bytes:raise ValueError('joint_recovery_workspace_capacity')
    point,z,hard=constraint_support(information.constraint_jacobian,-information.constraint_residual,n)
    u,s,vt=np.linalg.svd(information.jacobian@z,full_matrices=True)
    cutoff=relative_tolerance*(s[0] if len(s) else 0.)
    rank=int(np.count_nonzero(s>cutoff))
    root_noise=z@(vt[:rank].T/s[:rank]) if rank else np.zeros((n,0))
    root_free=z@vt[rank:].T
    root_shift=point-root_noise@(u[:,:rank].T@(information.residual+information.jacobian@point))
    root_delta=np.array([archive.root[k] for k in root_keys]);resolved={}

    def collect(keys):
        mean=np.empty(len(keys));shift=np.zeros(len(keys));noise={};free={}
        for row,key in enumerate(keys):
            if key in root_columns:
                column=root_columns[key];mean[row]=root_delta[column];shift[row]=root_shift[column]
                sources=((noise,{0:root_noise[column:column+1]}),(free,{0:root_free[column:column+1]}))
                local=0
            else:
                ident,local=archive.producers[key];value=resolved[ident]
                mean[row]=value['mean'][local];shift[row]=value['shift'][local]
                sources=((noise,value['noise']),(free,value['free']))
            for target,source in sources:
                for ident,coefficient in source.items():
                    if not coefficient.shape[1]:continue
                    if ident not in target:target[ident]=np.zeros((len(keys),coefficient.shape[1]))
                    target[ident][row]=coefficient[local]
        return dict(mean=mean,shift=shift,noise=noise,free=free)

    for ident in sorted(needed,reverse=True):
        node=archive.nodes[ident];cols=node.source_coordinates
        parents=collect([cols[i] for i in node.kept])
        value=dict(mean=node.offset+node.regression@parents['mean'],shift=node.regression@parents['shift'],
            noise={i:node.regression@c for i,c in parents['noise'].items()},
            free={i:node.regression@c for i,c in parents['free'].items()})
        if node.noise.shape[1]:value['noise'][ident]=node.noise
        if node.unbounded.shape[1]:value['free'][ident]=node.unbounded
        resolved[ident]=value
    value=collect(query);covariance=np.zeros((count,count))
    for coefficient in value['noise'].values():covariance+=coefficient@coefficient.T
    unbounded=np.column_stack(list(value['free'].values())) if value['free'] else np.zeros((count,0))
    return dict(order=order,deltas=value['mean'],linearized_mode=value['mean']+value['shift'],
        observable_covariance=covariance,unbounded_basis=unbounded,root_rank=rank+hard.shape[1],root_columns=n,
        root_constraint_rank=hard.shape[1],
        root_singular_values=s,relative_rank_tolerance=relative_tolerance,workspace_bound_bytes=bound,
        live_coordinates_treated_as_known=False,independent_pose_observations=False,map_qualified=False,
        information_scope='local_frozen_weight_gauss_newton',
        active_native_counts=dict(zip(('imu_intervals','dvl_observations','pressure_observations'),information.counts)),
        source_epoch=snapshot.epoch,source_stamp_ns=snapshot.stamp_ns,
        owner_revision=snapshot.owner_revision,prior_revision=snapshot.prior_revision)
