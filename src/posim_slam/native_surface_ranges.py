"""Original finite-cone/fan rows in an explicit native owner snapshot.

Plane seeds initialize unknown variables, never independent plane priors.
Nominal support is rebuilt from original cone ranges and current conditional
poses. The caller owns physical association and the full noise model.
"""
from dataclasses import dataclass

import numpy as np

from .native_surface import NativeSurfaceSnapshot,prepare_native_ranges
from .inertial import exp_so3
from .range_surface_joint import linearize_surface_ranges,normal_basis
from .retired_pose_archive import coordinates
from .surface_prior import Variable


@dataclass(frozen=True)
class NativeSurfaceRangeBlock:
    increment: object
    diagnostics: dict


@dataclass(frozen=True)
class NativeSurfaceRangeLinearization:
    order: tuple
    delta: np.ndarray
    jacobian: np.ndarray
    residual: np.ndarray
    original_keys: tuple
    diagnostics: dict


def _plane_information(jacobian,noise):
    rows=np.linalg.solve(np.linalg.cholesky(noise),jacobian) if len(jacobian) else np.zeros((0,3))
    _,s,v=np.linalg.svd(rows,full_matrices=True)
    tolerance=np.finfo(float).eps*max(rows.shape)*float(s[0]) if len(s) else 0.
    rank=int(np.sum(s>tolerance));free=v[rank:].T
    factor=v[:rank].T/s[:rank] if rank else np.zeros((3,0))
    return dict(rank=rank,singular_values=s,unbounded_basis=free,noise_factor=factor)


def conditional_surface_observability(block,observations):
    """Selected range rows with poses fixed; never a joint state covariance.

    block is the matching linearize_surface_ranges result. Support-only seeds
    add no likelihood. Rank-deficient normal/distance modes remain unbounded;
    a fan prediction is finite only if its gradient annihilates every such mode.
    Owner-prior information and joint pose uncertainty are excluded explicitly.
    """
    result={};start=6*len(block['state_stamps_ns']);indices=block['factor_indices']
    for index,ident in enumerate(block['plane_ids']):
        columns=slice(start+3*index,start+3*index+3)
        selected=[i for i,k in enumerate(indices) if observations[k]['plane_id']==ident]
        cones=[i for i in selected if observations[indices[i]]['model']=='cone']
        j=block['unwhitened_jacobian'];cov=block['measurement_covariance']
        cone_info=_plane_information(j[cones,columns],cov[np.ix_(cones,cones)])
        all_info=_plane_information(j[selected,columns],cov[np.ix_(selected,selected)])
        predictions=[]
        for i in selected:
            o=observations[indices[i]]
            if o['model']!='fan':continue
            gradient=j[i,columns];free=gradient@cone_info['unbounded_basis']
            tolerance=100*np.finfo(float).eps*max(1.,float(np.linalg.norm(gradient)))
            unbounded=bool(np.linalg.norm(free)>tolerance)
            predictions.append(dict(key=block['factor_keys'][i],unbounded_from_cone_rows=unbounded,
                conditional_cone_only_stddev_m=None if unbounded else float(np.linalg.norm(gradient@cone_info['noise_factor']))))
        result[ident]=dict(cone_rows=len(cones),all_rows=len(selected),cone_information=cone_info,
            all_range_information=all_info,fan_predictions=predictions,
            scope='Selected rows with poses fixed; support-only seeds add no information; excludes owner prior and joint pose uncertainty')
    return result


def linearize_native_surface_ranges(snapshot,*,planes,observations,
                                    measurement_covariance,uncertainty_source,
                                    require_support=True):
    """Read-only original range rows at exact native-owned coordinates.

    New planes supply initial_normal/initial_distance; existing planes supply
    only support_keys/max_plane_deviation/max_support_distance. Their mode and
    fixed references come from the native snapshot. Observations and covariance
    follow range_surface_joint.linearize_surface_ranges, including support-only
    cone rows. Only actual factor keys enter the original-reading ledger.

    This function neither chooses an echo/plane identity nor infers unknown
    variance from sensor resolution. No pose, normal or fitted-plane covariance
    is inserted as an extra observation. Every diagnostic remains unqualified.
    """
    if not isinstance(snapshot,NativeSurfaceSnapshot):raise ValueError('native_owned_surface_snapshot_required')
    if (not isinstance(planes,dict) or not 1<=len(planes)<=8
            or not isinstance(observations,(list,tuple)) or not 1<=len(observations)<=512):
        raise ValueError('bounded_native_surface_ranges_required')
    stamps=sorted({o['stamp_ns'] for o in observations})
    if not 1<=len(stamps)<=16 or any(type(t) is not int or not 0<t<=snapshot.stamp_ns for t in stamps):
        raise ValueError('exact_owned_surface_measurement_stamps_required')
    recovered=snapshot.recover(stamps)['deltas']
    states={};order=[];deltas=[]
    for index,stamp in enumerate(stamps):
        reference=snapshot.reference('state',stamp)
        pose=Variable('state',stamp,6,reference.reference[:12]);delta=recovered[6*index:6*index+6]
        order.append(pose);deltas.extend(delta)
        states[stamp]=dict(position=np.asarray(pose.reference[:3])+delta[:3],
            rotation_reference=np.asarray(pose.reference[3:12]).reshape(3,3),rotation_delta=delta[3:6])
    owned={v.ident:v for v in snapshot.order if v.kind=='plane'}
    roots=snapshot._archive.root;parameters={}
    guard_fields={'support_keys','max_plane_deviation','max_support_distance'}
    for ident in sorted(planes):
        if type(ident) is not int or not 0<=ident<2**31:raise ValueError('native_surface_plane_id_required')
        data=planes[ident]
        if not isinstance(data,dict):raise ValueError('native_surface_plane_hypothesis_required')
        if ident in owned:
            if set(data)-guard_fields:raise ValueError('native_surface_plane_coordinates_are_owned')
            variable=owned[ident]
            delta=np.asarray([roots[key] for key in coordinates((variable,))])
        else:
            if set(data)-guard_fields-{'initial_normal','initial_distance'}:
                raise ValueError('unknown_native_surface_plane_fields')
            normal=np.asarray(data['initial_normal'],dtype=float);distance=data['initial_distance']
            if normal.shape!=(3,) or not np.isfinite(normal).all():raise ValueError('native_surface_plane_seed_required')
            variable=Variable('plane',ident,3,tuple(normal)+(distance,));delta=np.zeros(3)
        order.append(variable);deltas.extend(delta)
        parameters[ident]=dict(normal_reference=np.asarray(variable.reference[:3]),normal_delta=delta[:2],
            distance=variable.reference[3]+delta[2],**{key:data[key] for key in guard_fields if key in data})
    block=linearize_surface_ranges(states=states,planes=parameters,observations=observations,
        measurement_covariance=measurement_covariance,uncertainty_source=uncertainty_source,require_support=require_support)
    diagnostics={key:block[key] for key in ('source_keys','factor_keys','support_only_indices','factor_indices',
        'predicted_ranges','conditional_hits','support_seeds','all_support_checked','uncertainty_source')}
    diagnostics.update(unwhitened_residual=block['unwhitened_residual'],physical_identity_verified=False,
        conditional_plane_information=conditional_surface_observability(block,observations),
        map_qualified=False,poses_are_independent_observations=False,
        scope='Original jointly whitened range rows in native coordinates; association/noise remain caller hypotheses')
    return NativeSurfaceRangeLinearization(tuple(order),np.asarray(deltas),block['jacobian'],block['residual'],
        tuple(block['factor_keys']),diagnostics)


def prepare_native_surface_ranges(snapshot,*,planes,observations,measurement_covariance,
                                  uncertainty_source,require_support=True,max_workspace_bytes=32*1024*1024):
    """Prepare only new likelihood rows; ledger admission is checked here.

    Use linearize_native_surface_ranges for read-only nonlinear evaluation of
    already consumed measurements. Conditional rank is diagnostic, not a
    correspondence decision or a substitute for full joint uncertainty.
    """
    options=dict(observations=observations,measurement_covariance=measurement_covariance,uncertainty_source=uncertainty_source)
    # A new surface is initialized against the ACTUAL factor rows. Nominal
    # support-only seeds cannot supply missing probability information. This
    # solve sets an initial reference only; its Hessian/covariance/damping never
    # becomes a prior or an extra native observation.
    initial=linearize_native_surface_ranges(snapshot,planes=planes,require_support=require_support,**options)
    owned={v.ident for v in snapshot.order if v.kind=='plane'}
    new=sorted(set(planes)-owned);candidate={ident:dict(p) for ident,p in planes.items()}
    initialization=dict(new_planes=new,initial_cost=float(initial.residual@initial.residual)/2,
        final_cost=float(initial.residual@initial.residual)/2,iterations=0,added_prior=False)
    if new:
        plane_ids=sorted(planes);start=6*sum(v.kind=='state' for v in initial.order)
        columns=[start+3*plane_ids.index(ident)+axis for ident in new for axis in range(3)]
        block=initial;damping=1e-3;converged=False
        for iteration in range(50):
            j=block.jacobian[:,columns];gradient=j.T@block.residual
            if np.linalg.norm(gradient,np.inf)<1e-9:converged=True;break
            h=j.T@j;scale=max(float(np.trace(h))/len(columns),1.)
            step=np.linalg.solve(h+damping*scale*np.eye(len(columns)),-gradient)
            trial={ident:dict(p) for ident,p in candidate.items()}
            for index,ident in enumerate(new):
                normal=np.asarray(candidate[ident]['initial_normal'])
                trial[ident]['initial_normal']=exp_so3(normal_basis(normal)@step[3*index:3*index+2])@normal
                trial[ident]['initial_distance']=candidate[ident]['initial_distance']+step[3*index+2]
            try:proposed=linearize_native_surface_ranges(snapshot,planes=trial,require_support=False,**options)
            except ValueError:proposed=None
            old_cost=float(block.residual@block.residual)
            cost=float(proposed.residual@proposed.residual) if proposed is not None else float('inf')
            initialization['iterations']=iteration+1
            if cost<old_cost:
                candidate=trial;block=proposed;damping=max(damping/3,1e-12)
                if np.linalg.norm(step)<1e-9 or old_cost-cost<1e-12*max(1.,old_cost):converged=True;break
            else:damping=min(damping*10,1e15)
        if not converged:raise ValueError('native_surface_nonlinear_seed_not_converged')
        initialization['final_cost']=float(block.residual@block.residual)/2
    # Validate every original support/cone/fan again at the solved reference.
    # An inconsistent surface candidate is rejected as a whole. No original
    # reading is removed and no native owner/ledger has been mutated here.
    block=linearize_native_surface_ranges(snapshot,planes=candidate,require_support=require_support,**options)
    block.diagnostics['initialization']=initialization
    increment=prepare_native_ranges(snapshot,order=block.order,delta=block.delta,jacobian=block.jacobian,
        residual=block.residual,range_keys=block.original_keys,max_workspace_bytes=max_workspace_bytes)
    return NativeSurfaceRangeBlock(increment,block.diagnostics)
