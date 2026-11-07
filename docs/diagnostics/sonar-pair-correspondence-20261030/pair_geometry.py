"""Bounded fixed-pose point fits for pair-constraint diagnostics.

Nine elevation seeds and declared range/bearing noise retain competing modes.
No point prior, pose prior, terrain, truth, residual trimming or map admission.
All coordinates are in the first sonar frame; the second pose maps B to A.
"""
import numpy as np

SIGMA=np.array([.025083611719310283,.008213314126139592])

def evaluate(points, measurements, rotation, translation, geometry):
    points=np.asarray(points,float)
    q=np.stack([points,(points-translation)@rotation],axis=-2)
    radius=np.linalg.norm(q,axis=-1)
    horizontal=np.hypot(q[...,0],q[...,1])
    bearing=np.arctan2(q[...,1],q[...,0])
    elevation=np.arctan2(q[...,2],horizontal)
    valid=((q[...,0]>0)&(radius>=geometry['range_min'])&(radius<=geometry['range_max'])
        &(abs(bearing)<=geometry['horizontal_fov']/2+5e-7)&(abs(elevation)<=geometry['vertical_fov']/2+5e-7)).all(axis=-1)
    safe_r=np.maximum(radius,1e-12)
    h2=np.maximum(horizontal**2,1e-12)
    projection=np.stack([q/safe_r[...,None],np.stack([-q[...,1]/h2,q[...,0]/h2,np.zeros_like(h2)],axis=-1)],axis=-2)
    prediction=np.stack([radius,bearing],axis=-1)
    residual=prediction-measurements[:,None,:,:]
    residual[...,1]=np.arctan2(np.sin(residual[...,1]),np.cos(residual[...,1]))
    residual=residual/SIGMA
    jacobian=projection.copy()
    jacobian[...,1,:,:]=projection[...,1,:,:]@rotation.T
    jacobian=jacobian/SIGMA[None,None,None,:,None]
    residual=residual.reshape(*points.shape[:-1],4)
    jacobian=jacobian.reshape(*points.shape[:-1],4,3)
    cost=np.sum(residual**2,axis=-1)
    cost=np.where(valid,cost,np.inf)
    return residual,jacobian,cost,valid

def margins(points,rotation,translation,geometry):
    q=np.stack([points,(points-translation)@rotation],axis=-2)
    radius=np.maximum(np.linalg.norm(q,axis=-1),1e-12)
    horizontal=np.maximum(np.hypot(q[...,0],q[...,1]),1e-12)
    bearing=np.arctan2(q[...,1],q[...,0])
    elevation=np.arctan2(q[...,2],horizontal)
    az_j=np.stack([-q[...,1],q[...,0],np.zeros_like(radius)],axis=-1)/horizontal[...,None]**2
    el_j=np.stack([-q[...,2]*q[...,0]/horizontal,-q[...,2]*q[...,1]/horizontal,horizontal],axis=-1)/radius[...,None]**2
    radial=q/radius[...,None]
    forward=np.broadcast_to([1.,0.,0.],q.shape)
    gradient=np.stack([forward,-az_j,az_j,-el_j,el_j,radial,-radial],axis=-2)
    gradient[...,1,:,:]=gradient[...,1,:,:]@rotation.T
    bounds=np.stack([q[...,0],geometry['horizontal_fov']/2-bearing,geometry['horizontal_fov']/2+bearing,
        geometry['vertical_fov']/2-elevation,geometry['vertical_fov']/2+elevation,radius-geometry['range_min'],geometry['range_max']-radius],axis=-1)
    return bounds.reshape(*points.shape[:-1],14),gradient.reshape(*points.shape[:-1],14,3)

def fit_pair(first,second,rotation,translation,*,geometry,iterations=30):
    from posim_slam.constrained_step import feasible_step
    first,second=np.asarray(first,float).reshape(-1,2),np.asarray(second,float).reshape(-1,2)
    rotation,translation=np.asarray(rotation,float),np.asarray(translation,float)
    if (first.shape!=second.shape or len(first)>300 or not np.isfinite(first).all() or not np.isfinite(second).all()
            or rotation.shape!=(3,3) or translation.shape!=(3,) or not np.isfinite(rotation).all() or not np.isfinite(translation).all()
            or not np.allclose(rotation.T@rotation,np.eye(3),atol=1e-8,rtol=0) or np.linalg.det(rotation)<0):
        raise ValueError('bounded_finite_pair_required')
    if (not 0<geometry['range_min']<geometry['range_max'] or not 0<geometry['horizontal_fov']<np.pi
            or not 0<geometry['vertical_fov']<np.pi):raise ValueError('explicit_sensor_geometry_required')
    measurements=np.stack([first,second],axis=1)
    elevation=np.linspace(-geometry['vertical_fov']/2,geometry['vertical_fov']/2,9)
    rays=np.stack([np.cos(first[:,1,None])*np.cos(elevation),np.sin(first[:,1,None])*np.cos(elevation),np.broadcast_to(np.sin(elevation),(len(first),9))],axis=-1)
    points=first[:,0,None,None]*rays
    current=evaluate(points,measurements,rotation,translation,geometry)
    converged=np.zeros(points.shape[:-1],bool)
    constrained=np.zeros_like(converged)
    for _ in range(iterations):
        residual,jacobian,cost,valid=current
        active=valid&~converged
        if not active.any():break
        u,s,v=np.linalg.svd(jacobian[active],full_matrices=False)
        inverse=np.zeros_like(s)
        np.divide(1,s,out=inverse,where=s>np.maximum(s[:,:1],1.)*4*np.finfo(float).eps)
        step=np.zeros_like(points)
        step[active]=-np.einsum('nij,nj->ni',v.transpose(0,2,1),inverse*np.einsum('nij,ni->nj',u,residual[active]))
        gradient=np.max(abs(np.einsum('...ij,...i->...j',jacobian,residual)),axis=-1)
        converged|=active&(np.linalg.norm(step,axis=-1)<1e-8)&(gradient<1e-6)
        g,a=margins(points,rotation,translation,geometry)
        crosses=active&~converged&np.any(g+np.einsum('...ij,...j->...i',a,step)<0,axis=-1)
        for feature,seed in np.argwhere(crosses):
            result=feasible_step(jacobian[feature,seed],residual[feature,seed],np.maximum(g[feature,seed],0.)*.99,a[feature,seed])
            step[feature,seed]=result['step']
            if result['converged'] and np.linalg.norm(result['step'])<1e-7:
                converged[feature,seed]=True
                constrained[feature,seed]=bool(result['active_constraints'])
        pending=active&~converged
        next_points=points.copy()
        for fraction in (1.,.5,.25,.125,.0625,.03125):
            candidate=points+fraction*step
            trial=evaluate(candidate,measurements,rotation,translation,geometry)
            improved=pending&trial[3]&(trial[2]<cost)
            next_points[improved]=candidate[improved]
            pending[improved]=False
        points=next_points
        current=evaluate(points,measurements,rotation,translation,geometry)
    residual,jacobian,cost,valid=current
    gradient=np.max(abs(np.einsum('...ij,...i->...j',jacobian,residual)),axis=-1)
    singular=np.linalg.svd(jacobian,compute_uv=False)
    rank=np.sum(singular>np.maximum(singular[...,:1],1.)*4*np.finfo(float).eps,axis=-1)
    return dict(points=points,residual=residual,cost=cost,valid=valid,converged=converged|(valid&(gradient<1e-6)),
        constrained_stationary=constrained,gradient_inf=gradient,rank=rank,singular=singular,measurement_covariance=np.diag(np.tile(SIGMA**2,2)))
