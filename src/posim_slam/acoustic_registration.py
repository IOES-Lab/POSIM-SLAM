"""Experimental planar acoustic registration; unresolved elevation is not XYZ.

Input coordinates represent slant range and bearing in the sonar fan. Callers
must separately qualify the planar assumption before using a result as motion.
"""
import numpy as np


def register(previous,current):
    target=np.asarray(previous,dtype=float);source=np.asarray(current,dtype=float)
    for points in (target,source):
        if points.ndim!=2 or points.shape[1]!=2 or not 20<=len(points)<=1024 or not np.isfinite(points).all():
            raise ValueError('invalid_acoustic_points')
        eigen=np.linalg.eigvalsh(np.cov(points.T))
        if eigen[-1]<=1e-8 or eigen[0]/eigen[-1]<.001:raise ValueError('degenerate_acoustic_geometry')
    r=np.eye(2);t=np.zeros(2)
    def matches(transformed):
        distance=np.sum((transformed[:,None,:]-target[None,:,:])**2,axis=2)
        nearest=np.argmin(distance,axis=1);back=np.argmin(distance,axis=0)
        ids=np.flatnonzero((back[nearest]==np.arange(len(source))) & (distance[np.arange(len(source)),nearest]<.3**2))
        if len(ids)<20:raise ValueError('insufficient_acoustic_overlap')
        return ids,nearest[ids]
    for iteration in range(30):
        moved=source@r.T+t;ids,other=matches(moved)
        a=moved[ids];b=target[other]
        residual=np.linalg.norm(a-b,axis=1)
        keep=residual<=np.quantile(residual,.8)
        if keep.sum()<20:raise ValueError('insufficient_acoustic_inliers')
        a=a[keep];b=b[keep];ca=a.mean(axis=0);cb=b.mean(axis=0)
        u,_,vt=np.linalg.svd((a-ca).T@(b-cb))
        step=vt.T@u.T
        if np.linalg.det(step)<0:vt[-1]*=-1;step=vt.T@u.T
        offset=cb-step@ca;r=step@r;t=step@t+offset
        if np.linalg.norm(offset)<1e-5 and abs(np.arctan2(step[1,0],step[0,0]))<1e-5:break
    moved=source@r.T+t;ids,other=matches(moved)
    rmse=float(np.sqrt(np.mean(np.sum((moved[ids]-target[other])**2,axis=1))))
    overlap=len(ids)/min(len(source),len(target));yaw=float(np.arctan2(r[1,0],r[0,0]))
    if rmse>.15 or overlap<.5 or np.linalg.norm(t)>1 or abs(yaw)>np.deg2rad(15):
        raise ValueError('acoustic_registration_unqualified')
    return {'translation':t.tolist(),'yaw_rad':yaw,'rmse_m':rmse,'mutual_overlap':overlap,'iterations':iteration+1}
