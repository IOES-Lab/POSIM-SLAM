"""Conservative spatial support for a locally planar visual surface."""
import math
import numpy as np
from .dvl_scale import vector


def require_surface_support(points, intersections, normal, *, max_distance):
    """Require projected intersections inside the hull and near actual samples.

    A convex hull prevents extrapolation beyond the observed patch; a separate
    nearest-sample limit rejects large unobserved holes inside that hull. This
    is a local-planarity assumption, not proof of dense surface occupancy.
    Distance is in the same units as points, supplied by the caller.
    """
    p=np.asarray(points,dtype=float);q=np.asarray(intersections,dtype=float);n=vector(normal)
    if (p.ndim!=2 or p.shape[1]!=3 or not 3<=len(p)<=100000
        or q.ndim!=2 or q.shape[1]!=3 or not 1<=len(q)<=256
        or not np.isfinite(p).all() or not np.isfinite(q).all()
        or not math.isclose(float(n@n),1.,abs_tol=1e-8,rel_tol=0)
        or not math.isfinite(max_distance) or max_distance<=0):
        raise ValueError('invalid_surface_support')
    axis=np.eye(3)[np.argmin(np.abs(n))]
    u=np.cross(n,axis);u/=np.linalg.norm(u);v=np.cross(n,u)
    basis=np.array([u,v]).T;origin=p.mean(axis=0)
    # Normalize to the requested support radius for scale-independent tolerance.
    samples=(p-origin)@basis/max_distance;queries=(q-origin)@basis/max_distance
    ordered=sorted(set(map(tuple,samples)))
    def cross(a,b,c):return (b[0]-a[0])*(c[1]-a[1])-(b[1]-a[1])*(c[0]-a[0])
    def chain(sequence):
        result=[]
        for point in sequence:
            while len(result)>=2 and cross(result[-2],result[-1],point)<=0:result.pop()
            result.append(point)
        return result
    hull=chain(ordered)[:-1]+chain(reversed(ordered))[:-1]
    if len(hull)<3:raise ValueError('degenerate_surface_support')
    for i,a in enumerate(hull):
        b=hull[(i+1)%len(hull)]
        if any(cross(a,b,x)<-1e-10 for x in queries):
            raise ValueError('surface_does_not_cover_ping')
    for query in queries:
        if np.min(np.sum((samples-query)**2,axis=1))>1.+1e-10:
            raise ValueError('surface_support_gap')
