"""First geometric returns from a local planar or quadratic height surface.

Surface coordinates are explicit: z = c0+c1*x+c2*y[+c3*x*x+c4*x*y+c5*y*y].
The caller supplies a fixed local frame and the actual authored fan rays in it.
Every ray is intersected; the first positive root can occlude a later root even
inside the sensor's blind range. The minimum over the fan is a conditional
first-return model, not an acoustic intensity-onset or material/noise model.
This module does not assign a surface to an image pixel or estimate vehicle pose.
"""
from __future__ import annotations
import numpy as np


def plane_returns(origin, rays, normal, distance, *, range_min, range_max):
    """First intersections with an explicitly supplied plane n dot X = distance.

    Unlike height surfaces this includes vertical object faces. A plane has no
    finite spatial support here; the caller must establish that separately.
    Normal is unit length. Normal derivatives are ambient coordinates (apply a
    tangent-space increment when preserving its unit norm). No plane is fitted.
    """
    o=np.asarray(origin,dtype=float);d=np.asarray(rays,dtype=float);n=np.asarray(normal,dtype=float)
    if (o.shape!=(3,) or n.shape!=(3,) or d.ndim!=3 or d.shape[2]!=3
            or not 1<=d.shape[0]<=128 or not 1<=d.shape[1]<=512
            or not all(np.isfinite(v).all() for v in (o,d,n))
            or not np.isfinite([distance,range_min,range_max]).all()
            or not 0<range_min<range_max or not np.isclose(np.linalg.norm(n),1.,atol=1e-8,rtol=0)
            or not np.allclose(np.linalg.norm(d,axis=2),1.,atol=1e-8,rtol=0)):
        raise ValueError('invalid_plane_return_geometry')
    denominator=d@n
    usable=abs(denominator)>1e-8
    ranges=np.full(denominator.shape,np.inf)
    np.divide(distance-n@o,denominator,out=ranges,where=usable)
    ranges[(ranges<range_min)|(ranges>range_max)]=np.inf
    selected=ranges.argmin(axis=1);indices=np.arange(len(d));values=ranges[indices,selected]
    valid=np.isfinite(values);values=np.where(valid,values,np.nan)
    denominator=np.where(valid,denominator[indices,selected],np.nan)
    points=o+values[:,None]*d[indices,selected]
    origin_jacobian=-n[None,:]/denominator[:,None]
    return dict(valid=valid,range_m=values,ray_indices=selected,points=points,
        origin_jacobian=origin_jacobian,direction_jacobian=values[:,None]*origin_jacobian,
        normal_jacobian=-points/denominator[:,None],distance_jacobian=1/denominator,
        map_qualified=False,scope='Conditional first return from an unbounded plane; no acoustic identity or pose estimate')


def surface_returns(origin, rays, coefficients, *, range_min, range_max):
    """Return per-beam first-hit ranges and local analytic derivatives.

    rays has shape (beams, vertical rays, 3), with up to 128 x 512 unit rays.
    Ties use the first authored ray. The Jacobian is valid only within the
    selected smooth root/ray branch; tangencies and missing returns are invalid.
    Invalid predictions are NaN and must never become zero-error observations.
    """
    o = np.asarray(origin, dtype=float); d = np.asarray(rays, dtype=float)
    c = np.asarray(coefficients, dtype=float)
    if (o.shape != (3,) or c.shape not in ((3,), (6,)) or d.ndim != 3
            or not 1 <= d.shape[0] <= 128 or not 1 <= d.shape[1] <= 512 or d.shape[2] != 3
            or not all(np.isfinite(x).all() for x in (o,d,c))
            or not np.allclose(np.linalg.norm(d,axis=2),1.,rtol=0,atol=1e-8)
            or not np.isfinite([range_min,range_max]).all() or not 0 < range_min < range_max):
        raise ValueError('invalid_ground_surface_rays')
    count = len(c); c = np.pad(c,(0,6-count)); x,y,z = o
    dx,dy,dz = np.moveaxis(d,-1,0)
    with np.errstate(over='ignore',invalid='ignore'):
        a = -(c[3]*dx*dx+c[4]*dx*dy+c[5]*dy*dy)
        b = dz-c[1]*dx-c[2]*dy-2*c[3]*x*dx-c[4]*(x*dy+y*dx)-2*c[5]*y*dy
        constant = z-(c[0]+c[1]*x+c[2]*y+c[3]*x*x+c[4]*x*y+c[5]*y*y)
        discriminant = b*b-4*a*constant
    if not all(np.isfinite(v).all() for v in (a,b,constant,discriminant)):
        raise ValueError('ground_surface_intersection_overflow')
    roots = np.full((*a.shape,2),np.inf)
    linear = a == 0
    usable_linear = linear & (b != 0)
    roots[:,:,0][usable_linear] = -constant/b[usable_linear]
    quadratic = ~linear & (discriminant >= 0)
    square = np.sqrt(np.maximum(discriminant,0.))
    # Stable quadratic roots avoid subtracting nearly equal b and sqrt(D).
    q = -.5*(b+np.copysign(square,b))
    usable_quadratic = quadratic & (q != 0)
    roots[:,:,0][usable_quadratic] = q[usable_quadratic]/a[usable_quadratic]
    roots[:,:,1][usable_quadratic] = constant/q[usable_quadratic]
    roots[(roots <= 0) | ~np.isfinite(roots)] = np.inf
    first = roots.min(axis=2)
    # A near blind-range hit occludes the rest of that ray, not other rays.
    first[(first < range_min) | (first > range_max)] = np.inf
    chosen = first.argmin(axis=1); index = np.arange(len(d)); distance = first[index,chosen]
    valid = np.isfinite(distance)
    hit = o+np.where(valid,distance,0.)[:,None]*d[index,chosen]
    x,y,z = hit.T
    basis = np.column_stack((np.ones(len(d)),x,y,x*x,x*y,y*y))[:,:count]
    gradient = np.column_stack((-c[1]-2*c[3]*x-c[4]*y,
                                -c[2]-c[4]*x-2*c[5]*y,np.ones(len(d))))
    denominator = np.sum(gradient*d[index,chosen],axis=1)
    valid &= abs(denominator) > 1e-8*np.linalg.norm(gradient,axis=1)
    distance = np.where(valid,distance,np.nan)
    denominator = np.where(valid,denominator,np.nan)
    origin_jacobian = -gradient/denominator[:,None]
    return dict(valid=valid,range_m=distance,ray_indices=chosen,
                points=np.where(valid[:,None],hit,np.nan),
                origin_jacobian=origin_jacobian,
                direction_jacobian=distance[:,None]*origin_jacobian,
                coefficient_jacobian=basis/denominator[:,None],
                map_qualified=False,
                scope='Conditional first geometric return; no acoustic onset, covariance or pose estimate')
