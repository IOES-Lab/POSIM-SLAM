"""Square-root Gaussian elimination with explicit affine equalities.

Exact calibrated modes are support constraints, not high-weight observations.
Finite noise, unbounded directions and deterministic modes remain distinct.
"""
import numpy as np
from .ping_joint import split_patch_information


def constraint_support(jacobian,target,columns):
    c=np.asarray(jacobian,dtype=float);d=np.asarray(target,dtype=float)
    if c.size==0:c=c.reshape(0,columns)
    if (not 0<=columns<=1024 or c.ndim!=2 or c.shape[1]!=columns or len(c)>1024
            or d.shape!=(len(c),) or not np.isfinite(c).all() or not np.isfinite(d).all()):
        raise ValueError('finite_bounded_affine_constraints_required')
    if not len(c):return np.zeros(columns),np.eye(columns),np.zeros((columns,0))
    u,s,v=np.linalg.svd(c,full_matrices=True)
    tolerance=np.finfo(float).eps*max(c.shape)*(s[0] if len(s) else 0.)
    rank=int(np.sum(s>tolerance));basis=v[:rank].T
    point=basis@((u[:,:rank].T@d)/s[:rank]) if rank else np.zeros(columns)
    if np.linalg.norm(c@point-d)>2e-10*max(1.,float(np.linalg.norm(d))):
        raise ValueError('inconsistent_affine_constraints')
    return point,v[rank:].T,basis


def split_constrained_information(jacobian,residual,*,retained_columns,
                                  constraint_jacobian=None,constraint_target=None):
    """Eliminate retiring coordinates while retaining their degenerate support.

    Input cost is ||J x + r||² on C x = target. Output marginal has finite rows
    plus remaining equalities. The removed variable is M x_kept + offset +
    L noise + U free. deterministic_basis is the orthogonal complement of L/U;
    its affine support remains exact through later finite likelihood updates.
    """
    j=np.asarray(jacobian,dtype=float);r=np.asarray(residual,dtype=float)
    if (j.ndim!=2 or j.shape[0]>4096 or j.shape[1]>1024 or r.shape!=(len(j),)
            or not np.isfinite(j).all() or not np.isfinite(r).all()
            or type(retained_columns) is not int or not 0<=retained_columns<=j.shape[1]):
        raise ValueError('invalid_constrained_information_split')
    n=j.shape[1];k=retained_columns;m=n-k
    c=np.zeros((0,n)) if constraint_jacobian is None else np.asarray(constraint_jacobian,dtype=float)
    d=np.zeros(0) if constraint_target is None else np.asarray(constraint_target,dtype=float)
    constraint_support(c,d,n)
    if not len(c):
        out=split_patch_information(j,r,retained_columns=k)
        out.update(constraint_jacobian=np.zeros((0,k)),constraint_target=np.zeros(0),deterministic_basis=np.zeros((m,0)))
        return out
    gone=c[:,k:];kept=c[:,:k]
    if m:
        u,s,v=np.linalg.svd(gone,full_matrices=True)
        tolerance=np.finfo(float).eps*max(gone.shape)*(s[0] if len(s) else 0.)
        rank=int(np.sum(s>tolerance));hard=v[:rank].T;z=v[rank:].T
        inverse=(hard/s[:rank])@u[:,:rank].T if rank else np.zeros((m,len(c)))
        remaining=u[:,rank:].T
    else:
        inverse=np.zeros((0,len(c)));z=np.zeros((0,0));hard=np.zeros((0,0));remaining=np.eye(len(c))
    p=-inverse@kept;q=inverse@d
    root_c=remaining@kept;root_d=remaining@d
    root_point,_,root_hard=constraint_support(root_c,root_d,k)
    root_c=root_hard.T;root_d=root_c@root_point
    reduced=np.column_stack((j[:,:k]+j[:,k:]@p,j[:,k:]@z))
    split=split_patch_information(reduced,r+j[:,k:]@q,retained_columns=k)
    split.update(backsub_matrix=p+z@split['backsub_matrix'],backsub_offset=q+z@split['backsub_offset'],
        observable_noise_factor=z@split['observable_noise_factor'],unobservable_basis=z@split['unobservable_basis'],
        deterministic_basis=hard,constraint_jacobian=root_c,constraint_target=root_d)
    return split
