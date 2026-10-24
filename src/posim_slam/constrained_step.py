"""Small active-set least-squares step with linearized lower-bound constraints."""
import numpy as np


def feasible_step(j,residual,margins,gradients,*,max_iterations=40):
    """Minimize ||J d + r|| subject to margins + gradients @ d >= 0.

    Starts at the feasible zero step. Null-space least squares avoids inserting
    damping information into rank-deficient directions. The caller must check
    nonlinear feasibility after this local step.
    """
    j=np.asarray(j,dtype=float);r=np.asarray(residual,dtype=float)
    g=np.asarray(margins,dtype=float);a=np.asarray(gradients,dtype=float)
    if (j.ndim!=2 or j.shape[1]!=3 or r.shape!=(len(j),) or a.shape!=(len(g),3)
            or not all(np.isfinite(v).all() for v in (j,r,g,a)) or np.any(g<0)):
        raise ValueError('invalid_feasible_step')
    d=np.zeros(3);active=[];converged=False
    for _ in range(max_iterations):
        if active:
            _,s,vt=np.linalg.svd(a[active],full_matrices=True)
            rank=int(np.sum(s>max(a[active].shape)*np.finfo(float).eps*s[0]))
            z=vt[rank:].T
        else:z=np.eye(3)
        p=z@np.linalg.lstsq(j@z,-(j@d+r),rcond=1e-10)[0]
        if np.linalg.norm(p)<1e-10:
            if not active:converged=True;break
            multipliers=np.linalg.lstsq(a[active].T,j.T@(j@d+r),rcond=1e-10)[0]
            if np.min(multipliers)>=-1e-9:converged=True;break
            active.pop(int(np.argmin(multipliers)));continue
        slope=a@p;remaining=g+a@d;alpha=1.;blocker=None
        for i in range(len(g)):
            if i not in active and slope[i]<-1e-12:
                bound=max(0.,float(remaining[i]))/(-slope[i])
                if bound<alpha:alpha=bound;blocker=i
        d+=alpha*p
        if blocker is not None:active.append(blocker)
    return dict(step=d,converged=converged,active_constraints=len(active))
