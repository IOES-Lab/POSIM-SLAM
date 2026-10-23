"""Experimental moving IMU/DVL alignment; no stationary or ideal-pose input.

A bounded factor graph estimates endpoint rotations/velocities, gravity and
constant interval biases. Explicit bias calibration priors support weak motion;
acceptance never means every bias is data-observable or navigation is qualified.
"""
import math
import numpy as np
from .dvl_scale import vector,rotation
from .inertial import exp_so3,skew
from .inertial_filter import InertialFilter,covariance

ALGORITHM_VERSION='0.1.0'


def log_so3(r):
    angle=math.acos(float(np.clip((np.trace(r)-1)/2,-1,1)))
    vee=np.array([r[2,1]-r[1,2],r[0,2]-r[2,0],r[1,0]-r[0,1]])
    if angle<1e-7:return vee*.5
    if angle>math.pi-.01:raise ValueError('alignment_rotation_singular')
    return vee*(angle/(2*math.sin(angle)))


def gravity_attitude(up):
    axis=np.array([1.,0.,0.]) if abs(up[0])<.95 else np.array([0.,1.,0.])
    x=axis-up*np.dot(axis,up);x/=np.linalg.norm(x)
    return np.vstack((x,np.cross(up,x),up))


def moving_initialize(window,dvl_samples,*,dvl_to_imu,imu_to_dvl,gyro_bias,gyro_bias_covariance,
                      accel_bias,accel_bias_covariance,noise_density,gravity_m_s2=9.80665):
    """8–24 bottom-locked DVL endpoints, 7–30 s of covered raw IMU.

    Samples use the stationary initializer's tuple contract. Biases are constant
    over this short graph, with supplied priors; raw gyro is never averaged as a
    zero-rate bias. Position/yaw define a local gauge. Pressure needs the matching
    vertical origin. Endpoint velocity/attitude/bias correlations are retained.
    """
    samples=list(dvl_samples);n=len(samples)
    if not 8<=n<=24:raise ValueError('moving_dvl_samples_required')
    mount=rotation(dvl_to_imu);lever=vector(imu_to_dvl)
    bg0=vector(gyro_bias);ba0=vector(accel_bias)
    bg_cov=covariance(gyro_bias_covariance,3,allow_singular=True);ba_cov=covariance(accel_bias_covariance,3,allow_singular=True)
    noise=np.asarray(noise_density,dtype=float)
    if noise.shape!=(4,) or not np.isfinite(noise).all() or np.any(noise<=0):raise ValueError('imu_noise_calibration_required')
    if not math.isfinite(gravity_m_s2) or not 9<gravity_m_s2<11:raise ValueError('invalid_gravity')
    times=[];measured=[];whiten_dvl=[]
    for stamp,velocity,uncertainty,locked in samples:
        if type(stamp) not in (int,float) or not math.isfinite(stamp) or stamp<0:raise ValueError('invalid_dvl_timestamp')
        if type(locked) is not bool or not locked:raise ValueError('dvl_bottom_lock_required')
        if times and not 0<stamp-times[-1]<=1.5:raise ValueError('moving_dvl_time_gap')
        times.append(float(stamp));measured.append(vector(velocity));whiten_dvl.append(np.linalg.inv(np.linalg.cholesky(covariance(uncertainty,3))))
    if not 7<=times[-1]-times[0]<=30:raise ValueError('moving_alignment_duration_required')
    pre=[];gyros=[];rotations=[np.eye(3)];indices=[6,7,8,3,4,5]
    for i in range(n-1):
        value=window.preintegrate(times[i],times[i+1],gyro_bias=bg0,accel_bias=ba0,noise_density=noise)
        value['whiten']=np.linalg.inv(np.linalg.cholesky(value['covariance'][np.ix_(indices,indices)]))
        value['dt']=times[i+1]-times[i];pre.append(value)
        knots,raw=window.interval(times[i],times[i+1]);gyros.append(raw[0,:3])
        rotations.append(rotations[-1]@value['rotation'])
    gyros.append(raw[-1,:3])
    velocities=np.array([r@(mount@v-np.cross(omega-bg0,lever)) for r,v,omega in zip(rotations,measured,gyros)])
    gravity=np.mean([(velocities[i+1]-velocities[i]-rotations[i]@p['delta_velocity'])/p['dt'] for i,p in enumerate(pre)],axis=0)
    magnitude=np.linalg.norm(gravity)
    if not 5<magnitude<15:raise ValueError('moving_gravity_inconsistent')
    gravity=gravity/magnitude*gravity_m_s2
    state=dict(g=gravity,bg=bg0.copy(),ba=ba0.copy(),r=rotations,v=velocities)
    dimension=9+3*(n-1)+3*n
    def ri(i):return 9+3*(i-1)
    def vi(i):return 9+3*(n-1)+3*i
    def prior_basis(c):
        values,axes=np.linalg.eigh(c);positive=values>0
        return axes[:,positive],(axes[:,positive]/np.sqrt(values[positive])).T
    bg_axes,wb=prior_basis(bg_cov);ba_axes,wa=prior_basis(ba_cov)
    # Zero-variance calibrated directions are exact constraints, not tiny-noise
    # pseudo-observations. Optimize only the remaining degrees of freedom.
    columns=[np.eye(dimension)[:,i] for i in range(3)]
    for offset,axes in ((3,bg_axes),(6,ba_axes)):
        for axis in axes.T:
            column=np.zeros(dimension);column[offset:offset+3]=axis;columns.append(column)
    columns.extend(np.eye(dimension)[:,i] for i in range(9,dimension))
    basis=np.array(columns).T;free_dimension=basis.shape[1]
    def apply(s,delta,amount=1.):
        return dict(g=s['g']+amount*delta[:3],bg=s['bg']+amount*delta[3:6],ba=s['ba']+amount*delta[6:9],
            r=[s['r'][0]]+[s['r'][i]@exp_so3(amount*delta[ri(i):ri(i)+3]) for i in range(1,n)],
            v=s['v']+amount*delta[vi(0):].reshape(n,3))
    def build(s,jacobian=True):
        rows=[];matrices=[];epsilon=1e-6
        def add(residual,j,w):
            rows.extend(w@residual)
            if jacobian:matrices.extend(w@j)
        for i in range(n):
            r=s['r'][i];body=r.T@s['v'][i]
            predicted=mount.T@(body+np.cross(gyros[i]-s['bg'],lever))
            j=np.zeros((3,dimension))
            if jacobian:
                j[:,3:6]=mount.T@skew(lever);j[:,vi(i):vi(i)+3]=mount.T@r.T
                if i:j[:,ri(i):ri(i)+3]=mount.T@skew(body)
            add(predicted-measured[i],j,whiten_dvl[i])
        for i,p in enumerate(pre):
            r=s['r'][i];relative=r.T@s['r'][i+1];db=np.r_[s['bg']-bg0,s['ba']-ba0]
            def rot_res(relative,bias):return log_so3((p['rotation']@exp_so3(p['bias_jacobian'][6:9]@bias)).T@relative)
            angle=rot_res(relative,db)
            dv=r.T@(s['v'][i+1]-s['v'][i]-s['g']*p['dt'])
            residual=np.r_[angle,dv-p['delta_velocity']-p['bias_jacobian'][3:6]@db]
            j=np.zeros((6,dimension))
            if jacobian:
                j[3:,:3]=-r.T*p['dt'];j[3:,3:9]=-p['bias_jacobian'][3:6]
                j[3:,vi(i):vi(i)+3]=-r.T;j[3:,vi(i+1):vi(i+1)+3]=r.T
                if i:j[3:,ri(i):ri(i)+3]=skew(dv)
                for axis in range(3):
                    e=np.eye(3)[axis]*epsilon
                    if i:j[:3,ri(i)+axis]=(rot_res(exp_so3(-e)@relative,db)-rot_res(exp_so3(e)@relative,db))/(2*epsilon)
                    j[:3,ri(i+1)+axis]=(rot_res(relative@exp_so3(e),db)-rot_res(relative@exp_so3(-e),db))/(2*epsilon)
                for axis in range(6):
                    e=np.eye(6)[axis]*epsilon
                    j[:3,3+axis]=(rot_res(relative,db+e)-rot_res(relative,db-e))/(2*epsilon)
            add(residual,j,p['whiten'])
        j=np.zeros((3,dimension));j[:,3:6]=np.eye(3);add(s['bg']-bg0,j,wb)
        j=np.zeros((3,dimension));j[:,6:9]=np.eye(3);add(s['ba']-ba0,j,wa)
        norm=np.linalg.norm(s['g']);j=np.zeros((1,dimension));j[0,:3]=s['g']/norm
        add(np.array([norm-gravity_m_s2]),j,np.array([[100.]]))
        return np.asarray(rows),np.asarray(matrices) if jacobian else None
    iterations=0
    for iterations in range(12):
        residual,j=build(state);cost=float(residual@residual)
        delta=basis@np.linalg.lstsq(j@basis,-residual,rcond=1e-10)[0]
        if np.linalg.norm(delta)<1e-6:break
        accepted=False
        for step in (1.,.5,.25,.125,.0625,.03125):
            candidate=apply(state,delta,step);candidate_residual,_=build(candidate,False)
            if candidate_residual@candidate_residual<cost:
                state=candidate;accepted=True;break
        if not accepted:
            if np.linalg.norm((j@basis).T@residual,np.inf)<1e-3:break
            raise ValueError('moving_alignment_not_converged')
    else:
        raise ValueError('moving_alignment_iteration_limit')
    residual,j=build(state);reduced_j=j@basis;singular=np.linalg.svd(reduced_j,compute_uv=False)
    if singular[-1]<=singular[0]*1e-10:raise ValueError('moving_alignment_rank_deficient')
    reduced=float(residual@residual)/max(1,len(residual)-free_dimension)
    if reduced>5 or np.max(np.abs(residual))>8:raise ValueError('moving_alignment_residual_rejected')
    if np.linalg.norm(state['bg']-bg0)>.05 or np.linalg.norm(state['ba']-ba0)>.5:
        raise ValueError('moving_bias_linearization_exceeded')
    posterior=basis@np.linalg.solve(reduced_j.T@reduced_j,np.eye(free_dimension))@basis.T*max(1.,reduced)
    def endpoint(s):
        up=-s['g']/np.linalg.norm(s['g']);world=gravity_attitude(up)
        return world@s['v'][-1],world@s['r'][-1]
    velocity,attitude=endpoint(state)
    transform=np.zeros((15,dimension));transform[9:12,3:6]=np.eye(3);transform[12:15,6:9]=np.eye(3)
    relevant=list(range(3))+list(range(ri(n-1),ri(n-1)+3))+list(range(vi(n-1),vi(n-1)+3))
    for axis in relevant:
        e=np.eye(dimension)[axis]*1e-6;plus=endpoint(apply(state,e));minus=endpoint(apply(state,-e))
        transform[3:6,axis]=(plus[0]-minus[0])/2e-6
        transform[6:9,axis]=(log_so3(attitude.T@plus[1])-log_so3(attitude.T@minus[1]))/2e-6
    p=transform@posterior@transform.T;p[:3,:3]=np.eye(3)*1e-8;p=(p+p.T)*.5
    estimator=InertialFilter(stamp=times[-1],position=[0,0,0],velocity=velocity,attitude=attitude,
        gyro_bias=state['bg'],accel_bias=state['ba'],state_covariance=p,noise_density=noise,gravity=[0,0,-gravity_m_s2])
    data_j=j[:3*n+6*(n-1)]@basis
    data_singular=np.linalg.svd(data_j,compute_uv=False)
    return estimator,dict(algorithm_version=ALGORITHM_VERSION,start=times[0],end=times[-1],dvl_samples=n,
        iterations=iterations+1,reduced_chi_square=reduced,max_whitened_residual=float(np.max(np.abs(residual))),
        covariance_condition=float((singular[0]/singular[-1])**2),data_rank=int(np.sum(data_singular>data_singular[0]*1e-8)),
        variables=free_dimension,fixed_bias_directions=6-bg_axes.shape[1]-ba_axes.shape[1],calibration_priors_used=True,bias_observability_qualified=False,
        attitude_three_sigma_deg=float(3*np.sqrt(np.linalg.eigvalsh(p[6:9,6:9]).max())*180/math.pi),
        velocity_three_sigma_m_s=float(3*np.sqrt(np.linalg.eigvalsh(p[3:6,3:6]).max())),
        stationary_required=False,orientation_measurement_used=False,navigation_ready=False,
        initial_position='endpoint_local_origin',initial_yaw='window_start_imu_x_gauge')
