"""Seeded 6DoF IMU/DVL/pressure experiment; synthetic input, not native acceptance.

Truth only generates measurements and scores output. The estimator receives
no truth updates. Initialization uses a known local-frame pose with sampled
prior errors; initialization/observability are not qualified by this experiment.
"""
import argparse,json,math
from pathlib import Path
import numpy as np
from posim_slam.inertial import ImuWindow,exp_so3
from posim_slam.inertial_filter import InertialFilter


def trajectory(t):
    rates=np.array([.37,.29,.21]);amplitudes=np.array([.6,.4,.2])
    position=amplitudes*np.sin(rates*t)+[0,0,-3]
    velocity=amplitudes*rates*np.cos(rates*t)
    acceleration=-amplitudes*rates**2*np.sin(rates*t)
    roll=.12*math.sin(.7*t);pitch=.1*math.sin(.5*t);yaw=.35*math.sin(.3*t)
    rd=.084*math.cos(.7*t);pd=.05*math.cos(.5*t);yd=.105*math.cos(.3*t)
    r=exp_so3([0,0,yaw])@exp_so3([0,pitch,0])@exp_so3([roll,0,0])
    omega=np.array([rd-yd*math.sin(pitch),pd*math.cos(roll)+yd*math.sin(roll)*math.cos(pitch),-pd*math.sin(roll)+yd*math.cos(roll)*math.cos(pitch)])
    return position,velocity,acceleration,r,omega


def run(seed,duration=20.):
    rng=np.random.default_rng(seed);dt=.01;noise=np.array([.0005,.005,.00005,.0005])
    initial=trajectory(0);sigma=np.array([.1]*3+[.2]*3+[.02]*3+[.001]*3+[.1]*3)
    bg=np.array([.002,-.001,.003]);ba=np.array([.03,-.02,.04])
    # Independent 2 s stationary raw gyro record. Mean estimates bias; the
    # conservative 0.001 rad/s prior includes calibration/model uncertainty.
    stationary=bg+rng.normal(size=(200,3))*noise[0]/math.sqrt(dt)
    calibrated_bg=stationary.mean(axis=0)
    error=rng.normal(size=15)*sigma
    args=dict(stamp=0.,position=initial[0]+error[:3],velocity=initial[1]+error[3:6],
              attitude=initial[3]@exp_so3(error[6:9]),gyro_bias=calibrated_bg,accel_bias=error[12:15],
              state_covariance=np.diag(sigma**2),noise_density=noise)
    fused=InertialFilter(**args);inertial=InertialFilter(**args);w=ImuWindow()
    mount=exp_so3([.2,-.1,.4]);dvl_lever=np.array([.18,-.06,-.1]);pressure_lever=np.array([-.1,.03,.2])
    errors=[];baseline=[];velocity_errors=[];orientation_errors=[];minimum_eigenvalue=math.inf
    rejections=0;nis=[]
    for index in range(round(duration/dt)+1):
        t=index*dt;p,v,a,r,omega=trajectory(t)
        if index:
            bg+=rng.normal(size=3)*noise[2]*math.sqrt(dt);ba+=rng.normal(size=3)*noise[3]*math.sqrt(dt)
        measured_gyro=omega+bg+rng.normal(size=3)*noise[0]/math.sqrt(dt)
        specific=r.T@(a-[0,0,-9.80665])+ba+rng.normal(size=3)*noise[1]/math.sqrt(dt)
        w.add(t,measured_gyro,specific)
        if not index:continue
        fused.propagate(w,t);inertial.propagate(w,t)
        if index%20==0:
            measured=mount.T@(r.T@v+np.cross(omega,dvl_lever))+rng.normal(size=3)*.02
            update=fused.update_dvl(measured,np.eye(3)*.02**2,stamp=t,gyro=measured_gyro,
                dvl_to_imu=mount,imu_to_dvl=dvl_lever,gyro_covariance=np.eye(3)*noise[0]**2/dt)
            rejections+=not update['accepted'];nis.append(update['nis'])
        if index%50==0:
            measured=float((p+r@pressure_lever)[2]+rng.normal()*.02)
            update=fused.update_depth(measured,.02**2,stamp=t,imu_to_pressure=pressure_lever)
            rejections+=not update['accepted'];nis.append(update['nis'])
        errors.append(float(np.linalg.norm(fused.position-p)));baseline.append(float(np.linalg.norm(inertial.position-p)))
        velocity_errors.append(float(np.linalg.norm(fused.velocity-v)))
        orientation_errors.append(math.acos(float(np.clip((np.trace(r.T@fused.attitude)-1)/2,-1,1))))
        if index%20==0:minimum_eigenvalue=min(minimum_eigenvalue,float(np.linalg.eigvalsh(fused.covariance).min()))
    return dict(seed=seed,duration_s=duration,samples=len(errors)+1,position_rmse_m=float(np.sqrt(np.mean(np.square(errors)))),
        imu_only_position_rmse_m=float(np.sqrt(np.mean(np.square(baseline)))),velocity_rmse_m_s=float(np.sqrt(np.mean(np.square(velocity_errors)))),
        attitude_rmse_deg=math.degrees(float(np.sqrt(np.mean(np.square(orientation_errors))))),
        gyro_bias_error_rad_s=float(np.linalg.norm(fused.gyro_bias-bg)),accel_bias_error_m_s2=float(np.linalg.norm(fused.accel_bias-ba)),
        accepted_updates=fused.updates,rejected_updates=int(rejections),max_nis=max(nis),minimum_covariance_eigenvalue=minimum_eigenvalue,
        finite=bool(np.isfinite(fused.covariance).all() and np.isfinite(fused.position).all()))


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--seeds',type=int,default=6);args=parser.parse_args()
    if not 1<=args.seeds<=50:parser.error('seeds must be 1..50')
    runs=[run(seed) for seed in range(args.seeds)]
    passed=all(r['finite'] and r['minimum_covariance_eigenvalue']>0 and r['position_rmse_m']<.5
        and r['velocity_rmse_m_s']<.12 and r['position_rmse_m']<r['imu_only_position_rmse_m']*.2 for r in runs)
    result=dict(passed=passed,scope='synthetic_6dof_known_initial_frame_no_truth_updates',alignment='none',scale=1.,initialization='known_local_frame_prior_and_2s_raw_stationary_gyro_calibration',runs=runs)
    args.output.parent.mkdir(parents=True,exist_ok=True);args.output.write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(dict(passed=passed,seeds=len(runs),worst_position_rmse_m=max(r['position_rmse_m'] for r in runs))))
    if not passed:raise SystemExit(1)
if __name__=='__main__':main()
