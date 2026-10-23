"""Synthetic IMU/DVL initialization and moving follow-up, without pose priors.

Uses a stationary fixture followed by smooth 6DoF motion. Truth generates raw
measurements and evaluates output only. Position/yaw are fixed once by the true
initial local gauge for evaluation, with no trajectory/scale fitting.
"""
import argparse,json,math
from pathlib import Path
import numpy as np
from posim_slam.inertial import ImuWindow,exp_so3
from posim_slam.inertial_initialization import stationary_initialize


def trajectory(t):
    rates=np.array([.37,.29,.21]);amp=np.array([.6,.4,.2])
    p=amp*(1-np.cos(rates*t));v=amp*rates*np.sin(rates*t);a=amp*rates**2*np.cos(rates*t)
    roll=.12*(1-math.cos(.7*t));pitch=.1*(1-math.cos(.5*t));yaw=.35*(1-math.cos(.3*t))
    rd=.084*math.sin(.7*t);pd=.05*math.sin(.5*t);yd=.105*math.sin(.3*t)
    mounting=exp_so3([.2,-.15,.4])
    r=exp_so3([0,0,yaw])@exp_so3([0,pitch,0])@exp_so3([roll,0,0])@mounting
    omega=mounting.T@np.array([rd-yd*math.sin(pitch),pd*math.cos(roll)+yd*math.sin(roll)*math.cos(pitch),-pd*math.sin(roll)+yd*math.cos(roll)*math.cos(pitch)])
    return p,v,a,r,omega


def run(seed):
    rng=np.random.default_rng(seed);dt=.01;noise=np.array([.0005,.005,.00005,.0005])
    bg=np.array([.002,-.001,.003]);ba=np.array([.03,-.02,.04]);w=ImuWindow()
    dvl_mount=exp_so3([.2,-.1,.4]);lever=np.array([.18,-.06,-.1]);stationary=[]
    r0=trajectory(0)[3]
    for index in range(201):
        t=index*dt
        w.add(t,bg+rng.normal(size=3)*noise[0]/math.sqrt(dt),r0.T@[0,0,9.80665]+ba+rng.normal(size=3)*noise[1]/math.sqrt(dt))
        if index%100==0:stationary.append((t,rng.normal(size=3)*.002,np.eye(3)*.002**2,True))
    # Only measured IMU, measured DVL and explicit noise/mount/bias calibration
    # enter initialization. No initial orientation, velocity or pose argument.
    f,diagnostics=stationary_initialize(w,0.,2.,stationary,dvl_to_imu=dvl_mount,imu_to_dvl=lever,
        accel_bias=[0,0,0],accel_bias_covariance=np.eye(3)*.1**2,noise_density=noise,
        angular_stationarity_std=.001,linear_stationarity_std=.01)
    # Separate evaluation gauge: initial true horizontal IMU X defines yaw.
    heading=math.atan2(r0[1,0],r0[0,0]);evaluate_rotation=exp_so3([0,0,-heading])
    initial_tilt_error=math.acos(float(np.clip((np.trace((evaluate_rotation@r0).T@f.attitude)-1)/2,-1,1)))
    errors=[];velocity_errors=[];angle_errors=[];rejected=0;minimum_eigenvalue=math.inf
    for index in range(1,2001):
        t=index*dt;p,v,a,r,omega=trajectory(t)
        bg+=rng.normal(size=3)*noise[2]*math.sqrt(dt);ba+=rng.normal(size=3)*noise[3]*math.sqrt(dt)
        gyro=omega+bg+rng.normal(size=3)*noise[0]/math.sqrt(dt)
        force=r.T@(a-[0,0,-9.80665])+ba+rng.normal(size=3)*noise[1]/math.sqrt(dt)
        w.add(2+t,gyro,force);f.propagate(w,2+t)
        if index%20==0:
            measured=dvl_mount.T@(r.T@v+np.cross(omega,lever))+rng.normal(size=3)*.02
            update=f.update_dvl(measured,np.eye(3)*.02**2,stamp=2+t,gyro=gyro,dvl_to_imu=dvl_mount,
                imu_to_dvl=lever,gyro_covariance=np.eye(3)*noise[0]**2/dt)
            rejected+=not update['accepted']
            if not f.valid:raise AssertionError('initialized segment lost')
            minimum_eigenvalue=min(minimum_eigenvalue,float(np.linalg.eigvalsh(f.covariance).min()))
        errors.append(float(np.linalg.norm(f.position-evaluate_rotation@p)))
        velocity_errors.append(float(np.linalg.norm(f.velocity-evaluate_rotation@v)))
        angle_errors.append(math.acos(float(np.clip((np.trace((evaluate_rotation@r).T@f.attitude)-1)/2,-1,1))))
    return dict(seed=seed,initialization=diagnostics,initial_tilt_error_deg=math.degrees(initial_tilt_error),
        position_rmse_m=float(np.sqrt(np.mean(np.square(errors)))),velocity_rmse_m_s=float(np.sqrt(np.mean(np.square(velocity_errors)))),
        attitude_rmse_deg=math.degrees(float(np.sqrt(np.mean(np.square(angle_errors))))),
        accepted_dvl=f.updates['dvl'],rejected_dvl=int(rejected),minimum_covariance_eigenvalue=minimum_eigenvalue,
        gyro_bias_error_rad_s=float(np.linalg.norm(f.gyro_bias-bg)),accel_bias_error_m_s2=float(np.linalg.norm(f.accel_bias-ba)))


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',type=Path,required=True);args=parser.parse_args()
    runs=[run(seed) for seed in range(6)]
    passed=all(r['position_rmse_m']<.25 and r['velocity_rmse_m_s']<.08 and r['attitude_rmse_deg']<3
        and r['minimum_covariance_eigenvalue']>0 and r['accepted_dvl']>=90 for r in runs)
    result=dict(passed=passed,scope='synthetic_stationary_raw_imu_dvl_then_20s_6dof',
        initial_pose_input=False,orientation_input=False,truth_observation_updates=False,
        evaluation_gauge='initial_true_origin_and_horizontal_imu_x_yaw_only_no_trajectory_fit',scale=1.,runs=runs)
    args.output.parent.mkdir(parents=True,exist_ok=True);args.output.write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(dict(passed=passed,seeds=6,worst_position_rmse_m=max(r['position_rmse_m'] for r in runs))))
    if not passed:raise SystemExit(1)
if __name__=='__main__':main()
