"""Synthetic moving initialization + continued fusion; no pose/attitude inputs."""
import argparse,json,math
from pathlib import Path
import numpy as np
from posim_slam.inertial import ImuWindow,exp_so3
from posim_slam.inertial_alignment import moving_initialize
from evaluate_stationary_initialization import trajectory


def run(seed, calibrated=True):
 rng=np.random.default_rng(seed);dt=.01;noise=np.array([.001,.02,.0001,.001]);w=ImuWindow()
 bg=np.array([.002,-.001,.003]);ba=np.array([.03,-.02,.04])
 mount=exp_so3([.2,-.1,.4]);lever=np.array([.18,-.06,-.1]);dvl=[];truth=[];gyro=[]
 for index in range(2001):
  t=index*dt;p,v,a,r,omega=trajectory(t)
  if index:
   bg+=rng.normal(size=3)*noise[2]*math.sqrt(dt);ba+=rng.normal(size=3)*noise[3]*math.sqrt(dt)
  measured_gyro=omega+bg+rng.normal(size=3)*noise[0]/math.sqrt(dt)
  force=r.T@(a-[0,0,-9.80665])+ba+rng.normal(size=3)*noise[1]/math.sqrt(dt)
  w.add(t,measured_gyro,force);gyro.append(measured_gyro)
  truth.append((p,v,r,bg.copy(),ba.copy()))
  if index%20==0:
   measured=mount.T@(r.T@v+np.cross(omega,lever))+rng.normal(size=3)*.01
   dvl.append((t,measured,np.eye(3)*.01**2,True))
 calibration_rng=np.random.default_rng(1000+seed)
 calibration_sigma=.0005 if calibrated else .01
 calibration_mean=np.array([.002,-.001,.003])+calibration_rng.normal(size=3)*calibration_sigma if calibrated else np.zeros(3)
 initial=[s for s in dvl if s[0]<=12 and abs(s[0]-round(s[0]))<1e-9]
 f,d=moving_initialize(w,initial,dvl_to_imu=mount,imu_to_dvl=lever,gyro_bias=calibration_mean,gyro_bias_covariance=np.eye(3)*calibration_sigma**2,
  accel_bias=[0,0,0],accel_bias_covariance=np.eye(3)*.1**2,noise_density=noise)
 r0=truth[0][2];heading=math.atan2(r0[1,0],r0[0,0]);gauge=exp_so3([0,0,-heading]);origin=truth[1200][0]
 angle=lambda r:math.degrees(math.acos(float(np.clip((np.trace(r)-1)/2,-1,1))))
 initial_angle=angle((gauge@truth[1200][2]).T@f.attitude)
 initial_velocity=float(np.linalg.norm(f.velocity-gauge@truth[1200][1]));errors=[];angles=[];rejected=0
 for sample in dvl:
  t,v,c,locked=sample
  if t<=12:continue
  f.propagate(w,t);index=round(t/dt)
  update=f.update_dvl(v,c,stamp=t,gyro=gyro[index],dvl_to_imu=mount,imu_to_dvl=lever,gyro_covariance=np.eye(3)*noise[0]**2/dt)
  rejected+=not update['accepted']
  if not f.valid:raise AssertionError('moving-initialized segment lost')
  errors.append(float(np.linalg.norm(f.position-gauge@(truth[index][0]-origin))))
  angles.append(angle((gauge@truth[index][2]).T@f.attitude))
 return dict(seed=seed,initialization=d,initial_attitude_error_deg=initial_angle,initial_velocity_error_m_s=initial_velocity,
  followup_position_rmse_m=float(np.sqrt(np.mean(np.square(errors)))),followup_attitude_rmse_deg=float(np.sqrt(np.mean(np.square(angles)))),
  followup_accepted_dvl=f.updates['dvl'],followup_rejected_dvl=int(rejected),minimum_covariance_eigenvalue=float(np.linalg.eigvalsh(f.covariance).min()))


def main():
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',type=Path,required=True);p.add_argument('--weak-prior',action='store_true');a=p.parse_args();runs=[run(seed,not a.weak_prior) for seed in range(6)]
 passed=all(r['initial_attitude_error_deg']<3 and r['initial_velocity_error_m_s']<.08 and r['followup_position_rmse_m']<.25
  and r['followup_attitude_rmse_deg']<3 and r['minimum_covariance_eigenvalue']>0 and r['followup_accepted_dvl']>=35 for r in runs)
 result=dict(passed=passed,scope='synthetic_12s_moving_initialization_then_8s_fusion',stationary_interval=False,
  orientation_or_pose_input=False,truth_observation_updates=False,gyro_calibration='independent_noisy_factory_measurement' if not a.weak_prior else 'uncalibrated_zero_mean',evaluation_gauge='window_start_imu_x_yaw_endpoint_origin_no_trajectory_or_scale_fit',runs=runs)
 a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text(json.dumps(result,indent=2)+'\n')
 print(json.dumps(dict(passed=passed,seeds=6,worst_initial_attitude_error_deg=max(r['initial_attitude_error_deg'] for r in runs))))
 if not passed:raise SystemExit(1)
if __name__=='__main__':main()
