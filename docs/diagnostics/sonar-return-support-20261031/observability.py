"""Ideal point-return motion design, separate from the recorded estimator.

All poses are fixed and noise scales remain diagnostic assumptions. This is an
optimistic information calculation, not an executed vehicle/sensor experiment.
"""
from pathlib import Path
import json
import math
import sys

import numpy as np

ROOT = Path(__file__).resolve().parent
INPUT = ROOT.parent/'sonar-inspection-scene-20261031'
from posim_slam.sonar_factor import point_return_factor
from posim_slam.sonar_joint import linearize_track


def rotation(q):
    x,y,z,w = q
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                     [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                     [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])


def axis_rotation(axis, angle):
    c,s = np.cos(angle),np.sin(angle)
    if axis == 'roll':return np.array([[1,0,0],[0,c,-s],[0,s,c]])
    return np.array([[c,-s,0],[s,c,0],[0,0,1]])


def compute(cal):
    geometry = {k:cal['sonar'][k] for k in ('imu_to_sonar','sonar_to_imu',
        'range_min','range_max','horizontal_fov','vertical_fov')}
    geometry['imu_to_sonar'] = np.array(geometry['imu_to_sonar'])
    geometry['sonar_to_imu'] = np.array(geometry['sonar_to_imu'])
    mount = rotation(cal['imu']['mount']['rotation'])
    imu_offset = np.array(cal['imu']['mount']['translation'])
    base = np.array([0.,0.,-2.2])
    sigma_r = .025083611719310283
    sigma_a = .008213314126139592
    rows = []
    for distance in (2.5,5.,10.):
        point = base+imu_offset+mount@(geometry['imu_to_sonar']+geometry['sonar_to_imu']@np.array([distance,0,0]))
        for motion, amplitude in [('stationary',0.),('yaw',15.),('yaw',45.),('yaw',55.),
                                  ('roll',5.),('roll',10.),('roll',15.),('roll',20.),('vertical',.25),('vertical',.5)]:
            states, observations, unavailable = {}, [], []
            sweep = np.linspace(-amplitude,amplitude,16)
            for index, value in enumerate(sweep):
                body = np.eye(3) if motion in ('stationary','vertical') else axis_rotation(motion,np.deg2rad(value))
                position = base+np.array([0.,0.,value if motion=='vertical' else 0.])
                state = dict(position=position+body@imu_offset,attitude=body@mount)
                stamp = (index+1)*500_000_000
                states[stamp] = state
                local = geometry['sonar_to_imu'].T@(state['attitude'].T@(point-state['position'])-geometry['imu_to_sonar'])
                measured_r = float(np.linalg.norm(local))
                measured_a = float(np.arctan2(local[1],local[0]))
                try:
                    point_return_factor(**state,landmark=point,measured_range=measured_r,
                                        measured_azimuth=measured_a,**geometry)
                except ValueError as error:
                    unavailable.append(dict(stamp_ns=stamp,reason=str(error)))
                    continue
                observations.append(dict(stamp_ns=stamp,range_m=measured_r,azimuth_rad=measured_a))
            singular = np.zeros(3)
            rank = 0
            if len(observations)>=2:
                block = linearize_track(observations=observations,states=states,landmark=point,
                    measurement_covariance=np.diag([sigma_r**2,sigma_a**2]*len(observations)),**geometry)
                singular = np.linalg.svd(block['jacobian'][:,-3:],compute_uv=False)
                rank = int(np.sum(singular>singular[0]*1e-8))
            rows.append(dict(source_range_m=distance,motion=motion,half_amplitude=amplitude,
                amplitude_unit='m' if motion=='vertical' else 'deg',total_poses=16,
                supported_observations=len(observations),unsupported=unavailable,
                singular_values=singular.tolist(),local_rank=rank,
                fixed_pose_measurement_position_3sigma_m=float(3/singular[-1]) if rank==3 else None,
                point_world=point.tolist()))
    output = dict(scope='Ideal fixed physical point, exact fixed poses, independent diagnostic range/azimuth noise; not an acquisition or map',
        sigma_range_m=sigma_r,sigma_azimuth_rad=sigma_a,rows=rows,
        installed_sdk_modified=False,actual_roll_control_executed=False,hardware_accuracy_claim=False)
    return output


if __name__=='__main__':
    result=compute(json.loads((INPUT/'calibration.json').read_text()))
    print(json.dumps(result,indent=2))
