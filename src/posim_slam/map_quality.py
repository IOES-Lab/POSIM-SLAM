"""Native visual-map information contract, independent of a host application.

The metric budget conditions on the native gauge and sensor-noise model. It is
not a calibrated accuracy claim. Empty, unobserved geometry remains a valid
result; relative camera uncertainty cannot substitute for world uncertainty.
"""
import math

WORLD_THREE_SIGMA_M = .1


def native_settings():
    return dict(joint_map_information=True,
                map_point_three_sigma_m=WORLD_THREE_SIGMA_M,
                map_observation_culling=True)


def validate_ready(value):
    if (value.get('map_information_transport') != 'joint_linearized_world_v1'
            or value.get('map_information_enabled') is not True
            or value.get('map_observation_culling') is not True
            or type(value.get('map_point_three_sigma_m')) not in (int,float)
            or not math.isclose(value['map_point_three_sigma_m'],WORLD_THREE_SIGMA_M,
                                rel_tol=0,abs_tol=1e-12)):
        raise ValueError('worker_visual_map_information_contract')


def validate_geometry(value,window):
    required=dict(policy='joint_linearized_position_information',
        pose_uncertainty_included=True,calibration_uncertainty_included=False,
        map_qualified=False,damping_included=False,observed_reprojection_culling=True,
        gauge='native_initial_position_and_yaw_prior',admission_coordinate='world_position',
        host_camera_uncertainty_scope='diagnostic_only')
    if (not isinstance(value,dict) or type(window) is not int or window<0
            or any(type(value.get(k)) is not type(v) or value[k]!=v for k,v in required.items())
            or type(value.get('metric_three_sigma_limit_m')) not in (int,float)
            or not math.isclose(value['metric_three_sigma_limit_m'],WORLD_THREE_SIGMA_M,
                                rel_tol=0,abs_tol=1e-12)
            or type(value.get('pixel_sigma')) not in (int,float)
            or not math.isfinite(value['pixel_sigma']) or value['pixel_sigma']<=0
            or any(type(value.get(k)) is not int or value[k]<0 for k in
                   ('active_eligible','active_unqualified','active_joint_observed','observed_outliers_withdrawn'))
            or value['active_eligible']+value['active_unqualified']!=window
            or not value['active_eligible']<=value['active_joint_observed']<=window):
        raise ValueError('invalid_visual_geometry_report')
    if 'error' not in value:
        raise ValueError('invalid_visual_geometry_report')
    error=value['error']
    if error is not None:
        # The native owner clears all point information on a failed query and
        # continues independently valid tracking. No point can be certified by
        # that query, but this is not a sensor/pose/transport failure.
        if (not isinstance(error,str) or not error.strip() or len(error)>1024
                or value['active_eligible'] or value['active_joint_observed']):
            raise ValueError('invalid_visual_geometry_report')
    return dict(value)
