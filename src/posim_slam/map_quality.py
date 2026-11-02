"""Persistent visual-map information contract, independent of the application.

World uncertainty conditions on the native gauge, declared observation noise
and approximate recovered factors. It is not a calibrated accuracy claim.
"""
import math
from .map_snapshot import VIO_EXPORT_WORKER_SECONDS

WORLD_THREE_SIGMA_M = .1


def native_settings():
    return dict(joint_map_information=True, persistent_mapping=True,
                map_point_three_sigma_m=WORLD_THREE_SIGMA_M)


def validate_ready(value):
    if (value.get('map_information_transport') != 'persistent_nfr_world_v1'
            or value.get('map_snapshot_update') != 'retained_revision_v1'
            or type(value.get('map_snapshot_timeout_ms')) is not int
            or value['map_snapshot_timeout_ms'] != VIO_EXPORT_WORKER_SECONDS*1000
            or value.get('persistent_map_transport') != 'retained_keyframe_reoptimization_v1'
            or value.get('map_information_enabled') is not True
            or type(value.get('map_point_three_sigma_m')) not in (int, float)
            or not math.isclose(value['map_point_three_sigma_m'], WORLD_THREE_SIGMA_M,
                                rel_tol=0, abs_tol=1e-12)):
        raise ValueError('worker_visual_map_information_contract')


def validate_geometry(value, window, *, total_points=None, stamp_ns=None):
    required = dict(policy='persistent_nfr_world_information',
        pose_uncertainty_included=True, calibration_uncertainty_included=False,
        map_qualified=False, damping_included=False,
        gauge='native_initial_position_and_yaw_prior', admission_coordinate='world_position',
        reprojection_policy='robust_multiview_reoptimization', nfr_approximation=True,
        cross_layer_statistical_independence_verified=False)
    counts = ('candidate_points', 'eligible_points', 'world_observed_points',
              'nonfinite_candidates', 'optimization_revision', 'optimized_through_ns',
              'keyframes', 'retirements')
    if (not isinstance(value, dict) or type(window) is not int or window < 0
            or any(type(value.get(k)) is not type(v) or value[k] != v for k, v in required.items())
            or type(value.get('metric_three_sigma_limit_m')) not in (int, float)
            or not math.isclose(value['metric_three_sigma_limit_m'], WORLD_THREE_SIGMA_M,
                                rel_tol=0, abs_tol=1e-12)
            or type(value.get('pixel_sigma')) not in (int, float)
            or not math.isfinite(value['pixel_sigma']) or value['pixel_sigma'] <= 0
            or any(type(value.get(k)) is not int or value[k] < 0 for k in counts)
            or not value['eligible_points'] <= value['world_observed_points'] <= value['candidate_points']
            or value['nonfinite_candidates'] + value['world_observed_points'] > value['candidate_points']
            or type(value.get('pending')) is not bool
            or value['pending'] != (value['optimization_revision'] == 0)
            or type(value.get('optimize_seconds')) not in (int, float)
            or not math.isfinite(value['optimize_seconds']) or value['optimize_seconds'] < 0
            or 'error' not in value):
        raise ValueError('invalid_visual_geometry_report')
    if value['pending'] and any(value[k] for k in counts):
        raise ValueError('invalid_visual_geometry_report')
    if not value['pending'] and (not value['optimized_through_ns'] or value['keyframes'] < 2
                                 or not value['retirements']):
        raise ValueError('invalid_visual_geometry_report')
    error = value['error']
    if error is not None and (
            not isinstance(error, str) or not error.strip() or len(error) > 1024
            or value['eligible_points'] or value['world_observed_points']):
        raise ValueError('invalid_visual_geometry_report')
    if total_points is not None and (type(total_points) is not int
            or total_points != value['eligible_points']):
        raise ValueError('invalid_visual_geometry_report')
    if stamp_ns is not None and (type(stamp_ns) is not int or stamp_ns <= 0
            or value['optimized_through_ns'] > stamp_ns):
        raise ValueError('invalid_visual_geometry_report')
    return dict(value)
