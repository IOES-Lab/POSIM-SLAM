import unittest
from posim_slam.map_quality import native_settings,validate_ready,validate_geometry


def geometry():
    return dict(policy='joint_linearized_position_information',pose_uncertainty_included=True,
        calibration_uncertainty_included=False,map_qualified=False,damping_included=False,
        observed_reprojection_culling=True,gauge='native_initial_position_and_yaw_prior',
        admission_coordinate='world_position',host_camera_uncertainty_scope='diagnostic_only',
        metric_three_sigma_limit_m=.1,pixel_sigma=.5,active_eligible=0,active_unqualified=7,
        active_joint_observed=0,observed_outliers_withdrawn=0,error=None)


class MapQualityTests(unittest.TestCase):
    def test_empty_unobserved_window_is_valid_without_a_false_ready_claim(self):
        self.assertEqual(validate_geometry(geometry(),7)['active_eligible'],0)
        value=geometry();value.update(active_unqualified=0)
        self.assertEqual(validate_geometry(value,0)['active_joint_observed'],0)

    def test_ready_requires_the_requested_metric_policy_and_native_culling(self):
        settings=native_settings()
        value=dict(map_information_transport='joint_linearized_world_v1',map_information_enabled=True,
            map_observation_culling=settings['map_observation_culling'],map_point_three_sigma_m=settings['map_point_three_sigma_m'])
        validate_ready(value)
        for key,invalid in [('map_information_transport',None),('map_information_enabled',False),
                ('map_observation_culling',False),('map_point_three_sigma_m',True),
                ('map_point_three_sigma_m',0),('map_point_three_sigma_m',float('nan'))]:
            with self.subTest(key=key),self.assertRaisesRegex(ValueError,'information_contract'):
                validate_ready(dict(value,**{key:invalid}))

    def test_relative_uncertainty_damping_unknown_errors_and_bad_counts_cannot_pass(self):
        for key,invalid in [('admission_coordinate','host_camera'),('pose_uncertainty_included',False),
                ('damping_included',True),('calibration_uncertainty_included',True),('map_qualified',True),
                ('active_joint_observed',8),('active_eligible',True),('active_unqualified',6),
                ('observed_outliers_withdrawn',-1),('metric_three_sigma_limit_m',.2),('pixel_sigma',float('inf'))]:
            with self.subTest(key=key),self.assertRaisesRegex(ValueError,'geometry_report'):
                validate_geometry(dict(geometry(),**{key:invalid}),7)
        value=geometry();value['error']='unavailable_factor_information'
        with self.assertRaisesRegex(ValueError,'information_unavailable'):validate_geometry(value,7)
        del value['error']
        with self.assertRaisesRegex(ValueError,'information_unavailable'):validate_geometry(value,7)


if __name__=='__main__':unittest.main()
