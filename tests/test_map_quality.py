import unittest
from posim_slam.map_quality import native_settings, validate_ready, validate_geometry


def geometry():
    return dict(policy='persistent_nfr_world_information', pose_uncertainty_included=True,
        calibration_uncertainty_included=False, map_qualified=False, damping_included=False,
        gauge='native_initial_position_and_yaw_prior', admission_coordinate='world_position',
        reprojection_policy='robust_multiview_reoptimization', nfr_approximation=True,
        cross_layer_statistical_independence_verified=False,
        metric_three_sigma_limit_m=.1, pixel_sigma=.5, candidate_points=7, eligible_points=0,
        world_observed_points=0, nonfinite_candidates=1, optimization_revision=1,
        optimized_through_ns=100, keyframes=8, retirements=1, optimize_seconds=.2,
        pending=False, error=None)


class MapQualityTests(unittest.TestCase):
    def test_map_count_and_optimization_cut_must_match_publication(self):
        value=geometry()
        validate_geometry(value,12,total_points=0,stamp_ns=110)
        for total,stamp in [(1,110),(False,110),(0,99),(0,True)]:
            with self.subTest(total=total,stamp=stamp),self.assertRaisesRegex(ValueError,'geometry_report'):
                validate_geometry(value,12,total_points=total,stamp_ns=stamp)

    def test_pending_and_unobserved_maps_never_claim_points(self):
        self.assertEqual(validate_geometry(geometry(),12)['eligible_points'],0)
        value=geometry()
        for key in ('candidate_points','nonfinite_candidates','optimization_revision',
                    'optimized_through_ns','keyframes','retirements'):value[key]=0
        value.update(pending=True,optimize_seconds=0.)
        validate_geometry(value,0)
        with self.assertRaisesRegex(ValueError,'geometry_report'):
            validate_geometry(dict(value,eligible_points=1,world_observed_points=1,candidate_points=1),0)

    def test_ready_requires_persistent_native_mapping_and_the_metric_policy(self):
        self.assertIs(native_settings()['persistent_mapping'],True)
        value=dict(map_information_transport='persistent_nfr_world_v1',
            persistent_map_transport='retained_keyframe_reoptimization_v1',
            map_graph_snapshot='original_measurements_cbor_v1',map_point_identity='original_observation_anchor_v1',
            mapping_epoch='fixture-segment',
            map_snapshot_view='published_snapshot_v1',map_snapshot_update='retained_revision_v1',map_snapshot_timeout_ms=10000,
            map_information_enabled=True,map_point_three_sigma_m=.1)
        validate_ready(value)
        validate_ready(value,epoch='fixture-segment')
        with self.assertRaisesRegex(ValueError,'information_contract'):
            validate_ready(value,epoch='another-segment')
        for key,invalid in [('map_information_transport','joint_linearized_world_v1'),
                ('map_snapshot_update',None),('map_snapshot_view',None),('map_snapshot_timeout_ms',True),('map_snapshot_timeout_ms',2000),
                ('persistent_map_transport',None),('map_information_enabled',False),
                ('map_graph_snapshot',None),('map_point_identity',None),('mapping_epoch',''),('mapping_epoch',None),
                ('map_point_three_sigma_m',True),('map_point_three_sigma_m',0),
                ('map_point_three_sigma_m',float('nan'))]:
            with self.subTest(key=key),self.assertRaisesRegex(ValueError,'information_contract'):
                validate_ready(dict(value,**{key:invalid}))

    def test_false_accuracy_claims_and_inconsistent_counts_are_rejected(self):
        for key,invalid in [('admission_coordinate','host_camera'),('pose_uncertainty_included',False),
                ('damping_included',True),('calibration_uncertainty_included',True),('map_qualified',True),
                ('cross_layer_statistical_independence_verified',True),('world_observed_points',7),
                ('eligible_points',True),('candidate_points',-1),('metric_three_sigma_limit_m',.2),
                ('pixel_sigma',float('inf')),('optimized_through_ns',0),('pending',True),
                ('optimize_seconds',float('nan'))]:
            with self.subTest(key=key),self.assertRaisesRegex(ValueError,'geometry_report'):
                validate_geometry(dict(geometry(),**{key:invalid}),12)

    def test_mapping_failure_withholds_certificates_without_invalidating_navigation(self):
        value=geometry();value['error']='mapping_image_capacity'
        self.assertEqual(validate_geometry(value,12)['error'],value['error'])
        for changes in ({'error':True},{'error':''},{'error':'x'*1025},
                {'world_observed_points':1},{'world_observed_points':1,'eligible_points':1}):
            with self.subTest(changes=changes),self.assertRaisesRegex(ValueError,'geometry_report'):
                validate_geometry(dict(value,**changes),12)
        del value['error']
        with self.assertRaisesRegex(ValueError,'geometry_report'):validate_geometry(value,12)


if __name__=='__main__':unittest.main()
