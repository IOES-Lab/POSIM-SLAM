import copy
import importlib.util
from pathlib import Path
import unittest

path=Path(__file__).resolve().parents[1]/'tools/audit_surface_pose_retention.py'
spec=importlib.util.spec_from_file_location('surface_retention_audit',path)
tool=importlib.util.module_from_spec(spec);spec.loader.exec_module(tool)


def inputs():
    predictions=dict(pose_replay_omits_sonar=True,onset_range_used_for_prediction_or_support=False,
        rows=[dict(sonar_stamp_ns=20,dvl_stamp_ns=30,available_stamp_ns=30,beam_id=4,patch_beam_id=2),
              dict(sonar_stamp_ns=40,dvl_stamp_ns=30,available_stamp_ns=40,beam_id=5,patch_beam_id=2)])
    spatial=dict(truth_is_estimator_input=False,online_state_updated=False,
        rows=[dict(stamp_ns=30,beams=[dict(source_id=2,association_rejection=None,
            plane_patch=dict(support_stamps_ns=[10,15,15]))])])
    events=[dict(type='state',stamp_ns=30,sonar=dict(retained_pose_stamps_ns=[15,20,30])),
            dict(type='state',stamp_ns=40,sonar=dict(retained_pose_stamps_ns=[10,15,20,30,40]))]
    return predictions,spatial,events


class SurfacePoseRetentionTests(unittest.TestCase):
    def test_endpoint_coverage_does_not_imply_complete_source_coverage(self):
        result=tool.audit(*inputs())
        self.assertEqual(result['counts'],dict(predictions=2,with_published_state=2,
            both_endpoint_poses_retained=2,all_source_poses_retained=1,no_published_state=0))
        self.assertEqual(result['rows'][0]['missing_pose_stamps_ns'],[10])
        self.assertFalse(result['intermediate_owner_transaction_observed'])

    def test_no_future_snapshot_is_missing_evidence(self):
        predictions,spatial,events=inputs();result=tool.audit(predictions,spatial,events[:1])
        self.assertEqual(result['counts']['no_published_state'],1)
        self.assertIsNone(result['rows'][1]['all_source_poses_retained'])
        self.assertIsNone(result['rows'][1]['missing_pose_stamps_ns'])

    def test_reused_prediction_or_unverified_support_rejected(self):
        args=inputs();args[0]['rows'].append(copy.deepcopy(args[0]['rows'][0]))
        with self.assertRaisesRegex(ValueError,'unique_prediction'):tool.audit(*args)
        args=inputs();args[1]['rows'][0]['beams'][0]['plane_patch']['support_stamps_ns']=[31]
        with self.assertRaisesRegex(ValueError,'prior_support'):tool.audit(*args)
        args=inputs();args[2][1]['stamp_ns']=29
        with self.assertRaisesRegex(ValueError,'ordered_native'):tool.audit(*args)


if __name__=='__main__':unittest.main()
