"""Recompute every retained state/point and the actual viewing baseline.

Uses public library helpers and recorded data only. This does not rerun native
estimation, reacquire sensors or certify physical return identity.
"""
from pathlib import Path
import gzip
import hashlib
import importlib.util
import json

import numpy as np
import posim_slam

ROOT = Path(__file__).resolve().parent
REFERENCE = ROOT.parent / 'sonar-inspection-scene-20261031'


def read(name):
    path = ROOT / name
    data = path.read_bytes() if path.exists() else gzip.decompress(path.with_suffix('.json.gz').read_bytes())
    return json.loads(data)


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    result = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(result)
    return result


def ply(name):
    lines = (ROOT / name).read_text().splitlines()
    count = int(next(row.split()[-1] for row in lines if row.startswith('element vertex ')))
    points = np.array([[float(x) for x in row.split()] for row in lines[lines.index('end_header')+1:]])
    assert points.shape == (count, 3)
    return points


def main():
    for name, expected in read('sha256.json').items():
        path = (ROOT / name).resolve()
        assert path.is_relative_to(ROOT) and sha(path) == expected, name
    for name, identity in read('data-identities.json').items():
        raw = gzip.decompress((ROOT / identity['stored']).read_bytes())
        assert len(raw) == identity['uncompressed_bytes']
        assert hashlib.sha256(raw).hexdigest() == identity['uncompressed_sha256'], name
    source = Path(posim_slam.__file__).resolve().parent
    for name, expected in read('component-source.json').items():
        assert sha(source / name) == expected, name
    project = ROOT.parents[2]
    candidates = [project/'tools/evaluate_vio.py', project/'libraries/posim-slam/tools/evaluate_vio.py',
                  source.parents[1]/'tools/evaluate_vio.py']
    evaluation = module('public_trajectory_evaluation', next(p for p in candidates if p.is_file()))
    geometry = module('public_surface_distances', REFERENCE/'verify.py')
    identity = read('source-identity.json')
    assert identity['source_integration'] == 'ffecee9be9976f68c9e883d6cf3bbd6d40eb7729'
    assert identity['capture_application'] == '06ad1d2b1bebdb5af9b299a592d37b601d120f3a'
    assert identity['capture_library'] == '7f48596463fbff92818b4f4b0316ebe50c552303'
    assert identity['source_prs_unmerged']
    for key in ('truth_is_estimator_input', 'rgb_is_estimator_input', 'map_qualified',
                'product_defaults_changed', 'installed_sdk_changed', 'noise_admission_changed'):
        assert not identity[key], key
    review = read('source-head-review.json')
    assert [r['pr'] for r in review['pull_requests']] == [6,31,32,33,34,35,36]
    assert review['all_heads_ancestors'] and all(not r['merged'] for r in review['pull_requests'])
    field = read('field-audit.json')
    assert len(field['all_frames']) == 193 and field['mission_status'] == 'completed'
    assert field['reference_mount_equals_sonar'] and not field['reference_is_estimator_input']
    assert field['minimum_planned_clearance_m'] > 1.7
    stamps = np.array([r['stamp_ns'] for r in field['all_frames']], dtype=np.int64)
    assert np.all(np.diff(stamps) > 0) and np.diff(stamps).max() <= 510_000_000
    hashes = read('raw-input-hashes.json')
    for row in field['all_frames']:
        assert hashes[row['file']] == row['sha256']
        assert row['shape'] == [599,256] and abs(row['reference_delta_ns']) <= 10_000_000
    chosen = read('comparison-selection.json')['selected']
    assert chosen == field['figure_frame']
    for name in (chosen['file'], chosen['reference_rgb']['file'], chosen['isometric_rgb']['file']):
        assert sha(ROOT/'field'/name) == hashes[name]
    with np.load(ROOT/'field'/chosen['file'], allow_pickle=False) as raw:
        assert raw['intensity_db'].shape == (599,256)
        assert not np.isnan(raw['intensity_db']).any() and not np.isposinf(raw['intensity_db']).any()
        np.testing.assert_allclose(raw['ranges'], np.arange(599)*1500/(2*29900), rtol=0, atol=2e-6)
    counts = read('field-receipt.json')
    assert counts['passed'] and counts['error'] is None and not counts['callback_errors']
    assert [counts[k] for k in ('sonar_frames','camera_frames','imu_samples','dvl_samples',
                               'pressure_samples','truth_assessment_samples','isometric_frames',
                               'sonar_reference_frames')] == [193,483,9685,97,968,2419,98,194]
    assert not counts['ping_present'] and len(read('native-dvl.json')) == 97
    cleanup = read('lifecycle.json')
    assert cleanup['exit'] == 0 and not cleanup['forced'] and not cleanup['error']
    assert len(cleanup['groups']) == 4 and all(not g['live_members'] for g in cleanup['groups'])
    context = read('context-receipt.json')
    assert context['exit'] == 0 and not context['forced'] and context['input_record_untouched']
    assert sha(ROOT/'context-isometric-rgb.png') == context['image_sha256']
    assert context['recorded_pose_stamp_ns'] == chosen['stamp_ns']
    for binary in read('binary-trace-proof.json').values():
        assert binary['loaded_trace'] and binary['matches']
        assert all(r['trace_lines'] for r in binary['matches'])
    reference = ROOT / read('scene-reference.json')['path']
    assert sha(reference) == read('scene-reference.json')['sha256']
    with np.load(reference, allow_pickle=False) as file:
        surfaces = dict(file)
    with np.load(ROOT/'trajectory-assessment.npz', allow_pickle=False) as file:
        trajectories = dict(file)
    assert len(trajectories['truth_stamps_ns']) == 2419
    np.testing.assert_array_equal(trajectories['navigation_stamps_ns'], trajectories['point_stamps_ns'])
    np.testing.assert_array_equal(trajectories['navigation_stamps_ns'], trajectories['original_navigation_stamps_ns'])
    np.testing.assert_array_equal(trajectories['navigation_poses'], trajectories['original_navigation_poses'])
    assert read('provenance-only-replay-control.json')['maximum_pose_component_change'] == 0
    calibration = read('calibration.json')
    truth = geometry.structured(trajectories['truth_stamps_ns'], trajectories['truth_body_poses'])
    summary = read('assessment-summary.json')
    archive = read('native-map.json')
    assert not archive['map_qualified'] and not archive['physical_identity_verified']
    assert not archive['joint_uncertainty_available'] and len(archive['points']) == 41
    points = np.array([p['point'] for p in archive['points']])
    np.testing.assert_array_equal(ply('native-map.ply'), points)
    receipts = []
    for mode in ('navigation','point'):
        receipt = read('replay-'+mode+'-receipt.json')
        receipts.append(receipt)
        assert receipt['replay_complete'] and receipt['failure'] is None
        assert receipt['native_states'] == 286 and receipt['camera_frames'] == 0
        assert receipt['exit_code'] == -15 and receipt['worker_shutdown'] == 'client close after replay'
        assert receipt['joint_dvl'] and receipt['joint_pressure'] and not receipt['ground_hypothesis_used']
        assert receipt['sonar_factors_omitted'] == (mode == 'navigation')
        assert receipt['worker_sha256'] == identity['native_worker_sha256']
        states = geometry.structured(trajectories[mode+'_stamps_ns'], trajectories[mode+'_poses'])
        assert len(states) == 286
        metrics, _ = evaluation.evaluate(states, truth, calibration['imu']['mount'])
        expected = summary['default_'+mode]
        assert expected['all_states_assessed'] and not expected['map_qualified']
        np.testing.assert_allclose(metrics['position_rmse_m'], expected['position_rmse_m'], rtol=1e-11)
        np.testing.assert_allclose(np.degrees(metrics['rotation_rmse_rad']), expected['rotation_rmse_deg'], rtol=1e-11)
        if mode == 'point':
            transform = metrics['evaluation_alignment']
            world = points @ np.array(transform['rotation']).T + transform['translation']
            distances = np.column_stack([geometry.surface_distances(world, t) for t in surfaces.values()])
            errors, which = distances.min(axis=1), distances.argmin(axis=1)
            np.testing.assert_array_equal(ply('world-aligned-assessment.ply'), world)
            for key, actual in [('rmse',np.sqrt(np.mean(errors**2))), ('median',np.median(errors)),
                                ('p90',np.quantile(errors,.9)), ('maximum',errors.max())]:
                np.testing.assert_allclose(actual, expected['surface_error_m'][key], rtol=1e-10)
            details = read('assessment-default_point.json')['per_point']
            assert len(details) == 41 and int((errors > .1).sum()) == 19
            for i, row in enumerate(details):
                assert row['source_id'] == archive['points'][i]['source_id']
                assert row['nearest_surface'] == list(surfaces)[which[i]]
                np.testing.assert_allclose(row['error_m'], errors[i], rtol=1e-10, atol=1e-12)
            with np.load(ROOT/'assessed-point-map.npz', allow_pickle=False) as file:
                np.testing.assert_array_equal(file['world'], world)
                np.testing.assert_allclose(file['error_m'], errors, rtol=1e-10, atol=1e-12)
    assert receipts[0]['input_sha256'] == receipts[1]['input_sha256']
    assert summary['default_point']['position_rmse_m'] > summary['default_navigation']['position_rmse_m']
    views = read('measured-view-assessment.json')
    assert len(views['frames']) == views['all_frames'] == 193
    assert not views['physical_reflector_identity_qualified'] and not views['map_accepted']
    block = surfaces['inspection_block_0'].reshape(-1,3)
    target = (block.min(axis=0)+block.max(axis=0))/2
    target[2] = block[:,2].max()
    times, poses = trajectories['truth_stamps_ns'], trajectories['truth_body_poses']
    lever = np.array(calibration['sonar']['sonar_mount']['translation'])
    for row, stamp in zip(views['frames'], stamps):
        assert row['stamp_ns'] == stamp
        index = int(np.argmin(abs(times-stamp)))
        assert abs(int(times[index])-int(stamp)) <= 25_000_000
        origin = poses[index,:3] + evaluation.matrix(poses[index,3:]) @ lever
        np.testing.assert_allclose(origin, row['source_origin'], rtol=0, atol=1e-12)
        np.testing.assert_allclose(poses[index,:3], row['body_position'], rtol=0, atol=1e-12)
        expected = np.arctan2(origin[1]-target[1], origin[0]-target[0])
        np.testing.assert_allclose(expected, row['viewing_bearing_rad'], rtol=0, atol=1e-12)
    corridor = [row for row in views['frames'] if row['in_planned_inspection_corridor']]
    assert len(corridor) == views['inspection_corridor_frames'] == 114
    bearing_span = np.degrees(np.ptp(np.unwrap([r['viewing_bearing_rad'] for r in corridor])))
    np.testing.assert_allclose(bearing_span, views['actual_target_viewing_bearing_span_deg'], rtol=1e-12)
    assert 62 < bearing_span < 64
    assert sum(r['target_center_in_fan'] for r in corridor) == views['center_supported'] == 113
    assert sum(r['all_top_and_flat_shadow_corners_in_fan'] for r in corridor) == 106
    failures = read('failure-preservation.json')
    assert not failures['pre_mission_fixture']['mission_started']
    assert failures['original_postcompute_failure']['driver_exit'] == 1
    assert failures['fixed_replay']['native_worker_exit'] == -15
    assert not failures['fixed_replay']['normal_zero_exit_qualified']
    print('PASS: all 193 input audits, 62.87 degree actual view span, 286 paired states, '
          '41 complete hypotheses, original reporting failure and unchanged replay output. Map remains unqualified.')


if __name__ == '__main__':
    main()
