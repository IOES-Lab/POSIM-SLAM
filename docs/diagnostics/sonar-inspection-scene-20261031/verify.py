"""Recompute bounded trajectory, every-point and exact-pose controls.

Only public POSIM-SLAM code and retained diagnostic data are used. This checks
the record, not sensor reacquisition, physical association or map acceptance.
"""
from pathlib import Path
import hashlib
import importlib.util
import json

import numpy as np
from posim_slam.sonar_input import validate_calibrated_frame
from posim_slam.sonar_joint import linearize_track
from posim_slam.sonar_triangulation import heldout_residuals

ROOT = Path(__file__).resolve().parent


def read(name):
    return json.loads((ROOT / name).read_text())


def surface_distances(points, triangles):
    """Independent Euclidean distance using plane projection and edge segments."""
    triangles = np.asarray(triangles)
    result = np.full(len(points), np.inf)
    for first in range(0, len(triangles), 1024):
        a, b, c = np.moveaxis(triangles[first:first+1024], 1, 0)
        u, v = b-a, c-a
        uu, uv, vv = (np.einsum('ij,ij->i', x, y) for x, y in ((u,u),(u,v),(v,v)))
        denominator = uu*vv-uv*uv
        assert np.all(denominator > 0)
        q = np.asarray(points)[:, None, :] - a
        qu, qv = np.einsum('pij,ij->pi', q, u), np.einsum('pij,ij->pi', q, v)
        s, t = (qu*vv-qv*uv)/denominator, (qv*uu-qu*uv)/denominator
        normal = q-s[:, :, None]*u-t[:, :, None]*v
        squared = np.where((s >= 0) & (t >= 0) & (s+t <= 1),
                           np.einsum('pij,pij->pi', normal, normal), np.inf)
        for start, end in ((a,b),(b,c),(c,a)):
            edge = end-start
            relative = np.asarray(points)[:, None, :] - start
            along = np.clip(np.einsum('pij,ij->pi', relative, edge)
                            / np.einsum('ij,ij->i', edge, edge), 0, 1)
            residual = relative-along[:, :, None]*edge
            squared = np.minimum(squared, np.einsum('pij,pij->pi', residual, residual))
        result = np.minimum(result, np.sqrt(squared.min(axis=1)))
    return result


def structured(stamps, poses):
    names = ('stamp_ns', 'x','y','z','qx','qy','qz','qw')
    result = np.empty(len(stamps), dtype=[(n, 'i8' if n == 'stamp_ns' else 'f8') for n in names])
    result['stamp_ns'] = stamps
    for index, name in enumerate(names[1:]):
        result[name] = poses[:, index]
    return result


def main():
    hashes = read('sha256.json')
    for name, expected in hashes.items():
        path = (ROOT / name).resolve()
        assert path.is_relative_to(ROOT) and path.is_file()
        assert hashlib.sha256(path.read_bytes()).hexdigest() == expected, name
    review = read('source-review.json')
    assert {r['number'] for r in review['source_scope']} == {6,31,32,33,34,35,36}
    assert review['all_heads_are_ancestors'] and not review['source_prs_merged']
    assert all(r['included_in_integration'] and not r['merged'] for r in review['source_scope'])
    identity = read('source-identity.json')
    assert identity['source_integration'] == review['source_integration']
    assert not any(identity[k] for k in ('installed_sdk_changed', 'input_record_estimator_started',
                                       'scene_is_estimator_input', 'map_qualified'))
    assert identity['owned_capture_exit'] == identity['owned_context_exit'] == 0
    audit, calibration = read('field-audit.json'), read('calibration.json')
    rows = audit['all_frames']
    assert len(rows) == audit['field']['sonar_frames'] == 95
    assert audit['mission_status'] == 'completed' and audit['field']['passed']
    assert audit['minimum_planned_clearance_m'] >= 1.25
    assert audit['dvl'] == [dict(valid=True, reason='bottom_locked', locked_beams=4, count=47)]
    assert not audit['field']['ping_present'] and not audit['reference_is_estimator_input']
    assert np.all(np.diff([r['stamp_ns'] for r in rows]) > 0)
    assert all(r['shape'] == [599,256] and abs(r['reference_delta_ns']) <= 10_000_000 for r in rows)
    assert audit['lifecycle']['exit'] == 0 and not audit['lifecycle']['forced']
    assert all(not r['live_members'] for r in audit['lifecycle']['groups'])
    selected = audit['figure_frame']
    path = ROOT / 'field' / selected['file']
    assert hashlib.sha256(path.read_bytes()).hexdigest() == selected['sha256']
    with np.load(path, allow_pickle=False) as data:
        validate_calibrated_frame(dict(data), **{k: calibration['sonar'][k] for k in (
            'beams','horizontal_fov','range_max','bandwidth_hz','sound_speed_m_s')})
    project = ROOT.parents[2]
    candidates = [project/'tools/evaluate_vio.py', project/'libraries/posim-slam/tools/evaluate_vio.py']
    tool = next((p for p in candidates if p.is_file()), None)
    assert tool is not None, 'Run in a public library source checkout, or initialize its application submodule'
    spec = importlib.util.spec_from_file_location('public_vio_evaluation', tool)
    evaluation = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(evaluation)
    with np.load(ROOT / 'trajectory-assessment.npz', allow_pickle=False) as data:
        arrays = dict(data)
    with np.load(ROOT / 'scene-assessment-only.npz', allow_pickle=False) as data:
        scene = dict(data)
    truth = structured(arrays['truth_stamps_ns'], arrays['truth_body_poses'])
    summary = read('assessment-summary.json')
    clocks, alignments = {}, {}
    for mode, expected in summary.items():
        receipt = read(mode+'-replay.json')
        assert receipt['replay_complete'] and receipt['failure'] is None
        assert receipt['joint_dvl'] and receipt['joint_pressure']
        states = structured(arrays[mode+'_stamps_ns'], arrays[mode+'_poses'])
        assert len(states) == expected['states'] == 139
        metrics, _ = evaluation.evaluate(states, truth, calibration['imu']['mount'])
        alignments[mode] = metrics['evaluation_alignment']
        clocks[mode] = states['stamp_ns']
        np.testing.assert_allclose(metrics['position_rmse_m'], expected['position_rmse_m'], rtol=1e-10)
        np.testing.assert_allclose(np.degrees(metrics['rotation_rmse_rad']), expected['rotation_rmse_deg'], rtol=1e-8)
        if 'point_count' not in expected:
            continue
        archive = read(mode+'-map.json')
        assert not any(archive[k] for k in ('map_qualified','physical_identity_verified','joint_uncertainty_available'))
        points = np.array([r['point'] for r in archive['points']])
        transform = alignments[mode]
        world = points @ np.array(transform['rotation']).T + transform['translation']
        distance = np.column_stack([surface_distances(world, t) for t in scene.values()])
        errors = distance.min(axis=1)
        assert len(points) == expected['point_count']
        with np.load(ROOT / (mode+'-assessed.npz'), allow_pickle=False) as saved:
            np.testing.assert_allclose(world, saved['world'], atol=1e-10)
            np.testing.assert_allclose(errors, saved['error_m'], atol=1e-10)
        np.testing.assert_allclose(np.sqrt(np.mean(errors**2)), expected['surface_error_m']['rmse'], rtol=1e-10)
        for limit in (.05,.1,.2):
            assert int((errors > limit).sum()) == expected['diagnostic_counts'][str(limit)]['outside']
        ply = (ROOT/(mode+'-map.ply')).read_text().splitlines()
        assert f'element vertex {len(points)}' in ply
        values = np.array([list(map(float, row.split()[:3])) for row in ply[ply.index('end_header')+1:]])
        np.testing.assert_allclose(values, points, rtol=0, atol=1e-8)
    np.testing.assert_array_equal(clocks['default_navigation'], clocks['default_point'])
    np.testing.assert_array_equal(clocks['default_navigation'], clocks['default_echo_boundary'])

    geometry = {k: calibration['sonar'][k] for k in ('imu_to_sonar','sonar_to_imu',
        'range_min','range_max','horizontal_fov','vertical_fov')}
    geometry['imu_to_sonar'] = np.array(geometry['imu_to_sonar'])
    geometry['sonar_to_imu'] = np.array(geometry['sonar_to_imu'])
    offset = np.array(calibration['imu']['mount']['translation'])
    mount = evaluation.matrix(calibration['imu']['mount']['rotation'])
    times = arrays['truth_stamps_ns']
    poses = arrays['truth_body_poses']
    cache = {}

    def state(stamp):
        if stamp not in cache:
            j = int(np.searchsorted(times, stamp))
            if j == 0 and stamp == times[0]:
                j = 1
            assert 0 < j < len(times) and times[j]-times[j-1] <= 50_000_000
            u = (stamp-times[j-1])/(times[j]-times[j-1])
            p = (1-u)*poses[j-1,:3]+u*poses[j,:3]
            r = evaluation.matrix(evaluation.slerp(poses[j-1,3:], poses[j,3:], u))
            cache[stamp] = dict(position=p+r@offset, attitude=r@mount)
        return cache[stamp]

    for mode, expected_count in [('point',64), ('echo_boundary',32)]:
        document = read('perfect-pose-'+mode+'.json')
        expected, rows = document['summary'], document['points']
        assert len(rows) == expected['emitted_denominator'] == expected['fitted'] == expected_count
        points, sigmas, rms, heldout = [], [], [], []
        for row in rows:
            obs = row['fit_observations']
            states = {r['stamp_ns']: state(r['stamp_ns']) for r in obs}
            covariance = np.diag(row['fit_covariance_diagonal'])
            hypotheses = row['hypotheses']
            assert len(hypotheses) + row['invalid_seeds'] == 9 and hypotheses
            assert all(a['cost'] <= b['cost'] for a,b in zip(hypotheses,hypotheses[1:]))
            for candidate in hypotheses:
                block = linearize_track(observations=obs, states=states, landmark=candidate['point'],
                                        measurement_covariance=covariance, **geometry)
                np.testing.assert_allclose(block['residual']@block['residual'], candidate['cost'], rtol=1e-9, atol=1e-9)
            best = hypotheses[0]
            points.append(best['point'])
            rms.append(np.sqrt(best['cost']/(2*len(obs))))
            block = linearize_track(observations=obs, states=states, landmark=best['point'],
                                    measurement_covariance=covariance, **geometry)
            singular = np.linalg.svd(block['jacobian'][:,-3:], compute_uv=False)
            if best['local_rank'] == 3:
                sigmas.append(3/singular[-1])
            unused = row['heldout_measurements']
            assert all(o['stamp_ns'] >= obs[-1]['stamp_ns']+2_000_000_000 for o in unused)
            check = heldout_residuals(point=best['point'], observations=unused,
                states={o['stamp_ns']:state(o['stamp_ns']) for o in unused}, geometry=geometry,
                fit_stamps_ns=[o['stamp_ns'] for o in obs])
            assert check['total'] == row['heldout']['total']
            assert check['valid_geometry'] == row['heldout']['valid_geometry']
            heldout.extend(check['observations'])
        errors = np.column_stack([surface_distances(points, t) for t in scene.values()]).min(axis=1)
        np.testing.assert_allclose(errors, [r['selected_error_m'] for r in rows], atol=1e-10)
        assert int((errors > .1).sum()) == expected['outside_10cm']
        np.testing.assert_allclose(np.sqrt(np.mean(errors**2)), expected['surface_rmse_m'], rtol=1e-10)
        np.testing.assert_allclose(np.median(rms), expected['fit_whitened_rms_median'], rtol=1e-10)
        np.testing.assert_allclose(np.median(sigmas), expected['measurement_only_max_position_3sigma_median_m'], rtol=1e-8)
        valid = [r for r in heldout if r['valid_geometry']]
        assert len(heldout) == expected['heldout_after_2s']['total_observations']
        assert len(valid) == expected['heldout_after_2s']['valid_geometry']
        np.testing.assert_allclose(np.sqrt(np.mean([r['range_residual_m']**2 for r in valid])),
                                   expected['heldout_after_2s']['range_rmse_m'], rtol=1e-10)
    print('PASS: source heads, 95-frame audit, representative raw frame, all 139-state trajectories, '
          'all 96 emitted points, nine-seed exact-pose costs and unused observations. No useful map accepted.')


if __name__ == '__main__':
    main()
