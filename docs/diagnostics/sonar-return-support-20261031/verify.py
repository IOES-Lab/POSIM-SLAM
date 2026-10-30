"""Independent prepared-input replay and complete conditional-output assessment."""
from pathlib import Path
import gzip
import hashlib
import importlib.util
import json

import numpy as np
import posim_slam
from posim_slam.sonar_joint import linearize_track
from posim_slam.sonar_triangulation import heldout_residuals
from replay_prepared import replay
from observability import compute

ROOT = Path(__file__).resolve().parent
REFERENCE = ROOT.parent/'sonar-inspection-scene-20261031'


def read(name):
    return json.loads((ROOT/name).read_text())


def module(name,path):
    spec = importlib.util.spec_from_file_location(name,path)
    value = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(value)
    return value


def main():
    for name,sha in read('sha256.json').items():
        path = (ROOT/name).resolve()
        assert path.is_relative_to(ROOT)
        assert hashlib.sha256(path.read_bytes()).hexdigest()==sha,name
    for name,sha in read('input-reference.json')['files'].items():
        assert hashlib.sha256((REFERENCE/name).read_bytes()).hexdigest()==sha,name
    source = Path(posim_slam.__file__).resolve().parent
    for name,sha in read('component-source.json').items():
        assert hashlib.sha256((source/name).read_bytes()).hexdigest()==sha,name
    protocol = read('protocol.json')
    assert not protocol['reconstruction']['native_fusion'] and not protocol['reconstruction']['map_qualified']
    assert not protocol['projective']['zero_elevation_assumed']
    calibration = json.loads((REFERENCE/'calibration.json').read_text())
    geometry = {k:calibration['sonar'][k] for k in ('imu_to_sonar','sonar_to_imu',
        'range_min','range_max','horizontal_fov','vertical_fov')}
    geometry['imu_to_sonar'] = np.array(geometry['imu_to_sonar'])
    geometry['sonar_to_imu'] = np.array(geometry['sonar_to_imu'])
    project = ROOT.parents[2]
    candidates = [project/'tools/evaluate_vio.py',project/'libraries/posim-slam/tools/evaluate_vio.py']
    evaluation = module('public_vio',next(p for p in candidates if p.is_file()))
    previous = module('prior_published_assessment',REFERENCE/'verify.py')
    with np.load(REFERENCE/'trajectory-assessment.npz',allow_pickle=False) as data:
        trajectory = dict(data)
    truth = previous.structured(trajectory['truth_stamps_ns'],trajectory['truth_body_poses'])
    nav = previous.structured(trajectory['default_navigation_stamps_ns'],trajectory['default_navigation_poses'])
    metrics,_ = evaluation.evaluate(nav,truth,calibration['imu']['mount'])
    alignment = metrics['evaluation_alignment']
    transform,offset = np.array(alignment['rotation']),np.array(alignment['translation'])
    with np.load(ROOT/'detected-features.npz',allow_pickle=False) as data:
        features = dict(data)
    assert len(features['stamps_ns'])==93 and len(features['pixels'])==2670
    assert features['offsets'][-1]==2670 and np.all(np.diff(features['offsets'])<=300)
    np.testing.assert_array_equal(features['nav_stamps_ns'],nav['stamp_ns'])
    np.testing.assert_allclose(features['nav_positions'],trajectory['default_navigation_poses'][:,:3],atol=1e-12)
    poses = {int(t):dict(position=p,attitude=r) for t,p,r in zip(
        features['nav_stamps_ns'],features['nav_positions'],features['nav_attitudes'])}
    with np.load(REFERENCE/'scene-assessment-only.npz',allow_pickle=False) as data:
        scene = dict(data)
    expected = read('assessment-summary.json')
    association = read('association-summary.json')
    for mode,count in [('descriptor',208),('projective',63)]:
        document = json.loads(gzip.decompress((ROOT/(mode+'-complete.json.gz')).read_bytes()))
        events,tracks = replay(ROOT,mode)
        assert events==document['events'],mode
        assert len(tracks)==expected[mode]['unique_tracks']
        repeated = {k:v for k,v in tracks.items() if len(v['observations'])>=3}
        assert len(repeated)==len(document['tracks'])==count
        assert sum(e['matched'] for e in events)==expected[mode]['matched']
        assert sum(e.get('ambiguous',0) for e in events)==expected[mode]['ambiguous']
        points,position_sigmas,held,errors = [],[],[],[]
        for row in document['tracks']:
            history = repeated[row['track_id']]['observations']
            assert history==row['all_observations']
            assert row['construction']==history[:8] and 3<=len(row['construction'])<=8
            hypotheses = row['hypotheses']
            assert len(hypotheses)+row['invalid_seeds']==9 and hypotheses
            assert all(a['cost']<=b['cost'] for a,b in zip(hypotheses,hypotheses[1:]))
            covariance = np.diag([protocol['reconstruction']['range_sigma_m']**2,
                protocol['reconstruction']['azimuth_sigma_rad']**2]*len(row['construction']))
            for h in hypotheses:
                block = linearize_track(observations=row['construction'],states=poses,landmark=h['point'],
                    measurement_covariance=covariance,**geometry)
                np.testing.assert_allclose(block['residual']@block['residual'],h['cost'],rtol=1e-9,atol=1e-9)
            best = hypotheses[0]
            points.append(best['point'])
            block = linearize_track(observations=row['construction'],states=poses,landmark=best['point'],
                measurement_covariance=covariance,**geometry)
            singular = np.linalg.svd(block['jacobian'][:,-3:],compute_uv=False)
            if best['local_rank']==3:position_sigmas.append(3/singular[-1])
            unused = [o for o in history if o['stamp_ns']>=row['construction'][-1]['stamp_ns']+2_000_000_000]
            assert unused==row['heldout_measurements']
            actual = heldout_residuals(point=best['point'],observations=unused,states=poses,geometry=geometry,
                fit_stamps_ns=[o['stamp_ns'] for o in row['construction']])
            assert actual['total']==row['heldout']['total']
            assert actual['valid_geometry']==row['heldout']['valid_geometry']
            held.extend(actual['observations'])
            errors.append(row['assessment']['surface_error_m'])
        world = np.array(points)@transform.T+offset
        distances = np.column_stack([previous.surface_distances(world,t) for t in scene.values()]).min(axis=1)
        np.testing.assert_allclose(distances,errors,atol=1e-10)
        with np.load(ROOT/(mode+'-assessed-map.npz'),allow_pickle=False) as data:
            np.testing.assert_allclose(world,data['world'],atol=1e-10)
            np.testing.assert_allclose(distances,data['error_m'],atol=1e-10)
        stats = expected[mode]
        assert stats['point_count']==count and not stats['map_qualified'] and not stats['native_fusion']
        assert int((distances>.1).sum())==stats['beyond_10cm']
        np.testing.assert_allclose(np.sqrt(np.mean(distances**2)),stats['surface_error_m']['rmse'],rtol=1e-10)
        np.testing.assert_allclose(np.median(distances),stats['surface_error_m']['median'],rtol=1e-10)
        np.testing.assert_allclose(np.median(position_sigmas),stats['median_fixed_pose_measurement_position_3sigma_m'],rtol=1e-8)
        assert sum(s<=.1 for s in position_sigmas)==stats['measurement_only_3sigma_at_most_10cm']==0
        valid = [o for o in held if o['valid_geometry']]
        assert len(held)==stats['validation']['observations'] and len(valid)==stats['validation']['geometry_valid']
        np.testing.assert_allclose(np.sqrt(np.mean([o['range_residual_m']**2 for o in valid])),stats['validation']['range_rmse_m'],rtol=1e-10)
        ply = (ROOT/(mode+'-map.ply')).read_text().splitlines()
        assert f'element vertex {count}' in ply
        np.testing.assert_allclose(np.array([list(map(float,line.split())) for line in ply[ply.index('end_header')+1:]]),points,atol=1e-12)
    coarse,_ = replay(ROOT,'projective',65)
    fine,_ = replay(ROOT,'projective',257)
    changed = [a['stamp_ns'] for a,b in zip(fine,coarse) if a['observations']!=b['observations']]
    assert changed==association['sampling_control']['different_frames']==[30_500_000_000]
    actual = compute(calibration)
    saved = read('observability-control.json')
    assert len(actual['rows'])==len(saved['rows'])==30
    for a,b in zip(actual['rows'],saved['rows']):
        assert a['local_rank']==b['local_rank'] and a['supported_observations']==b['supported_observations']
        np.testing.assert_allclose(a['singular_values'],b['singular_values'],atol=1e-10,rtol=1e-9)
        if a['local_rank']==3:
            np.testing.assert_allclose(a['fixed_pose_measurement_position_3sigma_m'],b['fixed_pose_measurement_position_3sigma_m'],rtol=1e-9)
        else:
            assert a['fixed_pose_measurement_position_3sigma_m'] is None
    assert all(r['local_rank']==2 for r in actual['rows'] if r['motion']=='stationary')
    print('PASS: identical detections; both complete association histories; all 271 conditional points, '
          'costs and unused-view checks; sampled-arc discrepancy and ideal-motion information. No map accepted.')


if __name__=='__main__':
    main()
