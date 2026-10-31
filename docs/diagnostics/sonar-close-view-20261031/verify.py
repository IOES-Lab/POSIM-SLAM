"""Verify all retained measurements, failures, points and association histories."""
from pathlib import Path
import gzip
import hashlib
import importlib.util
import json
import sys

import numpy as np
import posim_slam
from posim_slam.sonar_joint import linearize_track
from posim_slam.sonar_triangulation import heldout_residuals

ROOT=Path(__file__).resolve().parent
COMMON=ROOT.parent/'sonar-return-support-20261031'
REFERENCE=ROOT.parent/'sonar-inspection-scene-20261031'
sys.path.insert(0,str(COMMON))
from replay_prepared import replay


def read(name):
    return json.loads((ROOT/name).read_text())


def module(name,path):
    spec=importlib.util.spec_from_file_location(name,path)
    result=importlib.util.module_from_spec(spec)
    spec.loader.exec_module(result)
    return result


def origins(events):
    births,frames={},[]
    for frame in events:
        mapping={}
        for observation in frame['observations']:
            key=observation['track_id']
            if observation['length']==1:
                assert key not in births
                births[key]=[frame['stamp_ns'],observation['pixel_index']]
            mapping[observation['pixel_index']]=births[key]
        frames.append(mapping)
    return frames


def main():
    for name,sha in read('sha256.json').items():
        path=(ROOT/name).resolve()
        assert path.is_relative_to(ROOT)
        assert hashlib.sha256(path.read_bytes()).hexdigest()==sha,name
    source=Path(posim_slam.__file__).resolve().parent
    for name,sha in read('component-source.json').items():
        assert hashlib.sha256((source/name).read_bytes()).hexdigest()==sha,name
    reference=ROOT/read('scene-reference.json')['path']
    assert hashlib.sha256(reference.read_bytes()).hexdigest()==read('scene-reference.json')['sha256']
    identity=read('source-identity.json')
    assert identity['source_integration']=='ffecee9be9976f68c9e883d6cf3bbd6d40eb7729'
    assert identity['source_prs_unmerged'] and not identity['product_defaults_changed']
    assert not identity['installed_sdk_changed'] and not identity['truth_is_estimator_input'] and not identity['map_qualified']
    review=read('source-head-review.json')
    assert [r['pr'] for r in review['pull_requests']]==[6,31,32,33,34,35,36]
    assert review['all_heads_ancestors'] and all(not r['merged'] for r in review['pull_requests'])
    field=read('field-audit.json')
    assert len(field['all_frames'])==150 and field['mission_status']=='completed'
    assert field['minimum_planned_clearance_m']>=1.25
    assert field['reference_mount_equals_sonar'] and not field['reference_is_estimator_input']
    stamps=np.array([r['stamp_ns'] for r in field['all_frames']],dtype=np.int64)
    assert np.all(np.diff(stamps)>0) and np.diff(stamps).max()<=510_000_000
    hashes=read('raw-input-hashes.json')
    for row in field['all_frames']:
        assert hashes[row['file']]==row['sha256']
        assert abs(row['reference_delta_ns'])<=10_000_000
        assert row['shape']==[599,256]
    chosen=field['figure_frame']
    with np.load(ROOT/'field'/chosen['file'],allow_pickle=False) as frame:
        assert frame['intensity_db'].shape==(599,256)
        assert not np.isnan(frame['intensity_db']).any() and not np.isposinf(frame['intensity_db']).any()
        assert hashlib.sha256((ROOT/'field'/chosen['file']).read_bytes()).hexdigest()==chosen['sha256']
        np.testing.assert_allclose(frame['ranges'],np.arange(599)*1500/(2*29900),rtol=0,atol=2e-6)
    lifecycle=read('lifecycle.json')
    assert lifecycle['exit']==0 and not lifecycle['forced'] and not lifecycle['error']
    assert len(lifecycle['groups'])==4 and not any(g['live_members'] for g in lifecycle['groups'])
    project=ROOT.parents[2]
    paths=[project/'tools/evaluate_vio.py',project/'libraries/posim-slam/tools/evaluate_vio.py',source.parents[1]/'tools/evaluate_vio.py']
    evaluation=module('public_evaluate',next(p for p in paths if p.is_file()))
    prior=module('public_surfaces',REFERENCE/'verify.py')
    with np.load(reference,allow_pickle=False) as file:
        surfaces=dict(file)
    with np.load(ROOT/'trajectory-assessment.npz',allow_pickle=False) as file:
        trajectory=dict(file)
    calibration=read('calibration.json')
    truth=prior.structured(trajectory['truth_stamps_ns'],trajectory['truth_body_poses'])
    assert np.array_equal(trajectory['navigation_stamps_ns'],trajectory['point_stamps_ns'])
    assert len(trajectory['navigation_stamps_ns'])==223
    native=read('native-map.json')
    assert not native['physical_identity_verified'] and not native['joint_uncertainty_available']
    native_stats=read('native-assessment-summary.json')
    for mode in ('navigation','point'):
        states=prior.structured(trajectory[mode+'_stamps_ns'],trajectory[mode+'_poses'])
        metrics,_=evaluation.evaluate(states,truth,calibration['imu']['mount'])
        expected=native_stats['default_'+mode]
        np.testing.assert_allclose(metrics['position_rmse_m'],expected['position_rmse_m'],rtol=1e-10)
        if mode=='point':
            points=np.array([r['point'] for r in native['points']])
            assert len(points)==27
            transform=metrics['evaluation_alignment']
            world=points@np.array(transform['rotation']).T+transform['translation']
            distance=np.column_stack([prior.surface_distances(world,t) for t in surfaces.values()]).min(axis=1)
            assert int((distance>.1).sum())==7
            np.testing.assert_allclose(np.sqrt(np.mean(distance**2)),expected['surface_error_m']['rmse'],rtol=1e-10)
    navigation=prior.structured(trajectory['navigation_stamps_ns'],trajectory['navigation_poses'])
    metrics,_=evaluation.evaluate(navigation,truth,calibration['imu']['mount'])
    transform=metrics['evaluation_alignment']
    geometry={k:calibration['sonar'][k] for k in ('imu_to_sonar','sonar_to_imu','range_min','range_max','horizontal_fov','vertical_fov')}
    with np.load(ROOT/'detected-features.npz',allow_pickle=False) as file:
        features=dict(file)
    assert len(features['stamps_ns'])==148 and len(features['pixels'])==2554
    assert np.all(np.diff(features['offsets'])<=300)
    np.testing.assert_array_equal(features['nav_stamps_ns'],trajectory['navigation_stamps_ns'])
    np.testing.assert_allclose(features['nav_positions'],trajectory['navigation_poses'][:,:3],atol=1e-12)
    poses={int(t):dict(position=p,attitude=r) for t,p,r in zip(features['nav_stamps_ns'],features['nav_positions'],features['nav_attitudes'])}
    protocol=read('association-protocol.json')['reconstruction']
    summary=read('assessment-summary.json')
    for mode,total,count in [('descriptor',170,168),('projective',59,59)]:
        document=json.loads(gzip.decompress((ROOT/(mode+'-complete.json.gz')).read_bytes()))
        events,tracks=replay(ROOT,mode,calibration_path=ROOT/'calibration.json')
        assert events==document['events']
        repeated={k:v for k,v in tracks.items() if len(v['observations'])>=3}
        assert len(repeated)==len(document['tracks'])==total
        points,errors,sigmas=[],[],[]
        held=[]
        for row in document['tracks']:
            history=repeated[row['track_id']]['observations']
            assert history==row['all_observations'] and row['construction']==history[:8]
            assert len(row['hypotheses'])+row['invalid_seeds']==9
            if not row['hypotheses']:
                assert row['fit_unavailable']
                continue
            covariance=np.diag([protocol['range_sigma_m']**2,protocol['azimuth_sigma_rad']**2]*len(row['construction']))
            assert all(a['cost']<=b['cost'] for a,b in zip(row['hypotheses'],row['hypotheses'][1:]))
            for hypothesis in row['hypotheses']:
                block=linearize_track(observations=row['construction'],states=poses,landmark=hypothesis['point'],measurement_covariance=covariance,**geometry)
                np.testing.assert_allclose(block['residual']@block['residual'],hypothesis['cost'],rtol=1e-9,atol=1e-9)
            best=row['hypotheses'][0]
            points.append(best['point']);errors.append(row['assessment']['surface_error_m'])
            block=linearize_track(observations=row['construction'],states=poses,landmark=best['point'],measurement_covariance=covariance,**geometry)
            singular=np.linalg.svd(block['jacobian'][:,-3:],compute_uv=False)
            if best['local_rank']==3:
                sigmas.append(float(3/singular[-1]))
            unused=[o for o in history if o['stamp_ns']>=row['construction'][-1]['stamp_ns']+2_000_000_000]
            assert unused==row['heldout_measurements']
            actual=heldout_residuals(point=best['point'],observations=unused,states=poses,geometry=geometry,fit_stamps_ns=[o['stamp_ns'] for o in row['construction']])
            assert actual['total']==row['heldout']['total'] and actual['valid_geometry']==row['heldout']['valid_geometry']
            held.extend(actual['observations'])
        assert len(points)==count
        world=np.array(points)@np.array(transform['rotation']).T+transform['translation']
        distance=np.column_stack([prior.surface_distances(world,t) for t in surfaces.values()]).min(axis=1)
        np.testing.assert_allclose(distance,errors,atol=1e-10)
        saved=summary[mode]
        assert saved['point_count']==count and not saved['map_qualified'] and not saved['native_fusion']
        np.testing.assert_allclose(np.sqrt(np.mean(distance**2)),saved['surface_error_m']['rmse'],rtol=1e-10)
        np.testing.assert_allclose(np.median(sigmas),saved['median_fixed_pose_measurement_position_3sigma_m'],rtol=1e-9)
        assert sum(x<=.1 for x in sigmas)==saved['measurement_only_3sigma_at_most_10cm']
        assert len(held)==saved['validation']['observations']
        assert int((distance>.1).sum())==saved['beyond_10cm']
    fine,_=replay(ROOT,'projective',257,calibration_path=ROOT/'calibration.json')
    coarse,_=replay(ROOT,'projective',65,calibration_path=ROOT/'calibration.json')
    changes=[dict(stamp_ns=x['stamp_ns'],pixels=[p for p in set(a)|set(b) if a.get(p)!=b.get(p)]) for x,y,a,b in zip(fine,coarse,origins(fine),origins(coarse)) if a!=b]
    expected=read('support-diagnostics.json')['sampling_control']
    assert changes==expected['changed'] and len(changes)==2 and sum(len(r['pixels']) for r in changes)==3
    assert sum(x['observations']!=y['observations'] for x,y in zip(fine,coarse))==82
    # IDs are bookkeeping: renaming them must not change geometric membership.
    renamed=json.loads(json.dumps(fine))
    for row in renamed:
        for obs in row['observations']:
            obs['track_id']+=10000
    assert origins(fine)==origins(renamed)
    print('PASS: all prepared association histories, 27 native + 168/59 offline points, two unavailable fits, all seeds/future views, true coarse/fine memberships and original metadata. No map accepted.')


if __name__=='__main__':
    main()
