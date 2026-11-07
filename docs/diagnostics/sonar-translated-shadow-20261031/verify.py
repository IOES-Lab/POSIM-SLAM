"""Recompute all conditional observations, assessments and mean-image controls.

Optional --raw-root points to the canonical record/inputs directory and checks
extraction on every original frame. Default portable checks include three raw
frames, every retained region/point and all ideal-model interventions.
"""
from pathlib import Path
import argparse
import gzip
import hashlib
import importlib.util
import json
import lzma
import sys

import numpy as np
import posim_slam
from posim_slam.sonar_surface_power import SurfacePowerModel,TriangleSurface

ROOT=Path(__file__).resolve().parent
COMMON=ROOT.parent/'sonar-shadow-support-20261031'
REFERENCE=ROOT.parent/'sonar-translated-view-20261031'
sys.path.insert(0,str(COMMON))
from dvl_plane import plane_from_ranges
from shadow_extract import extract,plane_background,PROTOCOL
from shadow_geometry import shadow_edge


def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()


def payload(path):
    if path.suffix=='.gz':return gzip.decompress(path.read_bytes())
    if path.suffix=='.xz':return lzma.decompress(path.read_bytes())
    return path.read_bytes()


def read(name):
    path=ROOT/name
    if not path.exists():
        path=next(p for p in [path.with_suffix('.json.gz'),path.with_suffix('.json.xz')] if p.is_file())
    return json.loads(payload(path))


def module(name,path):
    spec=importlib.util.spec_from_file_location(name,path)
    result=importlib.util.module_from_spec(spec);spec.loader.exec_module(result)
    return result


def without_geometry(components):
    result=json.loads(json.dumps(components))
    for c in result:
        for o in c['observations']:o.pop('geometry',None)
    return result


def ply(name):
    lines=(ROOT/name).read_text().splitlines()
    count=int(next(x.split()[-1] for x in lines if x.startswith('element vertex ')))
    points=np.array([[float(x) for x in row.split()] for row in lines[lines.index('end_header')+1:]])
    assert points.shape==(count,3)
    return points


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--raw-root',type=Path)
    args=parser.parse_args()
    for name,digest in read('sha256.json').items():
        path=(ROOT/name).resolve();assert path.is_relative_to(ROOT) and sha(path)==digest,name
    for name,record in read('data-identities.json').items():
        raw=payload(ROOT/record['stored'])
        assert len(raw)==record['uncompressed_bytes'] and hashlib.sha256(raw).hexdigest()==record['uncompressed_sha256'],name
    for name,digest in read('references.json').items():
        path=(ROOT/name).resolve();assert path.is_relative_to(ROOT.parent) and sha(path)==digest,name
    source=Path(posim_slam.__file__).resolve().parent
    for name,digest in read('component-source.json').items():assert sha(source/name)==digest,name
    protocol=read('protocol.json');assert protocol['detector_protocol']==PROTOCOL
    assert protocol['representative_indices']==[48,96,144]
    for name,digest in protocol['public_helper_sha256'].items():
        if name in ('dvl_plane.py','shadow_extract.py','shadow_geometry.py'):assert sha(COMMON/name)==digest,name
    identity=read('source-identity.json')
    assert identity['source_integration']=='ffecee9be9976f68c9e883d6cf3bbd6d40eb7729'
    assert identity['source_prs']==[6,31,32,33,34,35,36] and not identity['source_prs_merged']
    for key in ['construction_uses_scene_truth_rgb','native_estimator_updated','map_accepted','installed_sdk_changed',
                'sensor_defaults_changed','recorded_input_detector_settings_changed','range_plane_pose_uncertainty_available']:
        assert not identity[key],key
    common=module('public_shadow_reproduction',COMMON/'verify.py')
    geometry=module('public_surface_geometry',ROOT.parent/'sonar-inspection-scene-20261031/verify.py')
    assert sha(ROOT.parent/'sonar-inspection-scene-20261031/verify.py')==protocol['public_helper_sha256']['surface_geometry.py']
    project=ROOT.parents[2]
    paths=[project/'tools/evaluate_vio.py',project/'libraries/posim-slam/tools/evaluate_vio.py',source.parents[1]/'tools/evaluate_vio.py']
    evaluation=module('public_trajectory_evaluation',next(p for p in paths if p.is_file()))
    assert sha(Path(evaluation.__file__))==protocol['public_helper_sha256']['evaluate_vio.py']
    with np.load(REFERENCE/'trajectory-assessment.npz',allow_pickle=False) as f:trajectory=dict(f)
    truth=geometry.structured(trajectory['truth_stamps_ns'],trajectory['truth_body_poses'])
    states=evaluation.read(ROOT/'navigation-states.csv')
    assert len(states)==286
    np.testing.assert_array_equal(states['stamp_ns'],trajectory['navigation_stamps_ns'])
    np.testing.assert_array_equal(np.column_stack([states[n] for n in ('x','y','z','qx','qy','qz','qw')]),trajectory['navigation_poses'])
    cal=json.loads((REFERENCE/'calibration.json').read_text())
    metrics,_=evaluation.evaluate(states,truth,cal['imu']['mount'])
    np.testing.assert_allclose(metrics['position_rmse_m'],.04791106141529139,rtol=1e-12)
    r=np.array(metrics['evaluation_alignment']['rotation']);t=np.array(metrics['evaluation_alignment']['translation'])
    poses={int(row['stamp_ns']):(np.array([row[n] for n in ('x','y','z')]),
           common.rotation([row[n] for n in ('qx','qy','qz','qw')])) for row in states}
    provenance=read('input-provenance.json');assert not provenance['construction_truth_or_scene_used']
    assert provenance['calibration']==cal and len(provenance['frames'])==191
    original=read('original-sonar-index.json');assert len(original)==193
    assert provenance['raw_sonar_frames']==original
    dvl=read('dvl-planes.json');assert len(dvl['packets'])==97 and len(dvl['planes'])==96
    assert dvl['packets']==read('original-dvl.json') and not dvl['range_uncertainty_available']
    assert len(dvl['unavailable'])==1 and dvl['unavailable'][0]['reason']=='native_pose_unavailable'
    for original_name,name in [('record/inputs/dvl.json','original-dvl.json'),('record/inputs/sonar.json','original-sonar-index.json')]:
        path=ROOT/name
        if not path.exists():path=ROOT/read('data-identities.json')[name]['stored']
        assert hashlib.sha256(payload(path)).hexdigest()==provenance['source_hashes'][original_name]
    assert sha(ROOT/'navigation-states.csv')==provenance['source_hashes']['replay-verified-navigation/states.csv']
    assert sha(REFERENCE/'calibration.json')==provenance['source_hashes']['record/calibration.json']
    planes={row['stamp_ns']:row for row in dvl['planes']}
    for packet in dvl['packets']:
        if packet['stamp_ns'] not in planes:continue
        actual=plane_from_ranges(packet,cal['dvl']);saved=planes[packet['stamp_ns']]
        assert actual['available'] and not actual['range_variances_available']
        np.testing.assert_allclose(actual['q'],saved['q'],rtol=0,atol=2e-8)
        np.testing.assert_allclose(actual['predicted'],saved['predicted'],rtol=0,atol=2e-8)
    inputs={row['stamp_ns']:row for row in provenance['frames']}
    for row in provenance['frames']:
        stamp=row['stamp_ns'];past=[s for s in planes if 0<=stamp-s<=1_010_000_000]
        assert max(past)==row['plane_stamp_ns']
        pp,pr=poses[max(past)];p,rotation=poses[stamp];plane=planes[max(past)]
        n=pr@np.array(cal['dvl']['estimator']['dvl_to_imu'])@plane['q'];n/=np.linalg.norm(n)
        constant=1/np.linalg.norm(plane['q'])+n@(pp+pr@cal['dvl']['estimator']['imu_to_dvl'])
        origin=p+rotation@cal['sonar']['imu_to_sonar'];orientation=rotation@np.array(cal['sonar']['sonar_to_imu'])
        np.testing.assert_allclose(origin,row['origin'],rtol=0,atol=1e-14)
        np.testing.assert_allclose(orientation,row['rotation'],rtol=0,atol=1e-14)
        np.testing.assert_allclose(orientation.T@n,row['normal_sonar'],rtol=0,atol=1e-14)
        np.testing.assert_allclose(constant-n@origin,row['plane_distance_m'],rtol=0,atol=1e-14)
    originals=json.loads((REFERENCE/'raw-input-hashes.json').read_text())
    representatives=read('representatives.json');assert [r['index'] for r in representatives]==[48,96,144]
    files={original[row['index']]['stamp_ns']:ROOT/row['file'] for row in representatives}
    if args.raw_root:
        files={row['stamp_ns']:args.raw_root/row['file'] for row in original}
    for stamp,path in files.items():assert sha(path)==originals[path.name],str(path)
    with np.load(ROOT.parent/'sonar-inspection-scene-20261031/scene-assessment-only.npz',allow_pickle=False) as f:surfaces=dict(f)
    views={row['stamp_ns']:row for row in json.loads(payload(REFERENCE/'measured-view-assessment.json.gz'))['frames']}
    expected_counts={'angular_median':(9617,1005,1157,60),'physical_plane':(10424,534,1725,47)}
    stats=read('assessment-summary.json')['modes']
    for mode,(count,pair_count,beyond,pair_beyond) in expected_counts.items():
        document=read(mode+'-complete.json')
        assert len(document['frames'])==193 and not document['map_accepted'] and not document['native_fusion']
        assert not document['plane_range_uncertainty_available'] and not document['pose_uncertainty_used']
        rebuilt=[];rebuilt_metadata=[];model=None
        for frame in document['frames']:
            stamp=frame['stamp_ns'];assert frame['raw_sha256']==originals[frame['file']]
            if stamp not in inputs:
                assert not frame['available'] and frame['reason']=='causal_pose_or_plane_unavailable';continue
            inp=inputs[stamp]
            for component in frame['components']:
                for obs in component['observations']:
                    saved=obs['geometry']
                    if component['aperture_truncated'] or component['angular_truncated']:
                        assert not saved['available'] and saved['reason']=='truncated_shadow_region';continue
                    if obs['edge_range_m']>=obs['shadow_end_range_m']:
                        assert not saved['available'] and saved['reason']=='zero_length_dark_run';continue
                    actual=shadow_edge(edge_range_m=obs['edge_range_m'],shadow_end_range_m=obs['shadow_end_range_m'],
                        azimuth_rad=obs['azimuth_rad'],plane_normal=inp['normal_sonar'],plane_distance_m=inp['plane_distance_m'],
                        vertical_fov_rad=cal['sonar']['vertical_fov'])
                    assert actual['available']==saved['available'] and actual['reason']==saved['reason']
                    assert not actual['covariance_available'] and not actual['map_qualified']
                    if actual['available']:
                        common.assert_derived(actual['point'].tolist(),saved['point'])
                        common.assert_derived(actual['height_m'],saved['height_m'])
                        world=np.array(inp['origin'])+np.array(inp['rotation'])@actual['point']
                        common.assert_derived(world.tolist(),saved['point_map'])
                        rebuilt.append(world)
                        rebuilt_metadata.append(dict(stamp_ns=stamp,component=component['component'],beam=obs['beam'],
                            height_m=actual['height_m'],paired=bool(obs['bright_pair_observed'] and obs['recovery_observed'])))
            if stamp in files:
                with np.load(files[stamp],allow_pickle=False) as f:raw=dict(f)
                if model is None:
                    az=np.arctan2(raw['beam_directions'][:,1],raw['beam_directions'][:,0])
                    model=SurfacePowerModel(az,np.linspace(-cal['sonar']['vertical_fov']/2,cal['sonar']['vertical_fov']/2,256),raw['ranges'],maximum_range_m=15.)
                expected=plane_background(model,inp['normal_sonar'],inp['plane_distance_m']) if mode=='physical_plane' else None
                actual=extract(raw,normal=inp['normal_sonar'],distance=inp['plane_distance_m'],
                    vertical_fov=cal['sonar']['vertical_fov'],range_max=15.,expected_power=expected)
                assert actual['available']==frame['available']
                common.assert_derived(actual['components'],without_geometry(frame['components']))
        assert len(rebuilt)==len(document['points'])==count
        for actual,saved in zip(rebuilt_metadata,document['points']):
            common.assert_derived(actual,{k:v for k,v in saved.items() if k!='point_map'})
        local=np.array([p['point_map'] for p in document['points']]);paired=np.array([p['paired'] for p in document['points']],dtype=bool)
        np.testing.assert_allclose(rebuilt,local,rtol=0,atol=2e-13)
        assert int(paired.sum())==pair_count
        world=local@r.T+t;distance=np.empty(len(world));which=np.empty(len(world),dtype=int)
        for start in range(0,len(world),256):
            d=np.column_stack([geometry.surface_distances(world[start:start+256],tri) for tri in surfaces.values()])
            distance[start:start+256]=d.min(axis=1);which[start:start+256]=d.argmin(axis=1)
        np.testing.assert_array_equal(ply(mode+'-all.ply'),world)
        assert int((distance>.1).sum())==beyond and int((distance[paired]>.1).sum())==pair_beyond
        with np.load(ROOT/(mode+'-assessed.npz'),allow_pickle=False) as f:
            np.testing.assert_array_equal(f['world'],world);np.testing.assert_array_equal(f['paired'],paired)
            np.testing.assert_allclose(f['error_m'],distance,rtol=1e-10,atol=1e-12)
            np.testing.assert_array_equal(f['nearest_surface'],which)
        for key,mask in [('all',np.ones(count,dtype=bool)),('paired',paired)]:
            e=distance[mask];summary=stats[mode][key]
            np.testing.assert_allclose(np.sqrt(np.mean(e**2)),summary['error_m']['rmse'],rtol=1e-10)
            assert summary['points']==int(mask.sum())
            stamps=sorted({p['stamp_ns'] for p,keep in zip(document['points'],mask) if keep})
            assert len(stamps)==summary['frames']
            span=np.degrees(np.ptp(np.unwrap([views[s]['viewing_bearing_rad'] for s in stamps])))
            np.testing.assert_allclose(span,summary['assessment_view_span_around_first_block_deg'],rtol=1e-12)
            for index,surface in enumerate(surfaces):
                assert int(((which==index)&mask).sum())==summary['nearest_surface_counts'][surface]
                assert int(((which==index)&mask&(distance<=.1)).sum())==summary['within_10cm_counts'][surface]
        grouped={}
        for index,p in enumerate(document['points']):
            if p['paired']:grouped.setdefault((p['stamp_ns'],p['component']),[]).append(index)
        snapshots=[]
        for frame in document['frames']:
            for component in frame['components']:
                ids=grouped.get((frame['stamp_ns'],component['component']),[])
                if len(ids)<3:continue
                h=np.array([document['points'][i]['height_m'] for i in ids])
                label=np.bincount(which[ids],minlength=len(surfaces)).argmax()
                snapshots.append(dict(stamp_ns=frame['stamp_ns'],component=component['component'],observation_count=len(ids),
                    median_height_m=float(np.median(h)),height_mad_m=float(np.median(abs(h-np.median(h)))),
                    centroid_world=world[ids].mean(axis=0).tolist(),assessment_nearest_surface_majority=list(surfaces)[label],
                    actual_assessment_viewing_bearing_rad=views[frame['stamp_ns']]['viewing_bearing_rad'],physical_identity_verified=False))
        common.assert_derived(snapshots,read(mode+'-component-snapshots.json')['rows'])
        assert len(snapshots)==stats[mode]['eligible_component_snapshots']
        assert not stats[mode]['map_accepted'] and not stats[mode]['cross_frame_identity_verified']
    verify_means(common,geometry,cal)
    print('PASS: 193 original-frame identities; all 20,041 conditional points and complete PLYs; '
          '96 causal DVL planes; 12 exact-edge witnesses; all mean-image/beam/median interventions. '
          +('All 193 raw frames re-extracted.' if args.raw_root else 'Three portable raw frames re-extracted.')+' No map accepted.')


def verify_means(common,geometry,cal):
    oracle=read('oracle-mean-control.json');factorial=read('beam-support-control.json')
    triangles=np.array(oracle['box_triangles']);scene=TriangleSurface(np.concatenate((oracle['plane_triangles'],triangles)))
    assert not oracle['physical_capture'] and not oracle['used_by_recorded_control']
    assert factorial['range_response_changed'] is False and factorial['vertical_ray_count_changed'] is False
    assert not factorial['recorded_pipeline_changed'] and not factorial['native_sensor_changed']
    saved=dict(PROTOCOL);witnesses=0;all_errors=[];pair_errors=[]
    try:
        for case,intervention in zip(oracle['cases'],factorial['cases']):
            assert case['index']==intervention['index']
            with np.load(ROOT/'field'/case['raw_file'],allow_pickle=False) as f:frame=dict(f)
            az=np.arctan2(frame['beam_directions'][:,1],frame['beam_directions'][:,0]);fov=cal['sonar']['vertical_fov']
            model=SurfacePowerModel(az,np.linspace(-fov/2,fov/2,256),frame['ranges'],maximum_range_m=15.)
            physical_mix=model.beam_mix.copy();origin=np.array(case['source_position']);rotation=np.array(case['source_rotation'])
            normal=np.array(case['normal']);H=case['distance'];observation=model.observation(scene,origin,rotation)
            for witness in case['geometric_witness']:
                actual=shadow_edge(edge_range_m=witness['edge_range_m'],shadow_end_range_m=witness['ground_endpoint_range_m'],
                    azimuth_rad=witness['azimuth_rad'],plane_normal=normal,plane_distance_m=H,vertical_fov_rad=fov)
                assert actual['available']==witness['available']
                if actual['available']:
                    np.testing.assert_allclose(actual['height_m'],case['true_height_m'],rtol=0,atol=1e-13)
                    np.testing.assert_allclose(origin+rotation@actual['point'],witness['actual_corner_world'],rtol=0,atol=5e-13)
                    witnesses+=1
            assert len(intervention['modes'])==4
            for control in intervention['modes']:
                model.beam_mix=physical_mix if control['mixing']=='source' else np.diag(physical_mix.sum(axis=1))
                predicted=observation.forward(np.ones(len(scene.triangles)))
                background=plane_background(model,normal,H);intensity=np.full(predicted.shape,-np.inf)
                np.log10(predicted,out=intensity,where=predicted>0);intensity*=10
                PROTOCOL['median_width']=control['median_width']
                result=extract(dict(frame,intensity_db=intensity),normal=normal,distance=H,vertical_fov=fov,range_max=15.,expected_power=background)
                common.assert_derived(result['components'],control['components'])
                assert result['available']==control['available']
                points=[];paired=[];heights=[]
                for component in result['components']:
                    for obs in component['observations']:
                        if component['aperture_truncated'] or component['angular_truncated'] or obs['edge_range_m']>=obs['shadow_end_range_m']:continue
                        actual=shadow_edge(edge_range_m=obs['edge_range_m'],shadow_end_range_m=obs['shadow_end_range_m'],
                            azimuth_rad=obs['azimuth_rad'],plane_normal=normal,plane_distance_m=H,vertical_fov_rad=fov)
                        if actual['available']:
                            points.append(origin+rotation@actual['point']);heights.append(actual['height_m'])
                            paired.append(bool(obs['bright_pair_observed'] and obs['recovery_observed']))
                points=np.array(points).reshape(-1,3);paired=np.array(paired,dtype=bool)
                expected=np.array([p['point_world'] for p in control['points']]).reshape(-1,3)
                np.testing.assert_allclose(points,expected,rtol=0,atol=2e-13)
                errors=geometry.surface_distances(points,triangles) if len(points) else np.array([])
                for key,mask in [('all',np.ones(len(points),dtype=bool)),('paired',paired)]:
                    e=errors[mask];row=control[key];assert row['points']==len(e)
                    if len(e):
                        np.testing.assert_allclose(np.sqrt(np.mean(e**2)),row['surface_rmse_m'],rtol=1e-10)
                        assert int((e>.1).sum())==row['beyond_10cm']
                if control['mixing']=='source' and control['median_width']==5:
                    assert int(paired.sum())==case['paired_points']
                    all_errors.extend(errors);pair_errors.extend(errors[paired])
                    common.assert_derived(result['components'],without_geometry(case['components']))
                    if paired.any():np.testing.assert_allclose(np.median(np.array(heights)[paired]),case['paired_height_median_m'],rtol=1e-12)
    finally:
        PROTOCOL.clear();PROTOCOL.update(saved)
    assert witnesses==12 and len(all_errors)==74 and len(pair_errors)==13
    summary=read('oracle-error-summary.json')['statistics']
    for key,values in [('all',all_errors),('paired',pair_errors)]:
        error=np.array(values);assert len(error)==summary[key]['points']
        np.testing.assert_allclose(np.sqrt(np.mean(error**2)),summary[key]['surface_rmse_m'],rtol=1e-10)
        assert int((error>.1).sum())==summary[key]['beyond_10cm']


if __name__=='__main__':main()
