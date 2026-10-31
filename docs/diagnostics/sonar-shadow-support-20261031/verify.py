"""Reproduce all conditional outputs; raw extraction is checked on three frames."""
from pathlib import Path
import gzip
import hashlib
import importlib.util
import json
import numpy as np
from dvl_plane import plane_from_ranges
from shadow_geometry import shadow_edge
from shadow_extract import extract,plane_background
from posim_slam.sonar_surface_power import SurfacePowerModel

ROOT=Path(__file__).resolve().parent


def rotation(q):
    x,y,z,w=np.asarray(q)/np.linalg.norm(q)
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                     [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                     [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])


def persistence(document,provenance):
    rows=[]
    for frame in document['frames']:
        for component in frame['components']:
            obs=[o for o in component['observations'] if o['bright_pair_observed']
                 and o['recovery_observed'] and o['geometry']['available']]
            if len(obs)>=3:
                rows.append(dict(stamp_ns=frame['stamp_ns'],component=component['component'],count=len(obs),
                    height_m=float(np.median([o['geometry']['height_m'] for o in obs])),
                    centroid=np.mean([o['geometry']['point_map'] for o in obs],axis=0).tolist()))
    split=len(rows)//2;cut=rows[split-1]['stamp_ns'];train=rows[:split]
    held=[r for r in rows[split:] if r['stamp_ns']>=cut+2_000_000_000]
    height=float(np.median([r['height_m'] for r in train]))
    residual=np.array([r['height_m']-height for r in held])
    poses={r['stamp_ns']:r for r in provenance['frames']}
    origin=np.array([poses[r['stamp_ns']]['origin'] for r in rows]);target=np.mean([r['centroid'] for r in train],axis=0)
    bearing=np.unwrap(np.arctan2(origin[:,1]-target[1],origin[:,0]-target[0]))
    return dict(training=train,unused=held,all_eligible_components=rows,
         predicted_height_m=height,unused_height_rmse_m=float(np.sqrt(np.mean(residual**2))),
         unused_denominator=len(rows)-split,unused_count=len(held),
         target_viewing_bearing_span_deg=float(np.rad2deg(np.ptp(bearing))),origin_axis_span_m=np.ptp(origin,axis=0).tolist())


def main():
    read=lambda name:json.loads((ROOT/name).read_text())
    for name,digest in read('sha256.json').items():
        assert hashlib.sha256((ROOT/name).read_bytes()).hexdigest()==digest,name
    inputs=read('inputs.json');references=read('references.json');provenance=read('input-provenance.json')
    assert not inputs['map_qualified'] and not inputs['native_fusion'] and inputs['range_uncertainty_unknown']
    assert not inputs['pose_uncertainty_used'] and not inputs['truth_or_scene_used_for_construction']
    assert inputs['source_integration']=='ffecee9be9976f68c9e883d6cf3bbd6d40eb7729'
    table=np.genfromtxt(ROOT/'navigation-states.csv',delimiter=',',names=True)
    poses={int(row['stamp_ns']):(np.array([row[n] for n in ('x','y','z')]),
        rotation([row[n] for n in ('qx','qy','qz','qw')])) for row in table}
    assert len(poses)==223
    cal=inputs['calibration'];planes={p['stamp_ns']:p for p in inputs['planes']}
    for packet in inputs['dvl_packets']:
        stamp=packet['stamp_ns']
        if stamp not in poses:continue
        fitted=plane_from_ranges(packet,cal['dvl']);stored=planes[stamp]
        assert fitted['available'] and not fitted['range_variances_available']
        # This optimizer mean is checked at convergence precision. Exact saved
        # plane observations remain the input to every construction below.
        np.testing.assert_allclose(fitted['q'],stored['q'],rtol=0,atol=2e-8)
        np.testing.assert_allclose(fitted['predicted'],stored['predicted'],rtol=0,atol=2e-8)
    assert len(planes)==74 and len(inputs['dvl_packets'])==75
    frame_inputs={r['stamp_ns']:r for r in inputs['frames']}
    for row in provenance['frames']:
        stamp=row['stamp_ns'];frame=frame_inputs[stamp];p,r=poses[stamp]
        past=[s for s in planes if 0<=stamp-s<=1_010_000_000]
        assert max(past)==frame['plane_stamp_ns']
        plane=planes[max(past)];pp,pr=poses[max(past)]
        n=pr@np.array(cal['dvl']['estimator']['dvl_to_imu'])@plane['q'];n/=np.linalg.norm(n)
        distance=1/np.linalg.norm(plane['q'])+n@(pp+pr@cal['dvl']['estimator']['imu_to_dvl'])
        rs=r@np.array(cal['sonar']['sonar_to_imu']);origin=p+r@cal['sonar']['imu_to_sonar']
        np.testing.assert_allclose(row['plane_normal'],rs.T@n,atol=2e-14)
        np.testing.assert_allclose(row['plane_distance_m'],distance-n@origin,atol=2e-14)
        np.testing.assert_array_equal(row['origin'],origin)
    assert len(provenance['frames'])==147 and len(inputs['raw_sonar_index'])==150
    provenance_by_stamp={r['stamp_ns']:r for r in provenance['frames']}
    documents={}
    point_counts={'angular_median':3911,'physical_plane':1428}
    paired_counts={'angular_median':547,'physical_plane':325}
    for mode in point_counts:
        with gzip.open(ROOT/(mode+'-complete.json.gz'),'rt') as f:document=json.load(f)
        documents[mode]=document;rebuilt=[]
        assert len(document['frames'])==150
        for frame in document['frames']:
            if not frame['available']:continue
            inp=provenance_by_stamp[frame['stamp_ns']]
            for c in frame['components']:
                for obs in c['observations']:
                    stored=obs['geometry']
                    if c['aperture_truncated'] or c['angular_truncated']:
                        assert not stored['available'] and stored['reason']=='truncated_shadow_region';continue
                    if obs['edge_range_m']>=obs['shadow_end_range_m']:
                        assert not stored['available'] and stored['reason']=='zero_length_dark_run';continue
                    result=shadow_edge(edge_range_m=obs['edge_range_m'],shadow_end_range_m=obs['shadow_end_range_m'],
                        azimuth_rad=obs['azimuth_rad'],plane_normal=inp['plane_normal'],plane_distance_m=inp['plane_distance_m'],
                        vertical_fov_rad=cal['sonar']['vertical_fov'])
                    assert result['available']==stored['available'] and result['reason']==stored['reason']
                    assert not result['covariance_available'] and not result['map_qualified']
                    if result['available']:
                        np.testing.assert_allclose(result['point'],stored['point'],rtol=0,atol=2e-13)
                        np.testing.assert_allclose(result['height_m'],stored['height_m'],rtol=0,atol=2e-14)
                        point=np.array(inp['origin'])+np.array(inp['rotation'])@result['point']
                        np.testing.assert_allclose(point,stored['point_map'],rtol=0,atol=2e-13)
                        rebuilt.append(point)
        assert len(rebuilt)==len(document['points'])==point_counts[mode]
        assert sum(p['paired'] for p in document['points'])==paired_counts[mode]
        np.testing.assert_allclose(rebuilt,[p['point_map'] for p in document['points']],rtol=0,atol=2e-13)
    model=None
    for example in references['representative_raw']:
        path=ROOT/example['path'];assert hashlib.sha256(path.read_bytes()).hexdigest()==example['sha256']
        with np.load(path,allow_pickle=False) as z:frame=dict(z)
        source=inputs['frames'][example['index']];stamp=source['stamp_ns']
        if model is None:
            az=np.arctan2(frame['beam_directions'][:,1],frame['beam_directions'][:,0])
            model=SurfacePowerModel(az,np.linspace(-cal['sonar']['vertical_fov']/2,cal['sonar']['vertical_fov']/2,256),
                                   frame['ranges'],maximum_range_m=15.)
        predicted=plane_background(model,source['normal_sonar'],source['plane_distance_m'])
        for mode in point_counts:
            current=extract(frame,normal=source['normal_sonar'],distance=source['plane_distance_m'],
                 vertical_fov=cal['sonar']['vertical_fov'],range_max=15.,
                 expected_power=predicted if mode=='physical_plane' else None)
            stored=documents[mode]['frames'][example['index']]
            assert current['available']==stored['available']
            # Ignore attached geometry when comparing raw extracted observations.
            for c in stored['components']:
                for obs in c['observations']:obs.pop('geometry',None)
            assert current['components']==stored['components'],(mode,stamp)
    # Scene/truth enter only this separate all-point assessment section.
    assert hashlib.sha256((ROOT/references['scene']).read_bytes()).hexdigest()==references['scene_sha256']
    spec=importlib.util.spec_from_file_location('surface_distance_check',ROOT.parent/'sonar-inspection-scene-20261031/verify.py')
    assessment=importlib.util.module_from_spec(spec);spec.loader.exec_module(assessment)
    summary=read('assessment-summary.json');alignment=summary['alignment']
    with np.load(ROOT/references['scene'],allow_pickle=False) as z:scene=dict(z)
    for mode,document in documents.items():
        points=np.array([r['point_map'] for r in document['points']])
        world=points@np.array(alignment['rotation']).T+alignment['translation']
        distances=np.empty((len(world),len(scene)))
        for start in range(0,len(world),256):
            distances[start:start+256]=np.column_stack([assessment.surface_distances(world[start:start+256],t) for t in scene.values()])
        errors=distances.min(axis=1)
        with np.load(ROOT/(mode+'-assessed.npz'),allow_pickle=False) as z:
            np.testing.assert_allclose(world,z['world'],atol=2e-13)
            np.testing.assert_allclose(errors,z['error_m'],atol=2e-12)
        for name,mask in [('all',np.ones(len(world),dtype='bool')),('paired',np.array([r['paired'] for r in document['points']]))]:
            saved=summary['results'][mode][name]
            assert saved['count']==int(mask.sum()) and saved['beyond_10cm']==int((errors[mask]>.1).sum())
            np.testing.assert_allclose(np.sqrt(np.mean(errors[mask]**2)),saved['surface_rmse_m'],atol=2e-12)
        ply=(ROOT/(mode+'-all.ply')).read_text().split('end_header\n',1)[1]
        np.testing.assert_array_equal(np.array([[float(v) for v in row.split()] for row in ply.splitlines()]),world)
    # Restore complete geometry after representative comparison removed it.
    with gzip.open(ROOT/'physical_plane-complete.json.gz','rt') as f:physical=json.load(f)
    original=read('height-persistence.json');result=persistence(physical,provenance)
    for key in ['predicted_height_m','unused_height_rmse_m','target_viewing_bearing_span_deg','origin_axis_span_m']:
        np.testing.assert_allclose(result[key],original[key],atol=2e-12)
    assert len(result['training'])==result['unused_count']==10
    print('PASS: 74 causal DVL planes, 150 frame classifications, all 3911/1428 conditional outputs and full PLY exports, three raw-image extractions, all-point assessment and narrow-view height persistence; no map qualification')


if __name__=='__main__':main()
