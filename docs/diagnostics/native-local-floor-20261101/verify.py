"""Recheck original local DVL rows, recorded native ownership and FLS coverage.

Uses the public component and NumPy. This does not restart the native worker,
render a world, infer landmarks or turn recorded transport into map acceptance.
"""
from pathlib import Path
import gzip
import hashlib
import json
import numpy as np

from posim_slam.gazebo_sampling import dvl_beam_rays,cubemap_rays
from posim_slam.inertial import exp_so3,left_jacobian_so3,skew
from posim_slam.native_surface import NativeSurfaceSnapshot
from posim_slam.native_surface_ranges import prepare_native_surface_ranges,linearize_native_surface_ranges
from posim_slam.range_surface_joint import normal_basis,sampled_ray_plane_factor
from posim_slam.surface_prior import Variable
from posim_slam.surface_support import _convex_hull

ROOT=Path(__file__).resolve().parent
digest=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()


def inputs():
    data=json.loads((ROOT.parent/'sampled-dvl-background-20261101/input-ranges.json').read_text())
    source=data['calibration']['bottom_ranges'];axes=np.array([b['axis'] for b in source['beams']])
    sampled=dvl_beam_rays(axes,[b['aperture_rad'] for b in source['beams']],source['resolution_m'],
        reference_to_sensor=np.eye(3),gz_sensors_version='10.1.1',gz_rendering_version='10.0.2')
    return data,axes,sampled


def options(snapshot,data,axes,sampled,start,owned=False):
    packets=data['packets'];training=list(range(start,start+8))
    indices=training+list(range(start-4,start))+list(range(start+8,start+12))
    poses={p['stamp_ns']:p for p in snapshot.recover_modes([packets[k]['stamp_ns'] for k in indices])['poses']}
    mount=np.array(data['calibration']['estimator']['dvl_to_imu'])
    lever=np.array(data['calibration']['estimator']['imu_to_dvl']);observations=[];seeds=[]
    for k in indices:
        packet=packets[k];pose=poses[packet['stamp_ns']]
        for i,b in enumerate(packet['beams']):
            row=dict(model='sampled',stamp_ns=packet['stamp_ns'],sensor_id=0,beam_id=b['id'],
                measured_range=b['range_m'],range_min=data['calibration']['bottom_ranges']['range_min_m'],
                range_max=data['calibration']['bottom_ranges']['range_max_m'],imu_to_sensor=lever,
                sensor_to_imu=mount,beam_axis=axes[i],ray_directions=sampled['beams'][i]['ray_directions'])
            if k in training:
                row.update(association='surface_hypothesis',plane_id=0)
                seeds.append(pose['position']+pose['attitude']@(lever+b['range_m']*mount@axes[i]))
            else:row['role']='support_only'
            observations.append(row)
    plane=dict(support_keys=[(r['stamp_ns'],0,r['beam_id']) for r in observations],max_plane_deviation=.05,max_support_distance=.75)
    if not owned:
        seeds=np.array(seeds);center=seeds.mean(0);_,_,v=np.linalg.svd(seeds-center,full_matrices=False);n=v[-1]
        if n[2]<0:n=-n
        plane.update(initial_normal=n,initial_distance=float(n@center))
    noise=np.eye(len(observations))*data['independent_stddev_m']**2
    for i in range(len(indices)):noise[4*i:4*i+4,4*i:4*i+4]+=data['packet_shared_stddev_m']**2
    return dict(planes={0:plane},observations=observations,measurement_covariance=noise,
        uncertainty_source='Unchanged uncalibrated 20mm independent / 10mm shared diagnostic model')


def range_queries(snapshot,data,axes,sampled):
    result=[];plane=next(v for v in snapshot.order if v.kind=='plane')
    mount=np.array(data['calibration']['estimator']['dvl_to_imu']);lever=np.array(data['calibration']['estimator']['imu_to_dvl'])
    source=data['calibration']['bottom_ranges']
    for k in [28,31,35]:
        stamp=data['packets'][k]['stamp_ns'];ref=snapshot.reference('state',stamp)
        query=snapshot.joint_marginal((Variable('state',stamp,6,ref.reference[:12]),plane))
        delta=query['deltas'];pose=snapshot.recover_modes([stamp])['poses'][0]
        basis=normal_basis(np.array(plane.reference[:3]));tangent=basis@delta[6:8]
        n=exp_so3(tangent)@np.array(plane.reference[:3]);d=plane.reference[3]+delta[8]
        cov=query['observable_covariance'];separate=cov.copy();separate[:6,6:]=0;separate[6:,:6]=0
        for i,b in enumerate(data['packets'][k]['beams']):
            f=sampled_ray_plane_factor(position=pose['position'],attitude=pose['attitude'],imu_to_sensor=lever,
                sensor_to_imu=mount,ray_directions=sampled['beams'][i]['ray_directions'],plane_normal=n,plane_distance=d,
                measured_range=b['range_m'],range_min=source['range_min_m'],range_max=source['range_max_m'])
            j=np.r_[f['pose_jacobian'][:3],f['pose_jacobian'][3:]@pose['attitude'].T@left_jacobian_so3(delta[3:6]),
                f['plane_jacobian'][:3]@left_jacobian_so3(tangent)@basis,f['plane_jacobian'][3]]
            result.append(dict(stamp_ns=stamp,beam_id=b['id'],residual_m=f['residual'],
                correlated_prediction_three_sigma_m=float(3*np.sqrt(max(0.,j@cov@j))),
                cross_blocks_removed_three_sigma_m=float(3*np.sqrt(max(0.,j@separate@j))),
                unbounded_projection_norm=float(np.linalg.norm(j@query['unbounded_basis']))))
    return result


def coverage(snapshot,data,axes,clock):
    sonar=clock['sonar'];stamps=clock['stamps_ns'];needed=sorted(set(stamps+[p['stamp_ns'] for p in data['packets']]))
    poses={}
    for start in range(0,len(needed),64):
        poses.update({p['stamp_ns']:p for p in snapshot.recover_modes(needed[start:start+64])['poses']})
    plane=next(v for v in snapshot.order if v.kind=='plane');basis=normal_basis(np.array(plane.reference[:3]))
    delta=np.array([snapshot._archive.root[('plane',0,i)] for i in range(3)])
    n=exp_so3(basis@delta[:2])@np.array(plane.reference[:3]);d=plane.reference[3]+delta[2]
    projected=normal_basis(n);mount=np.array(data['calibration']['estimator']['dvl_to_imu'])
    lever=np.array(data['calibration']['estimator']['imu_to_dvl']);points=[];local=[]
    for k,packet in enumerate(data['packets']):
        pose=poses[packet['stamp_ns']]
        for i,b in enumerate(packet['beams']):
            point=pose['position']+pose['attitude']@(lever+b['range_m']*mount@axes[i]);points.append(point)
            if 24<=k<40:local.append(point)
    points=np.array(points);local=np.array(local);center=local.mean(0)
    hulls={name:np.array(_convex_hull((p-center)@projected)) for name,p in [('local',local),('whole_record_potential',points)]}
    rays,_=cubemap_rays(np.array(clock['azimuth_float32'],dtype=np.float32),
        np.array(clock['elevation_float32'],dtype=np.float32),clock['cube_size'])
    rays=rays.reshape(len(clock['elevation_float32']),len(clock['azimuth_float32']),3)
    assert hashlib.sha256(np.ascontiguousarray(rays).tobytes()).hexdigest()==clock['sampled_ray_grid_float64_sha256']
    out=[]
    for stamp in stamps:
        pose=poses[stamp];origin=pose['position']+pose['attitude']@np.array(sonar['imu_to_sonar'])
        world=rays@(pose['attitude']@np.array(sonar['sonar_to_imu'])).T
        values=(d-n@origin)/(world@n);valid=(values>=sonar['range_min'])&(values<=sonar['range_max'])
        hits=origin+values[...,None]*world;q=(hits[valid]-center)@projected;by_hull={}
        for name,hull in hulls.items():
            inside=np.ones(len(q),dtype=bool)
            for a,b in zip(hull,np.roll(hull,-1,axis=0)):
                edge=b-a;inside &= edge[0]*(q[:,1]-a[1])-edge[1]*(q[:,0]-a[0]) >= -1e-10
            mask=np.zeros(valid.shape,dtype=bool);mask[valid]=inside
            by_hull[name]=dict(plane_ray_hits=int(valid.sum()),inside_guard_hull=int(mask.sum()),
                fully_inside_beams=int(np.sum(np.all(mask,axis=0))),all_source_rays_return_beams=int(np.sum(np.all(valid,axis=0))))
        out.append(dict(stamp_ns=stamp,coverage=by_hull))
    return out


def main():
    for name,h in json.loads((ROOT/'sha256.json').read_text()).items():assert digest(ROOT/name)==h,name
    expected=json.loads((ROOT/'assessment.json').read_text());data,axes,sampled=inputs()
    before_path=ROOT.parent/'native-initial-bias-20261101/native-snapshot.json.gz'
    assert digest(before_path)==data['snapshot_sha256']
    source_hashes=expected['fls_overlap']['inputs_sha256']
    assert digest(ROOT.parent/'sampled-dvl-background-20261101/input-ranges.json')==source_hashes['input-ranges.json']
    assert digest(ROOT/'native-final-snapshot.json.gz')==expected['final_snapshot']['sha256']
    before=NativeSurfaceSnapshot.read(json.loads(gzip.decompress(before_path.read_bytes())))
    after=NativeSurfaceSnapshot.read(json.loads(gzip.decompress((ROOT/'native-final-snapshot.json.gz').read_bytes())))
    results=[]
    for start in range(4,len(data['packets'])-11,8):
        try:prepared=prepare_native_surface_ranges(before,**options(before,data,axes,sampled,start))
        except ValueError as error:results.append(dict(start=start,accepted=False,reason=str(error)))
        else:results.append(dict(start=start,accepted=True,original_ranges=len(prepared.increment.original_keys)))
    assert results==expected['window_results']
    keys={(p['stamp_ns'],0,b['id']) for p in data['packets'][28:36] for b in p['beams']}
    assert set(after.ledger.recent)==keys and not before.ledger.recent
    assert after.stamp_ns==92500000000 and sum(v.kind=='plane' for v in after.order)==1
    kwargs=options(after,data,axes,sampled,28,owned=True)
    block=linearize_native_surface_ranges(after,**kwargs)
    assert block.diagnostics['all_support_checked']
    try:prepare_native_surface_ranges(after,**kwargs)
    except ValueError as error:assert str(error)==expected['duplicate_readings_rejected']
    else:raise AssertionError('Original readings reused')
    for current,old in zip(range_queries(after,data,axes,sampled),expected['followthrough_range_queries']):
        assert (current['stamp_ns'],current['beam_id'])==(old['stamp_ns'],old['beam_id'])
        for k,v in current.items():
            if k not in ('stamp_ns','beam_id'):np.testing.assert_allclose(v,old[k],rtol=1e-7,atol=1e-9)
    clock=json.loads((ROOT/'sonar-clock-calibration.json').read_text())
    assert clock['original_grid_npz_sha256']==source_hashes['calibration.npz']
    actual=coverage(after,data,axes,clock)
    assert len(actual)==181
    for current,old in zip(actual,expected['fls_overlap']['views']):
        assert current['stamp_ns']==old['stamp_ns'] and current['coverage']==old['coverage']
    assert expected['driver']['replay_complete'] and expected['driver']['native_states']==271
    assert expected['original_ranges_installed']==32 and expected['guard_ranges_installed']==0
    assert expected['workspace_bound_bytes']<=32*2**20 and not expected['map_accepted']
    print(json.dumps(dict(windows=len(results),support_admitted=sum(r['accepted'] for r in results),
        original_native_ranges=len(keys),unused_guard_ranges=32,states=271,
        exact_original_sonar_views=len(actual),local_fully_covered_beam_views=sum(v['coverage']['local']['fully_inside_beams']>0 for v in actual),
        map_accepted=False),indent=2))


if __name__=='__main__':main()
