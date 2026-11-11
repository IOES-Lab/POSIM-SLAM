"""Replay original DVL ranges against the shared native posterior; no map admission."""
from pathlib import Path
import argparse
import copy
import gzip
import hashlib
import json
import numpy as np

from posim_slam.constrained_information import constraint_support
from posim_slam.gazebo_sampling import dvl_beam_rays
from posim_slam.inertial import exp_so3
from posim_slam.native_surface import NativeSurfaceSnapshot
from posim_slam.native_surface_ranges import prepare_native_surface_ranges
from posim_slam.range_surface_joint import normal_basis,sampled_ray_plane_factor
from posim_slam.retired_pose_archive import coordinates
from posim_slam.surface_prior import Variable
from posim_slam.surface_support import _convex_hull

ROOT=Path(__file__).resolve().parent
digest=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()


def replay():
    data=json.loads((ROOT/'input-ranges.json').read_text())
    path=ROOT.parent/'native-initial-bias-20261101/native-snapshot.json.gz'
    assert digest(path)==data['snapshot_sha256']
    snapshot=NativeSurfaceSnapshot.read(json.loads(gzip.decompress(path.read_bytes())))
    assert not snapshot.ledger.recent and snapshot._archive.max_recovery_bytes==32*2**20
    calibration=data['calibration'];source=calibration['bottom_ranges']
    assert source['reference_to_sensor']==[0.,0.,0.,1.] and source['range_variance_m2'] is None
    beams=source['beams'];axes=np.array([b['axis'] for b in beams]);packets=data['packets']
    sampled=dvl_beam_rays(axes,[b['aperture_rad'] for b in beams],source['resolution_m'],
        reference_to_sensor=np.eye(3),gz_sensors_version='10.1.1',gz_rendering_version='10.0.2')
    train=[packets[i] for i in data['training_packet_indices']];stamps=[p['stamp_ns'] for p in train]
    assert len(train)==8 and len(packets)==83
    try:snapshot.recover(stamps)
    except ValueError as error:old_rejection=str(error)
    else:raise AssertionError('Original full-recovery capacity rejection changed')
    assert old_rejection=='conditional_recovery_workspace_capacity'
    modes=snapshot.recover_modes(stamps)
    query=tuple(Variable('state',t,6,snapshot.reference('state',t).reference[:12]) for t in stamps)
    joint=snapshot.joint_marginal(query)
    np.testing.assert_allclose(modes['deltas'],joint['deltas'],rtol=1e-10,atol=1e-10)
    assert not joint['live_coordinates_treated_as_known'] and not modes['uncertainty_computed']
    poses={p['stamp_ns']:p for p in modes['poses']}
    mount=np.array(calibration['estimator']['dvl_to_imu']);lever=np.array(calibration['estimator']['imu_to_dvl'])

    def seeds_for(pose_values):
        return np.array([pose_values[p['stamp_ns']]['position']+pose_values[p['stamp_ns']]['attitude']@
            (lever+b['range_m']*mount@axes[i]) for p in train for i,b in enumerate(p['beams'])])

    seeds=seeds_for(poses);center=seeds.mean(0);_,_,v=np.linalg.svd(seeds-center,full_matrices=False)
    normal=v[-1]
    if normal[2]<0:normal=-normal
    distance=float(normal@center);observations=[]
    for packet in train:
        for index,b in enumerate(packet['beams']):
            assert b['id']==beams[index]['id'] and b['locked'] and b['variance_m2'] is None
            observations.append(dict(model='sampled',stamp_ns=packet['stamp_ns'],sensor_id=0,beam_id=b['id'],plane_id=0,
                association='surface_hypothesis',measured_range=b['range_m'],range_min=source['range_min_m'],range_max=source['range_max_m'],
                imu_to_sensor=lever,sensor_to_imu=mount,beam_axis=axes[index],ray_directions=sampled['beams'][index]['ray_directions']))
    covariance=np.eye(32)*data['independent_stddev_m']**2
    for i in range(8):covariance[4*i:4*i+4,4*i:4*i+4]+=data['packet_shared_stddev_m']**2
    kwargs=dict(planes={0:dict(initial_normal=normal,initial_distance=distance,
        support_keys=[(o['stamp_ns'],0,o['beam_id']) for o in observations],max_plane_deviation=.05,max_support_distance=.75)},
        observations=observations,measurement_covariance=covariance,
        uncertainty_source='Frozen independent 20mm / packet-shared 10mm diagnostic model, uncalibrated')
    try:prepare_native_surface_ranges(snapshot,**kwargs)
    except ValueError as error:strict=dict(passed=False,reason=str(error))
    else:raise AssertionError('Recorded unsupported background was admitted')
    candidate=prepare_native_surface_ranges(snapshot,**kwargs,require_support=False)
    update=candidate.increment._likelihood;rows=update.new_live_rows;info=snapshot._information
    old_keys=coordinates(snapshot.order);old_delta=np.array([snapshot._archive.root[k] for k in old_keys])
    columns=rows.jacobian.shape[1];extra=columns-len(old_keys)
    j=np.vstack((np.pad(info.jacobian,((0,0),(0,extra))),rows.jacobian))
    b=np.r_[info.residual-info.jacobian@old_delta,rows.offset]
    c=np.pad(info.constraint_jacobian,((0,0),(0,extra)))
    point,z,_=constraint_support(c,info.constraint_jacobian@old_delta-info.constraint_residual,columns)
    u,s,vt=np.linalg.svd(j@z,full_matrices=True);rank=int(np.sum(s>s[0]*1e-10))
    noise=z@(vt[:rank].T/s[:rank]);free=z@vt[rank:].T
    mode=point-noise@(u[:,:rank].T@(b+j@point));cov=noise@noise.T
    archive=copy.copy(update.archive);archive.root=dict(zip(coordinates(rows.order),mode));archive._memo={}
    plane=rows.order[-1];assert plane.kind=='plane'
    nref=np.array(plane.reference[:3]);n=exp_so3(normal_basis(nref)@mode[-3:-1])@nref;d=plane.reference[3]+mode[-1]
    results=[];training_poses={};support_hits=[]
    for packet in packets:
        pose=archive.recover_modes([packet['stamp_ns']])['poses'][0]
        if packet['stamp_ns'] in stamps:training_poses[packet['stamp_ns']]=pose
        for i,measured in enumerate(packet['beams']):
            value=sampled_ray_plane_factor(position=pose['position'],attitude=pose['attitude'],imu_to_sensor=lever,sensor_to_imu=mount,
                ray_directions=sampled['beams'][i]['ray_directions'],plane_normal=n,plane_distance=d,
                measured_range=measured['range_m'],range_min=source['range_min_m'],range_max=source['range_max_m'])
            training=packet['stamp_ns'] in stamps
            results.append(dict(stamp_ns=packet['stamp_ns'],beam_id=measured['id'],training=training,residual_m=value['residual']))
            if training:support_hits.extend(value['conditional_support_hits'])
    seeds=seeds_for(training_poses);hits=np.array(support_hits);basis=normal_basis(n);origin=seeds.mean(0)
    p=(seeds-origin)@basis;q=(hits-origin)@basis;hull=np.array(_convex_hull(p))
    margins=[]
    for a,following in zip(hull,np.roll(hull,-1,axis=0)):
        edge=following-a
        margins.append((edge[0]*(q[:,1]-a[1])-edge[1]*(q[:,0]-a[0]))/np.linalg.norm(edge))
    margin=np.min(margins,axis=0);nearest=np.sqrt(((q[:,None]-p)**2).sum(2).min(1))
    def errors(training):
        r=np.array([x['residual_m'] for x in results if x['training']==training])
        return dict(count=len(r),rms_m=float(np.sqrt(np.mean(r*r))),maximum_absolute_m=float(abs(r).max()))
    summary=dict(schema='posim-slam-sampled-dvl-background-1',training=errors(True),unused=errors(False),strict_support=strict,
        support=dict(calibrated_ray_hits=len(hits),outside_nominal_hull=int(np.sum(margin<-1e-10)),
            maximum_extrapolation_m=float(max(0.,-margin.min())),maximum_nearest_seed_distance_m=float(nearest.max()),
            maximum_plane_deviation_m=float(abs(seeds@n-d).max())),
        old_full_recovery_rejection=old_rejection,mode_query_workspace_bound_bytes=modes['workspace_bound_bytes'],
        joint_query_workspace_bound_bytes=joint['workspace_bound_bytes'],likelihood_workspace_bound_bytes=update.workspace_bound_bytes,
        original_range_rows=32,new_likelihood_rows=len(rows.jacobian),changed_conditionals=len(update.changed_conditionals),
        root_rank=rank,root_columns=columns,unbounded_columns=free.shape[1],plane_normal=n.tolist(),plane_distance_m=float(d),
        initial_seed=candidate.diagnostics['initialization'],original_native_information_used_once=True,
        source_exactness=sampled['source_exactness'],noise_calibrated=False,native_owner_updated=False,map_accepted=False,range_cases=results)
    arrays=dict(mode=mode,covariance=cov,unbounded_basis=free,seed_projection=p,ray_projection=q,hull=hull,outside_hull=margin<-1e-10)
    return summary,arrays


def main(record=False):
    if not record:
        for name,checksum in json.loads((ROOT/'sha256.json').read_text()).items():assert digest(ROOT/name)==checksum,name
    result,arrays=replay()
    if record:
        assert not (ROOT/'assessment.json').exists()
        (ROOT/'assessment.json').write_text(json.dumps(result,indent=2)+'\n')
        np.savez_compressed(ROOT/'expected.npz',**arrays)
    else:
        expected=json.loads((ROOT/'assessment.json').read_text())
        for key in ('strict_support','support','training','unused','original_range_rows','new_likelihood_rows','changed_conditionals','root_rank','root_columns'):
            if isinstance(result[key],dict):
                for field,value in result[key].items():
                    if isinstance(value,(float,int)) and not isinstance(value,bool):np.testing.assert_allclose(value,expected[key][field],rtol=1e-7,atol=1e-10)
                    else:assert value==expected[key][field]
            else:assert result[key]==expected[key]
        np.testing.assert_allclose([r['residual_m'] for r in result['range_cases']],
            [r['residual_m'] for r in expected['range_cases']],rtol=1e-7,atol=1e-9)
        with np.load(ROOT/'expected.npz',allow_pickle=False) as old:
            for name,value in arrays.items():np.testing.assert_allclose(value,old[name],rtol=1e-7,atol=1e-8)
        assert result['likelihood_workspace_bound_bytes']<=32*2**20 and not result['map_accepted']
    print(json.dumps({k:result[k] for k in ('training','unused','strict_support','support','mode_query_workspace_bound_bytes','joint_query_workspace_bound_bytes','likelihood_workspace_bound_bytes','changed_conditionals')},indent=2))


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--record',action='store_true')
    main(parser.parse_args().record)
