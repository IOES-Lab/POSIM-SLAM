"""Offline overlap of conditional DVL planes and measured FLS ground onsets.

Uses fixed estimated poses and a recorded spatial-plane diagnostic. Earlier
sonar frames can be revisited when DVL support arrives, or earlier DVL support
can be compared with later sonar. This tests geometry, not physical echo/plane
identity. Five range/azimuth bin samples do not bound the entire uncertain cell.
No simulator truth, terrain prior or native estimator update is accepted.
"""
import argparse
import csv
import hashlib
import json
from pathlib import Path
import sys

import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src'))
from evaluate_vio import matrix
from posim_slam.sonar_ground import ground_entries
from posim_slam.sonar_plane_intersection import range_azimuth_plane
from posim_slam.surface_support import require_surface_support


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ('record','states','calibration','spatial-report','output'):
        parser.add_argument('--'+name,type=Path,required=True)
    parser.add_argument('--history-s',type=float,default=30.)
    args=parser.parse_args()
    if not 0<args.history_s<=120:parser.error('history must be in (0,120] seconds')
    timeline=json.loads((args.record/'timeline.json').read_text())
    calibration=json.loads(args.calibration.read_text())['sonar']
    spatial=json.loads(args.spatial_report.read_text())
    if spatial['truth_is_estimator_input'] or spatial['online_state_updated']:
        raise ValueError('offline_estimated_pose_diagnostic_required')
    poses={int(row['stamp_ns']):(np.array([float(row[k]) for k in ('x','y','z')]),
        matrix([float(row[k]) for k in ('qx','qy','qz','qw')]))
        for row in csv.DictReader(args.states.open())}
    frames=[];sources={}
    for row in timeline['sonar']:
        t=row['stamp_ns']
        if t not in poses:continue
        path=args.record/row['file'];sources[row['file']]=hashlib.sha256(path.read_bytes()).hexdigest()
        with np.load(path,allow_pickle=False) as stored:
            frame={k:stored[k].copy() for k in ('intensity_db','ranges','beam_directions','stamp')}
        entries=ground_entries(frame);ids=entries['selected_beams']
        ids=ids[np.isfinite(entries['range_m'][ids])]
        azimuths=entries['azimuth_rad'][ids];ranges=entries['range_m'][ids]
        dr=float(np.max(np.diff(frame['ranges']))/2)
        da=float(np.max(np.diff(entries['azimuth_rad']))/2)
        p,r=poses[t];origin=p+r@calibration['imu_to_sonar']
        # Centre plus four sampled bin corners. These are numerical cells,
        # not calibrated confidence intervals or a full-cell support proof.
        sampled_ranges=np.column_stack((ranges,ranges-dr,ranges-dr,ranges+dr,ranges+dr))
        sampled_angles=np.column_stack((azimuths,azimuths-da,azimuths+da,azimuths-da,azimuths+da))
        frames.append(dict(stamp_ns=t,ids=ids,origin=origin,rotation=r@calibration['sonar_to_imu'],
            ranges=sampled_ranges,angles=sampled_angles))
    patches=[];identifiers={};events=[]
    groups=('contemporaneous','sonar_first','dvl_first')
    totals={g:dict(patch_frame_pairs=0,onsets_tested=0,unique_centre=0,
        patch_pairs_with_support_sample_in_fan=0,
        centre_supported=0,five_samples_supported=0,strict_dvl_supported=0,
        minimum_onset_to_support_sample_m=None) for g in groups}
    for row in spatial['rows']:
        t=row['stamp_ns']
        for beam in row.get('beams',[]):
            patch=beam.get('plane_patch')
            if patch is None:continue
            n=np.array(patch['normal']);d=patch['distance'];support=np.array(patch['support_points'])
            # All supporting ranges precede this DVL sample; their poses are
            # held fixed. This is not a causal joint-state covariance claim.
            assert max(patch['support_stamps_ns'])<t
            strict=beam['association_rejection'] is None
            key=f'{t}:{beam["source_id"]}';patch_summary=dict(id=key,strict_dvl_support=strict,
                dvl_rejection=beam['association_rejection'],matches={g:0 for g in groups})
            patches.append(patch_summary)
            for frame in frames:
                lag=(t-frame['stamp_ns'])/1e9
                if abs(lag)>args.history_s:continue
                group='contemporaneous' if abs(lag)<=.51 else ('sonar_first' if lag>0 else 'dvl_first')
                total=totals[group];total['patch_frame_pairs']+=1
                total['onsets_tested']+=len(frame['ids'])
                local=(support-frame['origin'])@frame['rotation']
                distances=np.linalg.norm(local,axis=1)
                azimuth=np.arctan2(local[:,1],local[:,0])
                elevation=np.arctan2(local[:,2],np.linalg.norm(local[:,:2],axis=1))
                visible=(abs(azimuth)<=calibration['horizontal_fov']/2)&(abs(elevation)<=calibration['vertical_fov']/2)
                visible&=(distances>=calibration['range_min'])&(distances<=calibration['range_max'])
                total['patch_pairs_with_support_sample_in_fan']+=int(visible.any())
                if not len(frame['ids']):continue
                intersections=range_azimuth_plane(ranges=frame['ranges'].ravel(),azimuths=frame['angles'].ravel(),
                    origin=frame['origin'],sonar_to_map=frame['rotation'],plane_normal=n,plane_distance=d,
                    vertical_fov_rad=calibration['vertical_fov'])
                counts=intersections['count'].reshape(-1,5)
                points=intersections['points'][:,0].reshape(-1,5,3)
                for i in np.flatnonzero(counts[:,0]==1):
                    total['unique_centre']+=1
                    distance=float(np.min(np.linalg.norm(support-points[i,0],axis=1)))
                    best=total['minimum_onset_to_support_sample_m']
                    total['minimum_onset_to_support_sample_m']=distance if best is None else min(best,distance)
                    if np.max(abs(support@n-d))>spatial['config']['surface_sigma_m']:continue
                    try:require_surface_support(support,points[i,:1],n,max_distance=spatial['config']['neighborhood_m'])
                    except ValueError:continue
                    total['centre_supported']+=1
                    if not (counts[i]==1).all():continue
                    try:require_surface_support(support,points[i],n,max_distance=spatial['config']['neighborhood_m'])
                    except ValueError:continue
                    total['five_samples_supported']+=1
                    total['strict_dvl_supported']+=int(strict)
                    patch_summary['matches'][group]+=1
                    observation=f'{frame["stamp_ns"]}:{int(frame["ids"][i])}'
                    identifiers.setdefault(group,set()).add(observation)
                    events.append(dict(patch=key,sonar_stamp_ns=frame['stamp_ns'],beam_id=int(frame['ids'][i]),
                        lag_s=lag,group=group,strict_dvl_support=strict,conditional_point=points[i,0].tolist()))
    for group,total in totals.items():
        total['distinct_sonar_onsets']=len(identifiers.get(group,set()))
        total['patches_with_overlap']=sum(p['matches'][group]>0 for p in patches)
    report=dict(scope=__doc__,history_s=args.history_s,pose_covered_sonar_frames=len(frames),
        all_ground_onsets=sum(len(f['ids']) for f in frames),patch_count=len(patches),
        strict_dvl_patch_count=sum(p['strict_dvl_support'] for p in patches),totals=totals,patches=patches,events=events,
        truth_is_estimator_input=False,native_state_updated=False,map_qualified=False,
        physical_identity_verified=False,joint_pose_plane_uncertainty_available=False,
        bin_support_is_sampled_not_certified=True,ground_onset_is_not_identified_seabed=True,
        source_sha256={str(p):hashlib.sha256(p.read_bytes()).hexdigest()
            for p in (Path(__file__),args.states,args.calibration,args.spatial_report)},raw_sonar_sha256=sources)
    args.output.write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps({k:report[k] for k in ('pose_covered_sonar_frames','all_ground_onsets',
        'patch_count','strict_dvl_patch_count','totals','map_qualified')}))


if __name__=='__main__':main()
