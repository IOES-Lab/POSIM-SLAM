"""Predict FLS geometric first ranges from independent DVL-supported patches.

The pose replay must omit all sonar factors. Measured FLS onset ranges never
select a patch or establish support: only the predicted fan/plane intersection
does. A plane is a local hypothesis; unobserved occluders, material response,
beam mixing and an intensity threshold can invalidate a first-return model.
All selected predictions, missing onsets and large errors remain in the report.
"""
import argparse
import csv
import hashlib
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src'))
from evaluate_vio import matrix
from posim_slam.ground_surface_return import plane_returns
from posim_slam.sonar_ground import ground_entries
from posim_slam.surface_support import require_surface_support


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ('record','calibration','states','replay-receipt','spatial-report','output'):
        parser.add_argument('--'+name,type=Path,required=True)
    parser.add_argument('--history-s',type=float,default=30.)
    args=parser.parse_args()
    if not 0<args.history_s<=120:parser.error('history must be in (0,120] seconds')
    receipt=json.loads(args.replay_receipt.read_text())
    if (receipt.get('sonar_factors_omitted') is not True or receipt.get('camera_frames')!=0
            or receipt.get('sonar_track_updates_supplied')!=0):
        raise ValueError('independent_camera_free_no_acoustic_poses_required')
    cal=json.loads(args.calibration.read_text())['sonar']
    spatial=json.loads(args.spatial_report.read_text())
    if spatial['truth_is_estimator_input'] or spatial['online_state_updated']:
        raise ValueError('offline_sensor_only_spatial_hypotheses_required')
    model=args.record/'sensor-model.sdf'
    element=ET.parse(model).getroot().find('.//{http://gazebosim.org}multibeam_sonar')
    rays=int(element.findtext('ray/scan/vertical/rays'))
    low=float(element.findtext('ray/scan/vertical/min_angle'))
    high=float(element.findtext('ray/scan/vertical/max_angle'))
    if not 2<=rays<=512 or not np.isclose(high-low,cal['vertical_fov']):
        raise ValueError('bounded_calibrated_vertical_rays_required')
    elevation=np.linspace(low,high,rays)
    poses={int(row['stamp_ns']):(np.array([float(row[k]) for k in ('x','y','z')]),
        matrix([float(row[k]) for k in ('qx','qy','qz','qw')]))
        for row in csv.DictReader(args.states.open())}
    timeline=json.loads((args.record/'timeline.json').read_text());frames=[]
    for row in timeline['sonar']:
        stamp=row['stamp_ns']
        if stamp not in poses:continue
        with np.load(args.record/row['file'],allow_pickle=False) as raw:
            frame={k:raw[k].copy() for k in ('intensity_db','ranges','beam_directions','stamp')}
        entries=ground_entries(frame);ids=entries['selected_beams']
        angles=entries['azimuth_rad'][ids]
        half=float(np.max(np.diff(entries['azimuth_rad']))/2)
        p,r=poses[stamp];rotation=r@cal['sonar_to_imu'];fans=[]
        for offset in (0.,-half,half):
            az=np.clip(angles+offset,-cal['horizontal_fov']/2,cal['horizontal_fov']/2)
            directions=np.stack(np.broadcast_arrays(np.cos(az)[:,None]*np.cos(elevation),
                np.sin(az)[:,None]*np.cos(elevation),np.sin(elevation)),axis=-1)
            fans.append(directions@rotation.T)
        frames.append(dict(stamp_ns=stamp,ids=ids,origin=p+r@cal['imu_to_sonar'],
            fans=fans,onsets=entries['range_m'][ids]))
    chosen={};considered=0;support_failures={}
    for row in spatial['rows']:
        stamp=row['stamp_ns']
        for beam in row.get('beams',[]):
            if beam['association_rejection'] is not None:continue
            patch=beam['plane_patch'];n=np.array(patch['normal']);d=patch['distance']
            support=np.array(patch['support_points'])
            assert max(patch['support_stamps_ns'])<stamp
            if np.max(abs(support@n-d))>spatial['config']['surface_sigma_m']:continue
            for frame in frames:
                lag=(stamp-frame['stamp_ns'])/1e9
                if abs(lag)>args.history_s:continue
                predictions=[plane_returns(frame['origin'],fan,n,d,
                    range_min=cal['range_min'],range_max=cal['range_max']) for fan in frame['fans']]
                valid=np.logical_and.reduce([p['valid'] for p in predictions])
                for i in np.flatnonzero(valid):
                    considered+=1
                    try:require_surface_support(support,np.array([p['points'][i] for p in predictions]),n,
                        max_distance=spatial['config']['neighborhood_m'])
                    except ValueError as exc:
                        reason=str(exc);support_failures[reason]=support_failures.get(reason,0)+1;continue
                    # Earliest available support; when several patches already
                    # exist at exposure, prefer the latest DVL patch. Ties use
                    # source beam ID, never the observed onset or its error.
                    priority=(max(stamp,frame['stamp_ns']),-stamp,beam['source_id'])
                    key=(frame['stamp_ns'],int(frame['ids'][i]))
                    if key in chosen and chosen[key][0]<=priority:continue
                    measured=frame['onsets'][i];predicted=float(predictions[0]['range_m'][i])
                    observed=bool(np.isfinite(measured))
                    chosen[key]=(priority,dict(sonar_stamp_ns=key[0],beam_id=key[1],dvl_stamp_ns=stamp,
                        patch_beam_id=beam['source_id'],lag_s=lag,available_stamp_ns=priority[0],
                        predicted_range_m=predicted,observed_range_m=float(measured) if observed else None,
                        residual_m=float(measured-predicted) if observed else None,
                        selected_vertical_ray=int(predictions[0]['ray_indices'][i]),
                        predicted_point=predictions[0]['points'][i].tolist()))
    rows=[chosen[key][1] for key in sorted(chosen)]
    errors=np.array([row['residual_m'] for row in rows if row['residual_m'] is not None])
    stats=dict(median=float(np.median(abs(errors))),p90=float(np.percentile(abs(errors),90)),
        maximum=float(abs(errors).max()),signed_median=float(np.median(errors))) if len(errors) else None
    report=dict(scope=__doc__,rows=rows,selected_predictions=len(rows),observed_onsets=len(errors),
        missing_onsets=len(rows)-len(errors),absolute_residual_m=stats,
        onset_range_used_for_prediction_or_support=False,pose_replay_omits_sonar=True,
        selection_policy='earliest availability, then latest DVL timestamp, then beam ID; no measured-onset/error selection',
        model='first geometric plane intersection over calibrated vertical rays; local support checked at centre and two azimuth edges',
        considered_patch_beams=considered,support_failures=support_failures,vertical_rays=rays,
        history_s=args.history_s,truth_is_estimator_input=False,native_state_updated=False,
        physical_identity_verified=False,joint_uncertainty_available=False,map_qualified=False,
        source_sha256={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in
            (Path(__file__),model,args.states,args.replay_receipt,args.calibration,args.spatial_report)})
    args.output.write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps({k:report[k] for k in ('selected_predictions','observed_onsets','missing_onsets','absolute_residual_m','map_qualified')}))


if __name__=='__main__':main()
