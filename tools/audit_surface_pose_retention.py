"""Audit pose dependencies at each DVL/FLS association's availability.

Use the first published native state at or after availability. Include both
measurement poses and EVERY original DVL support timestamp. This diagnoses the
published retention policy; it does not inspect intermediate owner transactions,
establish echo identity, or replace old poses with frozen Cartesian support.
"""
import argparse
import bisect
import gzip
import hashlib
import json
from pathlib import Path


def audit(predictions,spatial,events):
    if (predictions.get('pose_replay_omits_sonar') is not True
            or predictions.get('onset_range_used_for_prediction_or_support') is not False
            or spatial.get('truth_is_estimator_input') is not False
            or spatial.get('online_state_updated') is not False):
        raise ValueError('independent_sensor_only_diagnostics_required')
    patches={}
    for row in spatial['rows']:
        for beam in row.get('beams',[]):
            if beam['association_rejection'] is not None:continue
            key=(row['stamp_ns'],beam['source_id']);support=beam['plane_patch']['support_stamps_ns']
            if (key in patches or not support or any(type(t) is not int or not 0<t<key[0] for t in support)):
                raise ValueError('unique_patch_and_original_prior_support_stamps_required')
            patches[key]=set(support)
    states=[row for row in events if row.get('type')=='state']
    times=[row['stamp_ns'] for row in states]
    if any(type(t) is not int or t<=0 for t in times) or times!=sorted(set(times)):
        raise ValueError('ordered_native_state_snapshots_required')
    for row in states:
        retained=row['sonar']['retained_pose_stamps_ns']
        if (any(type(t) is not int or t<=0 or t>row['stamp_ns'] for t in retained)
                or len(set(retained))!=len(retained)):
            raise ValueError('explicit_unique_retained_pose_stamps_required')
    rows=[];seen=set();counts=dict(predictions=0,with_published_state=0,
        both_endpoint_poses_retained=0,all_source_poses_retained=0,no_published_state=0)
    for prediction in predictions['rows']:
        sonar=prediction['sonar_stamp_ns'];dvl=prediction['dvl_stamp_ns'];available=prediction['available_stamp_ns']
        key=(sonar,prediction['beam_id']);patch=(dvl,prediction['patch_beam_id'])
        if (key in seen or patch not in patches or available!=max(sonar,dvl)
                or any(type(t) is not int or t<=0 for t in (sonar,dvl,available))):
            raise ValueError('original_unique_prediction_and_patch_identity_required')
        seen.add(key);counts['predictions']+=1
        required={sonar,dvl}|patches[patch];index=bisect.bisect_left(times,available)
        if index==len(times):
            counts['no_published_state']+=1
            rows.append(dict(sonar_stamp_ns=sonar,beam_id=key[1],dvl_stamp_ns=dvl,
                patch_beam_id=patch[1],available_stamp_ns=available,required_pose_stamps_ns=sorted(required),
                snapshot_stamp_ns=None,missing_pose_stamps_ns=None,all_source_poses_retained=None))
            continue
        state=states[index];retained=set(state['sonar']['retained_pose_stamps_ns']);missing=required-retained
        endpoints={sonar,dvl}<=retained;counts['with_published_state']+=1
        counts['both_endpoint_poses_retained']+=int(endpoints);counts['all_source_poses_retained']+=int(not missing)
        rows.append(dict(sonar_stamp_ns=sonar,beam_id=key[1],dvl_stamp_ns=dvl,patch_beam_id=patch[1],
            available_stamp_ns=available,snapshot_stamp_ns=state['stamp_ns'],
            required_pose_stamps_ns=sorted(required),missing_pose_stamps_ns=sorted(missing),
            both_endpoint_poses_retained=endpoints,all_source_poses_retained=not missing))
    return dict(scope=__doc__,counts=counts,rows=rows,truth_is_estimator_input=False,
        native_state_updated=False,intermediate_owner_transaction_observed=False,
        raw_support_beam_ids_available=False,map_qualified=False,
        next_requirement='Keep correlated surface variables through proper marginalization or admit complete resident source groups; never substitute a fitted-plane independent prior')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ('predictions','spatial','events','output'):parser.add_argument('--'+name,type=Path,required=True)
    args=parser.parse_args();inputs=(args.predictions,args.spatial,args.events)
    def read(path):
        return json.loads(gzip.decompress(path.read_bytes()) if path.suffix=='.gz' else path.read_bytes())
    report=audit(*(read(path) for path in inputs))
    report['source_sha256']={str(path):hashlib.sha256(path.read_bytes()).hexdigest() for path in (Path(__file__),*inputs)}
    args.output.write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report['counts']))


if __name__=='__main__':main()
