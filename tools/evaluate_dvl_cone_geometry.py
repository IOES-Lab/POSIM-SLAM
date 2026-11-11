"""Re-evaluate retained native flat-floor ranges without application imports.

This is a controlled geometric test using the authored floor, not SLAM or a
terrain prior supplied to an estimator. The explicit covariance is a diagnostic
assumption; native bottom range variance is unreported. No simulator is run.
"""
from pathlib import Path
import argparse
import hashlib
import json
import math
import xml.etree.ElementTree as ET

import numpy as np

from posim_slam.dvl_surface import cone_plane_range_factor,whiten_plane_ranges


def evaluate(folder, *, independent_sigma_m, shared_sigma_m):
    if (not math.isfinite(independent_sigma_m) or independent_sigma_m<=0
            or not math.isfinite(shared_sigma_m) or shared_sigma_m<0):
        raise ValueError('explicit_diagnostic_uncertainty_required')
    cases=json.loads((folder/'fixture/native.json').read_text())
    records=[];frames=[]
    for case in cases:
        world=ET.parse(folder/'fixture'/(case['case']+'.sdf')).getroot().find('world')
        model=world.find("model[@name='bluerov2']")
        pose=np.array(list(map(float,model.findtext('pose').split())))
        assert np.array_equal(pose[[0,1,3,4,5]],np.zeros(5))
        bottom=world.find("model[@name='bottom']")
        assert bottom.find('pose') is None
        size=list(map(float,bottom.findtext('.//collision/geometry/box/size').split()))
        floor_height=size[2]/2
        sensor=model.find("link/sensor[@name='practice_dvl']")
        mount=np.array(list(map(float,sensor.findtext('pose').split())))
        assert np.array_equal(mount[3:],np.zeros(3))
        source=sensor.find('{http://gazebosim.org}dvl')
        assert source.findtext('reference_frame')=='0 0 0 0 0 0'
        arrangement=source.find('arrangement');assert arrangement.get('degrees')=='true'
        beams={int(b.get('id')):b for b in arrangement}
        for sample in case['measurements']:
            factors=[]
            for measured in sample['beams']:
                row=dict(case=case['case'],stamp=sample['header']['stamp'],beam_id=measured['id'],locked=measured.get('locked',False))
                if not row['locked']:
                    assert 'range' not in measured
                    records.append(row);continue
                beam=beams[measured['id']]
                angle,tilt,aperture=(math.radians(float(beam.findtext(k))) for k in ('rotation','tilt','aperture'))
                axis=[-math.sin(tilt)*math.cos(angle),math.sin(tilt)*math.sin(angle),-math.cos(tilt)]
                geometry=dict(position=pose[:3],attitude=np.eye(3),imu_to_dvl=mount[:3],dvl_to_imu=np.eye(3),
                    beam_axis=axis,aperture_rad=aperture,plane_normal=[0,0,-1],plane_distance=-floor_height,
                    measured_range=measured['range']['mean'],range_min=float(source.findtext('minimum_range')),
                    range_max=float(source.findtext('maximum_range')))
                continuous=cone_plane_range_factor(**geometry)
                centre=cone_plane_range_factor(**dict(geometry,aperture_rad=0))
                row.update(measured_range_m=geometry['measured_range'],range_variance_reported=measured['range'].get('variance'),
                    continuous_prediction_m=continuous['predicted_range'],continuous_residual_m=continuous['residual'],
                    centre_prediction_m=centre['predicted_range'],centre_residual_m=centre['residual'])
                records.append(row);factors.append(continuous)
            if factors:
                covariance=independent_sigma_m**2*np.eye(len(factors))+shared_sigma_m**2*np.ones((len(factors),len(factors)))
                weighted=whiten_plane_ranges(factors,covariance,uncertainty_source='explicit diagnostic model; uncalibrated')
                frames.append(dict(case=case['case'],stamp=sample['header']['stamp'],
                    conditional_pose_rank=int(np.linalg.matrix_rank(weighted['pose_jacobian'])),
                    horizontal_and_yaw_column_norm=float(np.linalg.norm(weighted['pose_jacobian'][:,[0,1,5]])),
                    all_beams_retained=len(factors)))
    valid=[r for r in records if r['locked']]
    errors=np.array([[r['continuous_residual_m'],r['centre_residual_m']] for r in valid])
    return dict(scope=__doc__,recorded_samples=sum(len(c['measurements']) for c in cases),
        locked_returns=len(valid),unlocked_beams=len(records)-len(valid),rows=records,frames=frames,
        continuous_absolute_error_m=dict(median=float(np.median(abs(errors[:,0]))),maximum=float(abs(errors[:,0]).max())),
        centre_absolute_error_m=dict(median=float(np.median(abs(errors[:,1]))),maximum=float(abs(errors[:,1]).max())),
        uncertainty=dict(independent_sigma_m=independent_sigma_m,shared_sigma_m=shared_sigma_m,calibrated=False),
        full_native_reexecution=False,continuous_is_discrete_renderer=False,truth_is_estimator_input=False,map_qualified=False,
        source_sha256={str(p.relative_to(folder)):hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted((folder/'fixture').iterdir())})


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--record',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--independent-sigma-m',type=float,required=True)
    parser.add_argument('--shared-sigma-m',type=float,required=True)
    args=parser.parse_args()
    result=evaluate(args.record,independent_sigma_m=args.independent_sigma_m,shared_sigma_m=args.shared_sigma_m)
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps({k:result[k] for k in ('recorded_samples','locked_returns','unlocked_beams','continuous_absolute_error_m','centre_absolute_error_m','map_qualified')}))
