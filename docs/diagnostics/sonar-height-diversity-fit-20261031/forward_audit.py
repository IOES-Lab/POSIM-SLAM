# SPDX-License-Identifier: Apache-2.0
"""Post-seal, assessment-only scene/pose forward-model audit, not inference."""
import argparse
import json
import time
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation, Slerp
from posim_slam.sonar_surface_power import TriangleSurface
from posim_slam.sonar_source_surface import SourceSurfaceObservation
from posim_slam.sonar_power_evidence import profile_power_evidence
from run_fit import ROOT, BASE, sha, write, build


def truth_pose(stamp,times,poses):
    j = int(np.searchsorted(times,stamp))
    if j<len(times) and times[j]==stamp:
        return poses[j,:3],Rotation.from_quat(poses[j,3:]).as_matrix(),dict(lower_ns=int(times[j]),upper_ns=int(times[j]),fraction=0.)
    if j==0 or j==len(times) or not 0<int(times[j])-int(times[j-1])<=100_000_000:
        raise ValueError('truth_pose_bracket_unavailable')
    u = (int(stamp)-int(times[j-1]))/(int(times[j])-int(times[j-1]))
    p = (1-u)*poses[j-1,:3]+u*poses[j,:3]
    r = Slerp([0.,1.],Rotation.from_quat(poses[j-1:j+1,3:]))([u]).as_matrix()[0]
    return p,r,dict(lower_ns=int(times[j-1]),upper_ns=int(times[j]),fraction=float(u))


def run(records):
    seal = json.loads((ROOT/'construction-seal.json').read_text())
    for name,digest in seal['files'].items():assert sha(ROOT/name)==digest
    if (ROOT/'field-forward-audit.json').exists():raise FileExistsError('keep original forward audit')
    protocol = json.loads((ROOT/'forward-audit-protocol.json').read_text())
    scene_path = records/'sonar-inspection-scene-20261031/scene-assessment-only.npz'
    trajectory_path = records/'sonar-translated-view-20261031/trajectory-assessment.npz'
    calibration_path = records/'sonar-translated-view-20261031/calibration.json'
    gauge_path = records/'sonar-translated-shadow-20261031/assessment-summary.json'
    with np.load(scene_path,allow_pickle=False) as z:surfaces = dict(z)
    mesh = np.concatenate(list(surfaces.values()))
    with np.load(trajectory_path,allow_pickle=False) as z:
        times,poses = z['truth_stamps_ns'],z['truth_body_poses']
    assert np.all(np.diff(times)>0)
    calibration = json.loads(calibration_path.read_text())
    mount = calibration['sonar']['sonar_mount']
    mount_rotation = Rotation.from_quat(mount['rotation']).as_matrix()
    alignment = json.loads(gauge_path.read_text())['navigation']['evaluation_alignment']
    align_r,align_t = np.array(alignment['rotation']),np.array(alignment['translation'])
    f = build(records)
    arrays,results = {},[]
    started = time.monotonic()
    with np.load(ROOT/'fit-output.npz',allow_pickle=False) as fitted:
        for index in protocol['indices']:
            row = next(r for r in f.frames if r['index']==index)
            try:
                body,r,bracket = truth_pose(row['stamp_ns'],times,poses)
            except ValueError as e:
                results.append(dict(index=index,available=False,reason=str(e)));continue
            true_origin = body+r@mount['translation']
            true_rotation = r@mount_rotation
            nav_origin = align_r@row['origin']+align_t
            nav_rotation = align_r@row['rotation']
            pixels = np.array(row['pixels']);power = f.powers[index]
            for label,origin,rotation in [('full_scene_true_pose',true_origin,true_rotation),
                                          ('full_scene_navigation_pose',nav_origin,nav_rotation)]:
                tick = time.monotonic()
                observation = SourceSurfaceObservation(f.models[index],TriangleSurface((mesh-origin)@rotation))
                moments = observation.moments(np.full(len(mesh),.001),pixels)
                score = profile_power_evidence(power,moments['mean'],moments['covariance'])
                prefix = f'{label}_frame{index}_'
                for key,value in [('mean',moments['mean']),('covariance',moments['covariance']),
                    ('observed',power),('residual',score['normalized_residual']),('pixels',pixels),
                    ('ranges',observation.ranges),('normals',observation.normals),
                    ('hit_triangles',observation.hit_triangles),('origin',origin),('rotation',rotation)]:
                    arrays[prefix+key] = value
                counts = [];offset = 0
                for name,triangles in surfaces.items():
                    counts.append(dict(surface=name,triangles=len(triangles),
                        first_hit_rays=int(np.sum((observation.hit_triangles>=offset)&
                                                  (observation.hit_triangles<offset+len(triangles))))))
                    offset += len(triangles)
                result = dict(index=index,arm=label,available=True,gain=score['gain'],
                    composite_score=score['composite_log_score'],
                    normalized_residual_rms=float(np.sqrt(np.mean(score['normalized_residual']**2))),
                    normalized_residual_max=float(np.max(abs(score['normalized_residual']))),
                    first_hits=counts,truth_bracket=bracket,elapsed_s=time.monotonic()-tick)
                results.append(result);print(json.dumps(result),flush=True)
            prefix = f'height_diverse_fit_frame{index}_'
            m,c,p = (fitted[prefix+k] for k in ['mean','covariance','observed'])
            evidence = profile_power_evidence(p,m,c)
            results.append(dict(index=index,arm='sealed_finite_fit_navigation_pose',available=True,
                gain=evidence['gain'],composite_score=evidence['composite_log_score'],
                normalized_residual_rms=float(np.sqrt(np.mean(evidence['normalized_residual']**2))),
                normalized_residual_max=float(np.max(abs(evidence['normalized_residual'])))))
    np.savez_compressed(ROOT/'field-forward-audit.npz',**arrays)
    report = dict(scope=__doc__,assessment_only=True,inferred_geometry_unchanged=True,
        map_accepted=False,native_fusion=False,protocol_sha256=sha(ROOT/'forward-audit-protocol.json'),
        source_sha256=sha(Path(__file__)),full_scene_triangles=len(mesh),frames=results,
        references={str(p.relative_to(records)):sha(p) for p in [scene_path,trajectory_path,calibration_path,gauge_path]},
        elapsed_s=time.monotonic()-started)
    write(ROOT/'field-forward-audit.json',report)
    print(json.dumps(dict(complete=True,elapsed_s=report['elapsed_s'])),flush=True)


if __name__=='__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--records',type=Path,default=ROOT.parent)
    run(parser.parse_args().records)
