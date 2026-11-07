# SPDX-License-Identifier: Apache-2.0
"""Post-seal reference-scene support ablations; never estimator inputs."""
import argparse
import json
import time
from pathlib import Path
import numpy as np
from posim_slam.sonar_surface_power import TriangleSurface
from posim_slam.sonar_source_surface import SourceSurfaceObservation
from posim_slam.sonar_power_evidence import profile_power_evidence
from run_fit import ROOT, sha, write, build


def evaluate(records):
    seal = json.loads((ROOT/'construction-seal.json').read_text())
    for name,digest in seal['files'].items():assert sha(ROOT/name)==digest
    protocol = json.loads((ROOT/'scene-support-ablation-protocol.json').read_text())
    scene_path = records/'sonar-inspection-scene-20261031/scene-assessment-only.npz'
    gauge_path = records/'sonar-translated-shadow-20261031/assessment-summary.json'
    with np.load(scene_path,allow_pickle=False) as z:surfaces = dict(z)
    gauge = json.loads(gauge_path.read_text())['navigation']['evaluation_alignment']
    align_r,align_t = np.array(gauge['rotation']),np.array(gauge['translation'])
    with np.load(ROOT/'fit-output.npz',allow_pickle=False) as z:
        fitted_plane = z['height_diverse_fit_triangles'][:2]@align_r.T+align_t
    arms = {'reference_ground':surfaces['seabed'],
        'reference_ground_and_target':np.concatenate((surfaces['seabed'],surfaces['inspection_block_0'])),
        'fitted_plane_and_reference_target':np.concatenate((fitted_plane,surfaces['inspection_block_0']))}
    f = build(records)
    arrays,rows = {},[]
    started = time.monotonic()
    for index in protocol['indices']:
        row = next(r for r in f.frames if r['index']==index)
        origin = align_r@row['origin']+align_t
        rotation = align_r@row['rotation']
        pixels,power = np.array(row['pixels']),f.powers[index]
        for label,mesh in arms.items():
            obs = SourceSurfaceObservation(f.models[index],TriangleSurface((mesh-origin)@rotation))
            moments = obs.moments(np.full(len(mesh),.001),pixels)
            score = profile_power_evidence(power,moments['mean'],moments['covariance'])
            prefix = f'{label}_frame{index}_'
            for key,value in [('mean',moments['mean']),('covariance',moments['covariance']),
                              ('residual',score['normalized_residual'])]:arrays[prefix+key] = value
            rows.append(dict(index=index,arm=label,gain=score['gain'],
                composite_score=score['composite_log_score'],
                normalized_residual_rms=float(np.sqrt(np.mean(score['normalized_residual']**2))),
                hit_rays=int(sum(obs.hit_triangles.ravel()>=0)),triangles=len(mesh)))
    report = dict(scope=__doc__,assessment_only=True,adaptive_after_full_scene_audit=True,
        inferred_geometry_unchanged=True,map_accepted=False,native_fusion=False,frames=rows,
        protocol_sha256=sha(ROOT/'scene-support-ablation-protocol.json'),source_sha256=sha(Path(__file__)),
        references={str(p.relative_to(records)):sha(p) for p in [scene_path,gauge_path]},
        elapsed_s=time.monotonic()-started)
    return report,arrays


if __name__=='__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--records',type=Path,default=ROOT.parent)
    args = parser.parse_args()
    if (ROOT/'scene-support-ablation.json').exists():raise FileExistsError('preserve prior ablation')
    report,arrays = evaluate(args.records)
    write(ROOT/'scene-support-ablation.json',report)
    np.savez_compressed(ROOT/'scene-support-ablation.npz',**arrays)
    for row in report['frames']:print(json.dumps(row),flush=True)
