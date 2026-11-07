# SPDX-License-Identifier: Apache-2.0
"""Post-seal complete scene hypotheses; reference geometry is assessment-only."""
import argparse
import importlib.util
import json
from pathlib import Path
import numpy as np
from run_fit import ROOT, FAMILIES, sha, write


def samples(vertices, faces, grid_vertices, grid_faces):
    weights = np.array([[i/2,j/2,1-(i+j)/2] for i in range(3) for j in range(3-i)])
    points = np.vstack((vertices,np.einsum('ki,tij->tkj',weights,vertices[faces]).reshape(-1,3)))
    ground = np.r_[np.arange(len(vertices)) < grid_vertices,
                   np.repeat(np.arange(len(faces)) < grid_faces, len(weights))]
    return points, ground


def evaluate(records, save=True):
    seal = json.loads((ROOT/'construction-seal.json').read_text())
    for name, digest in seal['files'].items():
        assert sha(ROOT/name) == digest
    path = records/'sonar-inspection-scene-20261031/verify.py'
    spec = importlib.util.spec_from_file_location('scene_surface_distance',path)
    module = importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    scene_path = records/'sonar-inspection-scene-20261031/scene-assessment-only.npz'
    with np.load(scene_path,allow_pickle=False) as z:
        scene = dict(z)
    gauge_path = records/'sonar-translated-shadow-20261031/assessment-summary.json'
    gauge = json.loads(gauge_path.read_text())['navigation']['evaluation_alignment']
    rotation, translation = np.array(gauge['rotation']), np.array(gauge['translation'])
    seed = json.loads((ROOT/'inputs.json').read_text())['initialization']
    def stats(e):
        return dict(points=len(e),rmse=float(np.sqrt(np.mean(e*e))),median=float(np.median(e)),
                    p90=float(np.quantile(e,.9)),maximum=float(e.max()),beyond_10cm=int(sum(e>.1)))
    arrays, report = {}, dict(map_accepted=False,native_fusion=False,
        model_samples_are_not_measured_landmarks=True, prior_ground_is_not_verified=True,
        references={str(p.relative_to(records)):sha(p) for p in [path,scene_path,gauge_path]},
        alignment=gauge,families={})
    with np.load(ROOT/'fit-output.npz',allow_pickle=False) as z:
        for name in FAMILIES:
            points, ground = samples(z[name+'_vertices'],z[name+'_faces'],
                                     seed['grid_vertices'],seed['grid_faces'])
            world = points@rotation.T+translation
            errors = np.column_stack([module.surface_distances(world,t) for t in scene.values()])
            nearest, minimum = errors.argmin(axis=1), errors.min(axis=1)
            report['families'][name] = dict(all_surface_error_m=stats(minimum),
                prior_ground_error_m=stats(minimum[ground]),candidate_object_error_m=stats(minimum[~ground]),
                nearest_surface_counts={k:int(sum(nearest==i)) for i,k in enumerate(scene)})
            for key, value in [('points',points),('world',world),('ground_mask',ground),
                               ('all_surface_error',minimum),('nearest',nearest)]:
                arrays[name+'_'+key] = value
            if save:
                rows = ['ply','format ascii 1.0',
                        'comment Entire conditional finite scene; unmeasured prior and hidden faces retained',
                        f'element vertex {len(points)}','property double x','property double y',
                        'property double z','end_header']
                rows += [' '.join(format(v,'.17g') for v in row) for row in points]
                (ROOT/(name+'-complete.ply')).write_text('\n'.join(rows)+'\n')
            print(name, json.dumps(report['families'][name]),flush=True)
    if save:
        write(ROOT/'assessment.json',report)
        np.savez_compressed(ROOT/'assessed-clouds.npz',**arrays)
    return report, arrays


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--records',required=True,type=Path)
    evaluate(parser.parse_args().records)
