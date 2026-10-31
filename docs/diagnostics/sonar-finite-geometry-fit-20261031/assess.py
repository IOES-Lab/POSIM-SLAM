# SPDX-License-Identifier: Apache-2.0
"""Assessment only, executed after the immutable construction seal exists."""
import importlib.util
import json
from pathlib import Path
import numpy as np
from fit import FACES, ROOT, sha, write


def samples(vertices):
    # Every face retains the same fixed 8-division barycentric grid. Duplicates
    # on edges remain. These are model surface samples, never measured landmarks.
    weights = np.array([[i/8,j/8,1-(i+j)/8] for i in range(9) for j in range(9-i)])
    return np.vstack((vertices,np.einsum('ki,tij->tkj',weights,vertices[FACES]).reshape(-1,3)))


def evaluate(records, save=True, families=None):
    seal = json.loads((ROOT/'construction-seal.json').read_text())
    for name,digest in seal['files'].items():assert sha(ROOT/name)==digest
    geometry_path = records/'sonar-inspection-scene-20261031/verify.py'
    spec = importlib.util.spec_from_file_location('public_assessment_geometry',geometry_path)
    module = importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    scene_path = records/'sonar-inspection-scene-20261031/scene-assessment-only.npz'
    with np.load(scene_path,allow_pickle=False) as z:scene = dict(z)
    alignment_path = records/'sonar-translated-shadow-20261031/assessment-summary.json'
    reference = json.loads(alignment_path.read_text())
    align = reference['navigation']['evaluation_alignment']
    rotation,translation = np.array(align['rotation']),np.array(align['translation'])
    report = dict(map_accepted=False,native_fusion=False,construction_was_sealed=True,
                  geometry_is_restrictive_one_prism_hypothesis=True,
                  points_are_model_surface_samples_not_measured_landmarks=True,
                  alignment=align,diagnostic_10cm_not_universal_acceptance=True,families={},
                  references={str(p.relative_to(records)):sha(p) for p in [geometry_path,scene_path,alignment_path]})
    arrays = {}
    with np.load(ROOT/'fit-output.npz',allow_pickle=False) as z:
        for name in ['initial_prism','fitted_prism'] if families is None else families:
            points = samples(z[name+'_vertices'])
            world = points@rotation.T+translation
            errors = np.column_stack([module.surface_distances(world,triangles) for triangles in scene.values()])
            nearest = np.argmin(errors,axis=1)
            all_errors = errors.min(axis=1)
            targets = {k:np.linalg.norm(v.mean(axis=(0,1))-world.mean(axis=0)) for k,v in scene.items() if k!='seabed'}
            target = min(targets,key=targets.get)
            target_error = errors[:,list(scene).index(target)]
            def stats(a):
                return dict(rmse=float(np.sqrt(np.mean(a*a))),median=float(np.median(a)),
                            p90=float(np.quantile(a,.9)),maximum=float(a.max()),beyond_10cm=int(sum(a>.1)))
            report['families'][name] = dict(points=len(points),all_surface_error_m=stats(all_errors),
                nearest_surface_counts={k:int(sum(nearest==i)) for i,k in enumerate(scene)},
                assessment_nearest_center_object=target,target_surface_error_m=stats(target_error))
            for key,value in [('points',points),('world',world),('all_surface_error',all_errors),
                              ('target_surface_error',target_error),('nearest',nearest)]:arrays[name+'_'+key]=value
            if save:
                rows=['ply','format ascii 1.0','comment Conditional model surface samples; map unqualified',
                      f'element vertex {len(points)}','property double x','property double y','property double z','end_header']
                rows += [' '.join(format(v,'.17g') for v in row) for row in points]
                (ROOT/(name+'-complete.ply')).write_text('\n'.join(rows)+'\n')
    if save:
        write(ROOT/'assessment.json',report)
        np.savez_compressed(ROOT/'assessed-clouds.npz',**arrays)
    return report,arrays


if __name__=='__main__':
    import argparse
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--records',required=True,type=Path)
    args = parser.parse_args()
    print(json.dumps(evaluate(args.records)[0],indent=2))
