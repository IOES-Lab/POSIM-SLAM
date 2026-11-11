# SPDX-License-Identifier: Apache-2.0
"""Post-seal complete surface assessment and shared recorded-camera plotting."""
import argparse
import importlib.util
import json
import sys
from pathlib import Path
import numpy as np
from run_birth import ROOT,PREVIOUS,sha,write,load_scenes

FAMILIES=['initial_multiple_reflectors','fitted_partial_scene']


def module(name,path):
    spec=importlib.util.spec_from_file_location(name,path)
    result=importlib.util.module_from_spec(spec);spec.loader.exec_module(result);return result


def points(vertices,faces,grid_vertices,grid_faces,original_vertices,original_faces):
    used=np.unique(faces)
    weights=np.array([[i/2,j/2,1-(i+j)/2] for i in range(3) for j in range(3-i)])
    result=np.vstack((vertices[used],np.einsum('ki,tij->tkj',weights,vertices[faces]).reshape(-1,3)))
    # The removal alternative preserves order before/after the deleted faces;
    # classify faces by their vertex IDs, not a shifted array index.
    face_ground=np.all(faces<grid_vertices,axis=1)
    face_born=np.any(faces>=original_vertices,axis=1)
    ground=np.r_[used<grid_vertices,np.repeat(face_ground,len(weights))]
    born=np.r_[used>=original_vertices,np.repeat(face_born,len(weights))]
    return result,ground,born


def evaluate(records,save=True):
    seal=json.loads((ROOT/'construction-seal.json').read_text())
    for name,digest in seal['files'].items():assert sha(ROOT/name)==digest
    data=json.loads((ROOT/'inputs.json').read_text())
    selected=json.loads((ROOT/'fit-output.json').read_text())['selected_index']
    scenes=load_scenes()
    geometry_path=records/'sonar-inspection-scene-20261031/verify.py'
    geometry=module('birth_assessment_geometry',geometry_path)
    scene_path=records/'sonar-inspection-scene-20261031/scene-assessment-only.npz'
    with np.load(scene_path,allow_pickle=False) as z:reference=dict(z)
    gauge_path=records/'sonar-translated-shadow-20261031/assessment-summary.json'
    gauge=json.loads(gauge_path.read_text())['navigation']['evaluation_alignment']
    rotation,translation=np.array(gauge['rotation']),np.array(gauge['translation'])
    init=data['baseline_initialization']
    def stats(e):
        if not len(e):return dict(available=False,points=0)
        return dict(available=True,points=len(e),rmse=float(np.sqrt(np.mean(e*e))),
            median=float(np.median(e)),p90=float(np.quantile(e,.9)),maximum=float(e.max()),beyond_10cm=int(sum(e>.1)))
    arrays={};report=dict(map_accepted=False,native_update=False,training_only_selection=selected,
        model_samples_are_not_measured_landmarks=True,prior_ground_is_not_verified=True,
        alignment=gauge,references={str(p.relative_to(records)):sha(p) for p in [geometry_path,scene_path,gauge_path]},families={})
    for name,index in zip(FAMILIES,[0,selected]):
        candidate=scenes[index]
        sample,ground,born=points(candidate['vertices'],candidate['faces'],init['grid_vertices'],init['grid_faces'],
                                  len(scenes[0]['vertices']),len(scenes[0]['faces']))
        world=sample@rotation.T+translation
        errors=np.column_stack([geometry.surface_distances(world,t) for t in reference.values()])
        nearest,minimum=errors.argmin(axis=1),errors.min(axis=1)
        report['families'][name]=dict(all_surface_error_m=stats(minimum),prior_ground_error_m=stats(minimum[ground]),
            candidate_object_error_m=stats(minimum[~ground]),existing_object_error_m=stats(minimum[~ground&~born]),
            born_object_error_m=stats(minimum[born]),
            nearest_surface_counts={k:int(sum(nearest==i)) for i,k in enumerate(reference)})
        for key,value in [('points',sample),('world',world),('ground_mask',ground),('born_mask',born),
                           ('all_surface_error',minimum),('nearest',nearest)]:arrays[name+'_'+key]=value
        if save:
            rows=['ply','format ascii 1.0','comment Entire finite scene hypothesis; no accepted map',
                  f'element vertex {len(sample)}','property double x','property double y','property double z','end_header']
            rows+=[' '.join(format(v,'.17g') for v in row) for row in sample]
            (ROOT/(name+'-complete.ply')).write_text('\n'.join(rows)+'\n')
        print(name,json.dumps(report['families'][name]),flush=True)
    if save:
        write(ROOT/'assessment.json',report);np.savez_compressed(ROOT/'assessed-clouds.npz',**arrays)
    return report,arrays


def render(records):
    source=records/PREVIOUS/'render.py'
    sys.path.insert(0,str(source.parent))
    painter=module('shared_birth_scene_plot',source)
    painter.main(records,root=ROOT,families=FAMILIES,
        labels=['Unchanged finite scene','Training-selected reflector birth'],
        title='Source-only multiview reflector birth hypotheses\nNo accepted SLAM cloud or acoustic pose update')
    write(ROOT/'plot-helper.json',dict(path=str(source.relative_to(records)),sha256=sha(source),
                                     default_output_compatibility='plot-default-compatibility.json'))


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--records',required=True,type=Path)
    parser.add_argument('--render',action='store_true');args=parser.parse_args()
    render(args.records) if args.render else evaluate(args.records)
