# SPDX-License-Identifier: Apache-2.0
"""Source-only reflector discovery and finite-scene inverse; no truth/RGB input."""
import argparse
import gzip
import hashlib
import importlib.util
import json
import time
from pathlib import Path
import numpy as np
import posim_slam
from posim_slam.sonar_birth import _mean_image,unexplained_regions,support_cells,shared_support,choose_surface_birth
from posim_slam.sonar_source_surface import SourceSurfaceObservation
from posim_slam.sonar_surface_power import TriangleSurface
from posim_slam.sonar_scene_fit import SceneHeightFit

ROOT=Path(__file__).resolve().parent
PREVIOUS='sonar-partial-scene-fit-20261031'
TRAIN=[27,49,81,88,151]
UNUSED=[51,90,153]


def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()


def write(path,value):path.write_text(json.dumps(value,indent=2,allow_nan=False)+'\n')


def helper(records):
    path=records/'sonar-finite-geometry-fit-20261031/fit.py'
    spec=importlib.util.spec_from_file_location('public_birth_geometry',path)
    result=importlib.util.module_from_spec(spec);spec.loader.exec_module(result)
    return result


def previous(records):
    source=records/PREVIOUS
    for name,digest in json.loads((source/'construction-seal.json').read_text())['files'].items():
        assert sha(source/name)==digest,name
    data=json.loads((source/'inputs.json').read_text())
    with np.load(source/'fit-output.npz',allow_pickle=False) as z:
        vertices=z['fitted_partial_scene_vertices'].copy();faces=z['fitted_partial_scene_faces'].copy()
    return data,vertices,faces


def raw_view(records,row,pixels=None):
    path=records/row['raw_record']/'raw'/row['file']
    assert sha(path)==row['raw_sha256']
    with np.load(path,allow_pickle=False) as z:
        model=helper(records).make_model(z)
        power=10**(z['intensity_db'].astype(float)/10)
        ranges=z['ranges'].copy()
    view=dict(model=model,origin=np.array(row['origin']),rotation=np.array(row['rotation']))
    if pixels is not None:
        view.update(pixels=np.asarray(pixels,dtype=int),power=power[tuple(np.asarray(pixels).T)])
    return view,power,ranges


def load_scenes():
    metadata=json.loads((ROOT/'scenes.json').read_text())
    with np.load(ROOT/'scenes.npz',allow_pickle=False) as z:
        scenes=[dict(row,vertices=z[f'scene{i}_vertices'],faces=z[f'scene{i}_faces'],
                     reflectivity=z[f'scene{i}_reflectivity']) for i,row in enumerate(metadata)]
    return scenes


def discover(records):
    if (ROOT/'discovery-seal.json').exists() or (ROOT/'proposal-images.npz').exists():
        raise FileExistsError('preserve the original discovery')
    started=time.monotonic()
    data,vertices,faces=previous(records)
    init=data['initialization'];base=helper(records)
    arrays,frames,view_cells,region_scores=[],[],[],[]
    stored={}
    for index in TRAIN:
        row=next(r for r in data['frames'] if r['index']==index)
        view,power,ranges=raw_view(records,row)
        observation=SourceSurfaceObservation(view['model'],TriangleSurface((vertices[faces]-view['origin'])@view['rotation']))
        prediction=_mean_image(observation,np.full(len(faces),.001))
        # Independent exact operator checks span all ranges and all beams.
        check=np.array([(i,j) for i in [0,60,180,300,420,540,598] for j in [0,64,128,192,255]])
        exact=observation.moments(np.full(len(faces),.001),check)['mean']
        error=abs(prediction['mean'][tuple(check.T)]-exact)
        assert np.all(error<=prediction['arithmetic_envelope'][tuple(check.T)])
        regions=unexplained_regions(power,prediction)
        chosen=sorted([r for r in regions['regions'] if r['selected']],key=lambda r:(-r['excess_power'],r['label']))[:8]
        controls=[(i,j) for i in [60,180,360,540] for j in np.rint(np.linspace(16,239,8)).astype(int)]
        cells,weights=[],[]
        selected_rows=[]
        for region in chosen:
            pixels=np.argwhere(regions['labels']==region['label'])
            physical=(ranges[pixels[:,0]]>=.2)&(ranges[pixels[:,0]]<=15.)
            eligible=pixels[physical]
            if len(eligible):
                grid=support_cells(eligible,ranges=ranges,azimuth=view['model'].azimuth_rad,
                    origin=view['origin'],rotation=view['rotation'],vertical_fov=np.pi/9)
                samples=eligible[np.rint(np.linspace(0,len(eligible)-1,4)).astype(int)]
                controls.extend(map(tuple,samples))
            else:grid=np.empty((0,3),dtype=int)
            cells.append(grid);weights.append(region['excess_power']/region['pixels']/regions['gain'])
            selected_rows.append(dict(region,physical_pixels=len(eligible),search_cells=len(grid)))
        pixels=np.unique(np.asarray(controls,dtype=int),axis=0)
        assert len(pixels)<=64
        frames.append(dict(row,pixels=pixels.tolist(),proposal_gain=regions['gain'],
                           all_regions=regions['regions'],birth_regions=selected_rows,
                           exact_query_check_max_absolute_error=float(error.max()),
                           exact_query_check_max_envelope_ratio=float(np.max(error/prediction['arithmetic_envelope'][tuple(check.T)]))))
        view_cells.append(cells);region_scores.append(weights)
        prefix=f'frame{index}_'
        for name,value in [('mean',prediction['mean']),('arithmetic_envelope',prediction['arithmetic_envelope']),
                            ('labels',regions['labels']),('check_pixels',check),('check_exact_mean',exact)]:stored[prefix+name]=value
        for k,grid in enumerate(cells):stored[prefix+f'region{k}_cells']=grid
        print(json.dumps(dict(index=index,all_regions=len(regions['regions']),selected_regions=len(cells),
            cells=sum(map(len,cells)),gain=regions['gain'],elapsed_s=time.monotonic()-started)),flush=True)
    shared=shared_support(view_cells,minimum_views=3)
    score=np.array([sum(region_scores[int(v)][region] for v,regions in membership.items() for region in regions)
                    for membership in shared['memberships']])
    # Ground is only the preceding causal-DVL-mean initialization hypothesis.
    seed=json.loads((records/'sonar-finite-geometry-fit-20261031/inputs.json').read_text())['seed']
    plane=np.array(seed['ground']);reference=np.array(seed['reference_xy'])
    world=(shared['cells']+.5)*.2
    ground=plane[2]+(world[:,:2]-reference)@plane[:2]
    height=world[:,2]-ground
    eligible=shared['selected']&(height>=.075)&(height<=1.5)
    bounds=[(v[:,:2].min(axis=0)-.2,v[:,:2].max(axis=0)+.2) for v in np.split(vertices[init['grid_vertices']:],2)]
    inside=np.zeros(len(world),dtype=bool)
    for lo,hi in bounds:inside|=np.all((world[:,:2]>=lo)&(world[:,:2]<=hi),axis=1)
    order=sorted(np.flatnonzero(eligible&~inside),key=lambda i:(-shared['distinct_views'][i],-score[i],tuple(shared['cells'][i])))
    modes=[]
    for i in order:
        if any(np.linalg.norm(world[i,:2]-np.array(m['centre'])[:2])<.75 for m in modes):continue
        modes.append(dict(cell=shared['cells'][i].tolist(),centre=world[i].tolist(),
                          distinct_views=int(shared['distinct_views'][i]),rank_score=float(score[i]),
                          ground_prior_z=float(ground[i]),memberships=shared['memberships'][i]))
        if len(modes)==3:break
    scenes=[dict(name='unchanged_scene',kind='baseline')]
    scene_arrays=dict(scene0_vertices=vertices,scene0_faces=faces,scene0_reflectivity=np.full(len(faces),.001))
    # First original seed cluster has only frame 88; retain transparent removal
    # as an alternative rather than treating track count as physical support.
    scene_arrays.update(scene1_vertices=vertices,scene1_faces=np.delete(faces,np.arange(init['grid_faces'],init['grid_faces']+12),axis=0),
                        scene1_reflectivity=np.full(len(faces)-12,.001))
    scenes.append(dict(name='without_single_view_seed',kind='remove_unconfirmed_seed'))
    for mode_index,mode in enumerate(modes):
        x,y,z=mode['centre'];xy=np.array([[-.2,-.2],[.2,-.2],[.2,.2],[-.2,.2]])+[x,y]
        bottoms=plane[2]+(xy-reference)@plane[:2]
        for offset in [-.1,0.,.1]:
            h=float(np.clip(z+offset-mode['ground_prior_z'],.05,1.5))
            bottom=np.column_stack((xy,bottoms));box=np.vstack((bottom,bottom+[0.,0.,h]))
            i=len(scenes)
            scene_arrays.update({f'scene{i}_vertices':np.vstack((vertices,box)),
                f'scene{i}_faces':np.vstack((faces,base.FACES+len(vertices))),
                f'scene{i}_reflectivity':np.full(len(faces)+12,.001)})
            scenes.append(dict(name=f'birth{mode_index}_height{offset:+.1f}',kind='finite_box_birth',
                               mode=mode_index,height_offset_m=offset,ground_prior_height_m=h))
    np.savez_compressed(ROOT/'scenes.npz',**scene_arrays)
    write(ROOT/'scenes.json',scenes)
    write(ROOT/'inputs.json',dict(frames=frames,unused_frames=[r for r in data['frames'] if r['index'] in UNUSED],
        modes=modes,baseline_initialization=init,truth_scene_rgb_used=False,unused_raw_power_opened=False,
        references={str(p.relative_to(records)):sha(p) for p in [records/PREVIOUS/'inputs.json',
            records/PREVIOUS/'fit-output.npz',records/PREVIOUS/'construction-seal.json',
            records/'sonar-finite-geometry-fit-20261031/fit.py',records/'sonar-finite-geometry-fit-20261031/inputs.json']}))
    stored.update(shared_cells=shared['cells'],distinct_views=shared['distinct_views'],shared_score=score,
                  eligible=eligible,inside_existing=inside)
    np.savez_compressed(ROOT/'proposal-images.npz',**stored)
    (ROOT/'memberships.json.gz').write_bytes(gzip.compress(json.dumps(shared['memberships'],separators=(',',':')).encode(),mtime=0))
    package=Path(posim_slam.__file__).resolve().parent
    write(ROOT/'component-source.json',{n:sha(package/n) for n in ['sonar_birth.py','sonar_scene_fit.py',
        'sonar_surface_power.py','sonar_source_surface.py','sonar_source_normals.py','sonar_speckle.py']})
    names=['protocol.json','run_birth.py','inputs.json','scenes.json','scenes.npz','proposal-images.npz','memberships.json.gz','component-source.json']
    write(ROOT/'discovery-seal.json',dict(truth_scene_rgb_used=False,unused_power_used=False,
                                        files={n:sha(ROOT/n) for n in names}))
    print(json.dumps(dict(discovery_complete=True,shared_cells=len(world),three_view_cells=int(sum(shared['selected'])),
        eligible_novel_cells=int(sum(eligible&~inside)),modes=modes,scenes=len(scenes),elapsed_s=time.monotonic()-started)),flush=True)


def train(records):
    if (ROOT/'fit-output.json').exists():raise FileExistsError('preserve original birth selection')
    for name,digest in json.loads((ROOT/'discovery-seal.json').read_text())['files'].items():assert sha(ROOT/name)==digest
    data=json.loads((ROOT/'inputs.json').read_text());scenes=load_scenes()
    views=[raw_view(records,row,row['pixels'])[0] for row in data['frames']]
    started=time.monotonic()
    baseline=scenes[0]
    inverse=choose_surface_birth(baseline['vertices'],baseline['faces'],baseline['reflectivity'],views,scenes[1:])
    arrays,rows={},[]
    for i,alternative in enumerate(inverse['alternatives']):
        result=alternative['result']
        frames=[]
        for frame,value in zip(data['frames'],result['views']):
            prefix=f'scene{i}_frame{frame["index"]}_'
            for key in ['mean','observed','pixels','residual']:arrays[prefix+key]=value[key]
            frames.append(dict(index=frame['index'],score=value['score'],gain=value['gain'],
                               residual_rms=float(np.sqrt(np.mean(value['residual']**2)))))
        arrays[f'scene{i}_face_ray_counts']=result['face_ray_counts']
        rows.append(dict(index=i,name=alternative['name'],score=alternative['score'],frames=frames))
    selected=inverse['proposed_index']
    for i in sorted({0,selected}):
        scene=scenes[i]
        fit=SceneHeightFit(scene['vertices'],scene['faces'],scene['reflectivity'],views)
        result=fit.evaluate(fit.vertices[:,2],moments=True)
        for frame,value in zip(data['frames'],result['views']):arrays[f'scene{i}_frame{frame["index"]}_covariance']=value['moments']['covariance']
    np.savez_compressed(ROOT/'fit-output.npz',**arrays)
    write(ROOT/'fit-output.json',dict(selected_index=selected,alternatives=rows,training_only_selection=True,
        map_accepted=False,native_update=False,truth_scene_rgb_used=False,elapsed_s=time.monotonic()-started))
    names=['discovery-seal.json','fit-output.json','fit-output.npz']
    write(ROOT/'construction-seal.json',dict(truth_scene_rgb_used=False,unused_power_used=False,
        files={n:sha(ROOT/n) for n in names}))
    print(json.dumps(dict(selected_index=selected,score=rows[selected]['score'],baseline_score=rows[0]['score'],
                         scenes=len(scenes),elapsed_s=time.monotonic()-started)),flush=True)


def unused(records):
    if (ROOT/'unused-output.json').exists():raise FileExistsError('preserve original unused predictions')
    for name,digest in json.loads((ROOT/'construction-seal.json').read_text())['files'].items():assert sha(ROOT/name)==digest
    data=json.loads((ROOT/'inputs.json').read_text());scenes=load_scenes()
    selected=json.loads((ROOT/'fit-output.json').read_text())['selected_index']
    boxes=[scene['vertices'][-8:] for scene in scenes if scene['kind']=='finite_box_birth' and scene['height_offset_m']==0.]
    views,frames=[],[]
    for row in data['unused_frames']:
        # Query geometry is frozen training geometry; no unused power is read
        # until these coordinates are constructed from original header grids.
        previous_row=next(r for r in json.loads((records/PREVIOUS/'inputs.json').read_text())['frames'] if r['index']==row['index'])
        pixels=list(map(tuple,previous_row['pixels']))
        raw_path=records/row['raw_record']/'raw'/row['file'];assert sha(raw_path)==row['raw_sha256']
        with np.load(raw_path,allow_pickle=False) as z:
            ranges=z['ranges'];az=np.arctan2(z['beam_directions'][:,1],z['beam_directions'][:,0])
        for box in boxes:
            local=(box-np.array(row['origin']))@np.array(row['rotation'])
            distance=np.linalg.norm(local,axis=1);angle=np.arctan2(local[:,1],local[:,0])
            ri=np.rint(np.interp([distance.min(),np.median(distance),distance.max()],ranges,np.arange(len(ranges)))).astype(int)
            bi=np.rint(np.interp([angle.min(),np.median(angle),angle.max()],az,np.arange(len(az)))).astype(int)
            pixels.extend([(ri[1],bi[1]),(ri[0],bi[1]),(ri[2],bi[1]),(ri[1],bi[0]),(ri[1],bi[2])])
        pixels=np.unique(np.array(pixels,dtype=int),axis=0);assert len(pixels)<=63
        views.append(raw_view(records,row,pixels)[0]);frames.append(dict(row,pixels=pixels.tolist()))
    arrays,results={},[]
    for i in sorted({0,selected}):
        scene=scenes[i];fit=SceneHeightFit(scene['vertices'],scene['faces'],scene['reflectivity'],views)
        result=fit.evaluate(fit.vertices[:,2],moments=True)
        values=[]
        for frame,row in zip(frames,result['views']):
            prefix=f'scene{i}_frame{frame["index"]}_'
            for key in ['mean','observed','pixels','residual']:arrays[prefix+key]=row[key]
            arrays[prefix+'covariance']=row['moments']['covariance']
            values.append(dict(index=frame['index'],score=row['score'],gain=row['gain'],
                               residual_rms=float(np.sqrt(np.mean(row['residual']**2)))))
        results.append(dict(index=i,score=result['score'],frames=values))
    np.savez_compressed(ROOT/'unused-output.npz',**arrays)
    write(ROOT/'unused-output.json',dict(selected_index_fixed_from_training=selected,
        unused_views_cannot_select=True,frames=frames,results=results,map_accepted=False,native_update=False))
    print(json.dumps(results),flush=True)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--records',required=True,type=Path)
    parser.add_argument('phase',choices=['discover','train','unused']);args=parser.parse_args()
    {'discover':discover,'train':train,'unused':unused}[args.phase](args.records)
