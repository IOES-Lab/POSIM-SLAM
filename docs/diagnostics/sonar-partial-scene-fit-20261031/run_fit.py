# SPDX-License-Identifier: Apache-2.0
"""Sensor-only finite scene inverse fit; truth/RGB absent from this module."""
import argparse
import hashlib
import importlib.util
import json
import time
from pathlib import Path
import numpy as np
import posim_slam
from posim_slam.sonar_scene_fit import SceneHeightFit

ROOT = Path(__file__).resolve().parent
PREVIOUS = 'sonar-height-diversity-fit-20261031'
BASE = 'sonar-finite-geometry-fit-20261031'
FAMILIES = ['initial_multiple_reflectors', 'fitted_partial_scene']


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def write(path, value):
    path.write_text(json.dumps(value, indent=2, allow_nan=False)+'\n')


def helper(records):
    spec = importlib.util.spec_from_file_location('scene_prior_geometry', records/BASE/'fit.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def inputs(records):
    result = json.loads((records/PREVIOUS/'inputs.json').read_text())
    for row in result['frames']:
        path = records/row['raw_record']/'raw'/row['file']
        assert sha(path) == row['raw_sha256']
    return result


def build(records, vertices=None, faces=None, edges=None):
    base = helper(records)
    data = inputs(records)
    frames, views = data['frames'], []
    for frame in frames:
        with np.load(records/frame['raw_record']/'raw'/frame['file'], allow_pickle=False) as z:
            pixels = np.array(frame['pixels'])
            views.append(dict(model=base.make_model(z), origin=np.array(frame['origin']),
                              rotation=np.array(frame['rotation']), pixels=pixels,
                              power=10**(z['intensity_db'][pixels[:,0], pixels[:,1]].astype(float)/10)))
    if vertices is None:
        with np.load(ROOT/'seed.npz', allow_pickle=False) as z:
            vertices, faces, edges = z['vertices'], z['faces'], z['edges']
    train = [view for frame, view in zip(frames, views) if frame['index'] in [27,49,81,88,151]]
    return (SceneHeightFit(vertices, faces, np.full(len(faces), .001), train,
                           edges=edges, regularization=10.),
            SceneHeightFit(vertices, faces, np.full(len(faces), .001), views,
                           edges=edges, regularization=10.), frames)


def prepare(records):
    if (ROOT/'inputs.json').exists():
        raise FileExistsError('preserve original initialization')
    data, base = inputs(records), helper(records)
    previous = json.loads((records/BASE/'inputs.json').read_text())
    plane = np.array(previous['seed']['ground'])
    reference = np.array(previous['seed']['reference_xy'])
    endpoints = []
    for row in data['frames']:
        if row['index'] not in [27,49,81,88,151]:
            continue
        az, el = np.meshgrid(np.linspace(-np.pi/3, np.pi/3, 51), [-np.pi/18, np.pi/18])
        rays = np.column_stack((np.cos(el.ravel())*np.cos(az.ravel()),
                                np.cos(el.ravel())*np.sin(az.ravel()), np.sin(el.ravel())))
        endpoints.extend(np.vstack((np.array(row['origin']),
                         row['origin']+15*rays@np.array(row['rotation']).T))[:, :2])
    endpoints = np.asarray(endpoints)
    lo = 2*np.floor(endpoints.min(axis=0)/2)-2
    hi = 2*np.ceil(endpoints.max(axis=0)/2)+2
    x, y = np.arange(lo[0], hi[0]+1, 2.), np.arange(lo[1], hi[1]+1, 2.)
    xx, yy = np.meshgrid(x, y)
    xy = np.column_stack((xx.ravel(), yy.ravel()))
    vertices = np.column_stack((xy, plane[2]+(xy-reference)@plane[:2]))
    faces = []
    for j in range(len(y)-1):
        for i in range(len(x)-1):
            a = j*len(x)+i
            faces.extend([[a,a+1,a+len(x)+1], [a,a+len(x)+1,a+len(x)]])
    grid_vertices, grid_faces = len(vertices), len(faces)
    with np.load(records/BASE/'seed-candidates.npz', allow_pickle=False) as z:
        points, labels = z['points'], z['labels']
    clusters = []
    for label in np.unique(labels):
        selected = points[labels == label]
        center = np.median(selected[:, :2], axis=0)
        centered = selected[:, :2]-center
        _, vectors = np.linalg.eigh(centered.T@centered)
        v = vectors[:, -1]
        yaw = float((np.arctan2(v[1],v[0])+np.pi/2)%np.pi-np.pi/2)
        c, s = np.cos(yaw), np.sin(yaw)
        rotation = np.array([[c,s],[-s,c]])
        uv = centered@rotation.T
        low, high = uv.min(axis=0), uv.max(axis=0)
        size = np.maximum(high-low,.2)
        center = center+(low+high)/2@rotation
        height = float(np.clip(np.median(selected[:,2]-(plane[2]+(selected[:,:2]-reference)@plane[:2])), .05, 1.))
        seed = dict(ground=plane, reference_xy=reference, center_xy=center)
        parameters = np.r_[np.zeros(5),size,height,yaw,0.]
        _, object_vertices = base.geometry(parameters, seed, True)
        faces.extend((base.FACES+len(vertices)).tolist())
        vertices = np.vstack((vertices, object_vertices))
        clusters.append(dict(label=int(label), candidates=len(selected),
                             parameters=parameters.tolist(), seed={k:np.asarray(v).tolist() for k,v in seed.items()}))
    faces = np.asarray(faces)
    edges = np.unique(np.sort(np.concatenate([faces[:,[0,1]],faces[:,[1,2]],faces[:,[2,0]]]),axis=1), axis=0)
    seeds = dict(vertices=vertices, faces=faces, edges=edges, points=points, labels=labels)
    if (ROOT/'seed.npz').exists():
        with np.load(ROOT/'seed.npz', allow_pickle=False) as saved:
            for key, value in seeds.items():
                np.testing.assert_array_equal(value, saved[key])
    else:
        np.savez_compressed(ROOT/'seed.npz', **seeds)
    write(ROOT/'inputs.json', dict(data, initialization=dict(clusters=clusters,
          grid_vertices=grid_vertices, grid_faces=grid_faces, grid_spacing_m=2.,
          plane_extrapolation_is_prior=True, measured_ground_support=False),
          references={str(p.relative_to(records)):sha(p) for p in [records/PREVIOUS/'inputs.json',
              records/BASE/'inputs.json', records/BASE/'seed-candidates.npz', records/BASE/'fit.py']},
          protocol_sha256=sha(ROOT/'protocol.json')))
    package = Path(posim_slam.__file__).resolve().parent
    write(ROOT/'component-source.json', {n:sha(package/n) for n in ['sonar_scene_fit.py',
          'sonar_source_surface.py','sonar_surface_power.py','sonar_source_normals.py','sonar_speckle.py']})


def run(records):
    if (ROOT/'trace.jsonl').exists():
        raise FileExistsError('preserve every original attempt')
    fit, all_views, frames = build(records)
    start = time.monotonic()
    with (ROOT/'trace.jsonl').open('x') as stream:
        def log(row):
            row = dict(row, elapsed_s=time.monotonic()-start)
            stream.write(json.dumps(row,allow_nan=False)+'\n'); stream.flush()
            print(json.dumps({k:v for k,v in row.items() if k!='heights'}), flush=True)
        fitted = fit.fit(callback=log)
    arrays, report = {}, dict(map_accepted=False, native_fusion=False, fixed_navigation=True,
        truth_scene_rgb_used=False, elapsed_s=time.monotonic()-start, families={})
    for name, z in zip(FAMILIES, [fit.vertices[:,2], fitted['vertices'][:,2]]):
        result = all_views.evaluate(z, moments=True)
        arrays[name+'_vertices'], arrays[name+'_faces'] = result['vertices'], result['faces']
        arrays[name+'_face_ray_counts'] = result['face_ray_counts']
        rows = []
        for frame, row in zip(frames,result['views']):
            prefix = name+f'_frame{frame["index"]}_'
            for key in ['mean','observed','pixels','residual']:
                arrays[prefix+key] = row[key]
            arrays[prefix+'covariance'] = row['moments']['covariance']
            rows.append(dict(index=frame['index'], score=row['score'], gain=row['gain'],
                             normalized_residual_rms=float(np.sqrt(np.mean(row['residual']**2)))))
        report['families'][name] = dict(frames=rows, prior_penalty=result['prior_penalty'],
             training_score=sum(r['score'] for r in rows if r['index'] in [27,49,81,88,151]),
             unused_temporal_score=sum(r['score'] for r in rows if r['index'] in [51,90,153]),
             vertices=len(result['vertices']), faces=len(result['faces']),
             outside_mesh_support='unknown', ray_counts_are_not_physical_identity=True)
    write(ROOT/'fit-output.json',report)
    np.savez_compressed(ROOT/'fit-output.npz', **arrays)
    names = ['protocol.json','run_fit.py','seed.npz','inputs.json','component-source.json',
             'trace.jsonl','fit-output.json','fit-output.npz']
    write(ROOT/'construction-seal.json',dict(truth_scene_rgb_used=False,
          files={n:sha(ROOT/n) for n in names}))
    print(json.dumps(report), flush=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--records', required=True, type=Path)
    parser.add_argument('--prepare', action='store_true')
    args = parser.parse_args()
    prepare(args.records) if args.prepare else run(args.records)
