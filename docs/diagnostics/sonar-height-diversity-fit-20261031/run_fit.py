# SPDX-License-Identifier: Apache-2.0
"""Compare identical finite-surface search schedules with extra height views.

Reuse the public observation/search implementation, not private acquisition or
assessment geometry. No native pose update, sensor change or map admission.
"""
import argparse
import gzip
import hashlib
import importlib
import json
import sys
import time
from pathlib import Path
import numpy as np

ROOT = Path(__file__).resolve().parent
BASE = 'sonar-finite-geometry-fit-20261031'
FAMILIES = ['initial_prism','three_view_control','height_diverse_fit']


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def write(path, value):
    path.write_text(json.dumps(value,indent=2,allow_nan=False)+'\n')


def helpers(records):
    source = records/BASE
    sys.path.insert(0,str(source.resolve()))
    base = importlib.import_module('fit')
    assert Path(base.__file__).resolve()==(source/'fit.py').resolve()
    base.ROOT = ROOT
    return base


def raw_path(records, frame):
    base = ROOT if frame['raw_record']==ROOT.name else records/frame['raw_record']
    path = base/'raw'/frame['file']
    assert (path.resolve().is_relative_to(records.resolve())
            or path.resolve().is_relative_to(ROOT.resolve()))
    return path


def prepare(records, raw):
    base = helpers(records)
    source = records/BASE
    seal = json.loads((source/'construction-seal.json').read_text())
    for name,digest in seal['files'].items():assert sha(source/name)==digest
    previous = json.loads((source/'inputs.json').read_text())
    result = json.loads((source/'fit-output.json').read_text())
    initial = result['families']['fitted_prism']['parameters']
    seed = previous['seed']
    _,vertices = base.geometry(np.asarray(initial),seed,True)
    protocol = json.loads((ROOT/'protocol.json').read_text())
    provenance_path = records/'sonar-translated-shadow-20261031/input-provenance.json.gz'
    provenance = json.loads(gzip.decompress(provenance_path.read_bytes()))
    rows = []
    (ROOT/'raw').mkdir()
    for index in [27,49,81,88,151,51,90,153]:
        if index not in [27,81]:
            row = dict(next(r for r in previous['frames'] if r['index']==index),raw_record=BASE)
            assert sha(raw_path(records,row))==row['raw_sha256']
        else:
            name = f'sonar-{index:04d}.npz'
            row = dict(next(r for r in provenance['frames'] if r['file']==name),index=index,raw_record=ROOT.name)
            assert sha(raw/name)==row['raw_sha256']
            (ROOT/'raw'/name).write_bytes((raw/name).read_bytes())
            with np.load(raw/name,allow_pickle=False) as z:
                from posim_slam.sonar_input import validate_calibrated_frame
                validate_calibrated_frame(dict(z),beams=256,horizontal_fov=2*np.pi/3,
                    range_max=15.,bandwidth_hz=29900.,sound_speed_m_s=1500.)
                ranges = z['ranges'];az = np.arctan2(z['beam_directions'][:,1],z['beam_directions'][:,0])
            local = (vertices-np.array(row['origin']))@np.array(row['rotation'])
            r = np.linalg.norm(local,axis=1);a = np.arctan2(local[:,1],local[:,0])
            ri = np.rint(np.interp(np.linspace(r.min()-.6,r.max()+.6,8),ranges,np.arange(len(ranges)))).astype(int)
            bi = np.rint(np.interp(np.linspace(a.min()-np.deg2rad(8),a.max()+np.deg2rad(8),4),az,np.arange(len(az)))).astype(int)
            row['pixels'] = np.unique(np.array([(i,j) for i in ri for j in bi]+
                [(i,j) for i in [80,140,200,300] for j in [32,96,160,224]]),axis=0).tolist()
        rows.append(row)
    write(ROOT/'inputs.json',dict(seed=seed,initial_parameters=initial,frames=rows,
        protocol_sha256=sha(ROOT/'protocol.json'),construction_truth_scene_rgb_used=False,
        previous_record_files={name:sha(source/name) for name in ['construction-seal.json','inputs.json',
            'fit-output.json','fit-output.npz']},provenance_sha256=sha(provenance_path)))
    package = Path(importlib.import_module('posim_slam').__file__).resolve().parent
    write(ROOT/'source-helpers.json',dict(base_record=BASE,modules={name:sha(source/name)
        for name in ['fit.py','assess.py','render.py']},package={name:sha(package/name)
        for name in ['sonar_surface_power.py','sonar_source_normals.py','sonar_source_surface.py',
                     'sonar_speckle.py','sonar_power_evidence.py']}))
    print(json.dumps(dict(prepared_indices=[r['index'] for r in rows],new_raw_copies=2,
                         existing_raw_shared=6,truth_scene_rgb_used=False)),flush=True)


def build(records):
    base = helpers(records)
    class Fit(base.Fit):
        def __init__(self):
            self.inputs = json.loads((ROOT/'inputs.json').read_text())
            self.protocol = json.loads((ROOT/'protocol.json').read_text())
            self.seed,self.frames = self.inputs['seed'],self.inputs['frames']
            self.models,self.powers = {},{}
            for f in self.frames:
                path = raw_path(records,f)
                assert sha(path)==f['raw_sha256']
                with np.load(path,allow_pickle=False) as z:
                    self.models[f['index']] = base.make_model(z)
                    pixels = np.array(f['pixels'])
                    self.powers[f['index']] = 10**(z['intensity_db'][pixels[:,0],pixels[:,1]].astype(float)/10)
            self.start,self.calls = time.monotonic(),0
            self.training = self.protocol['training_control_indices']

        def evaluate(self, parameters, prism=True, frames=None, final=False, causal=False):
            if frames is None:frames = [f for f in self.frames if f['index'] in self.training]
            return super().evaluate(parameters,prism,frames,final,causal)
    return Fit()


def support(records):
    from posim_slam.sonar_surface_power import TriangleSurface
    from posim_slam.sonar_source_surface import SourceSurfaceObservation
    f = build(records);base = helpers(records)
    triangles,_ = base.geometry(np.array(f.inputs['initial_parameters']),f.seed,True)
    rows = []
    for row in f.frames:
        if row['index'] not in [27,81]:continue
        local = (triangles-np.array(row['origin']))@np.array(row['rotation'])
        obs = SourceSurfaceObservation(f.models[row['index']],TriangleSurface(local))
        basis = base.mean_basis(obs,np.array(row['pixels']))
        ratio = np.exp(f.inputs['initial_parameters'][9])
        rows.append(dict(index=row['index'],predicted_object_hit_rays=int(sum(obs.hit_triangles.ravel()>=2)),
            predicted_object_power_fraction=float((ratio*basis[:,1]).sum()/(basis@np.array([1.,ratio])).sum()),
            queries=len(row['pixels']),raw_positive_selected=int(sum(f.powers[row['index']]>0)),
            shape_conditioned=True,physical_identity_verified=False))
    write(ROOT/'initial-view-support.json',dict(views=rows,truth_scene_rgb_used=False))
    print(json.dumps(rows,indent=2),flush=True)


def run(records):
    if (ROOT/'trace.jsonl').exists():raise FileExistsError('keep original attempted search')
    f = build(records)
    initial = np.array(f.inputs['initial_parameters'])
    states = {'initial_prism':initial}
    with (ROOT/'trace.jsonl').open('x') as trace:
        for label,key in [('three_view_control','training_control_indices'),
                          ('height_diverse_fit','training_height_diverse_indices')]:
            f.training = f.protocol[key]
            trace.write(json.dumps(dict(event='family',name=label,training_indices=f.training))+'\n');trace.flush()
            states[label] = f.search(initial,True,trace)
    report = dict(map_accepted=False,native_fusion=False,conditional_fixed_navigation=True,
        truth_scene_rgb_used_in_construction=False,families={})
    arrays = {}
    groups = {name:f.protocol[key] for name,key in [('original_three','training_control_indices'),
              ('height_diverse_five','training_height_diverse_indices'),('unused_temporal','unused_temporal_indices')]}
    groups['new_height_two'] = [27,81]
    for label,parameters in states.items():
        value,details,data,triangles,vertices = f.evaluate(parameters,True,f.frames,True)
        report['families'][label] = dict(parameters=parameters.tolist(),frames=details,
            score_groups={name:sum(d['score'] for d in details if d['index'] in indices)
                          for name,indices in groups.items()})
        for k,v in data.items():arrays[label+'_'+k] = v
        arrays[label+'_triangles'],arrays[label+'_vertices'] = triangles,vertices
    np.savez_compressed(ROOT/'fit-output.npz',**arrays)
    write(ROOT/'fit-output.json',report)
    names=['protocol.json','run_fit.py','inputs.json','source-helpers.json','initial-view-support.json',
           'trace.jsonl','fit-output.npz','fit-output.json']
    write(ROOT/'construction-seal.json',dict(truth_scene_rgb_used=False,
        files={name:sha(ROOT/name) for name in names},calls=f.calls,elapsed_s=time.monotonic()-f.start))
    print(json.dumps(report),flush=True)


if __name__=='__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('mode',choices=['prepare','support','fit','assess','render'])
    parser.add_argument('--records',type=Path,default=ROOT.parent)
    parser.add_argument('--raw',type=Path)
    args = parser.parse_args()
    if args.mode=='prepare':prepare(args.records,args.raw)
    elif args.mode=='support':support(args.records)
    elif args.mode=='fit':run(args.records)
    else:
        helpers(args.records)
        module = importlib.import_module('assess' if args.mode=='assess' else 'render')
        module.ROOT = ROOT
        if args.mode=='assess':
            print(json.dumps(module.evaluate(args.records,families=FAMILIES)[0],indent=2))
        else:module.main(args.records,families=FAMILIES)
