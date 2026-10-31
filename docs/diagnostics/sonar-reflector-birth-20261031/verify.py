# SPDX-License-Identifier: Apache-2.0
"""Recompute source-only birth discovery/selection and complete failed shapes."""
import argparse
import gzip
import importlib.util
import json
from pathlib import Path
import shutil
import sys
import tempfile
import numpy as np
import posim_slam
import run_birth
import assess_scene
import full_image_audit
import reference_image_audit
import native_full_audit
import native_reference_audit
from run_birth import ROOT,PREVIOUS,TRAIN,sha,raw_view,load_scenes
from posim_slam.sonar_scene_fit import SceneHeightFit


def old_arithmetic(records):
    source=records/PREVIOUS
    sys.path.insert(0,str(source))
    spec=importlib.util.spec_from_file_location('previous_birth_arithmetic',source/'verify.py')
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    return module.derived_power_equal


def geometry_equal(a,b):
    a,b=np.asarray(a),np.asarray(b)
    assert a.shape==b.shape
    if a.dtype.kind in 'biu':np.testing.assert_array_equal(a,b)
    else:
        budget=32*np.finfo(float).eps*np.maximum(1.,abs(b))
        assert np.all(np.isfinite(a)) and np.all(abs(a-b)<=budget)


def main(records,full=False):
    for name,digest in json.loads((ROOT/'sha256.json').read_text()).items():
        assert (ROOT/name).resolve().is_relative_to(ROOT.resolve()) and sha(ROOT/name)==digest,name
    for seal_name in ['discovery-seal.json','construction-seal.json']:
        seal=json.loads((ROOT/seal_name).read_text())
        assert not seal['truth_scene_rgb_used'] and not seal['unused_power_used']
        for name,digest in seal['files'].items():assert sha(ROOT/name)==digest,name
    package=Path(posim_slam.__file__).resolve().parent
    for name,digest in json.loads((ROOT/'component-source.json').read_text()).items():assert sha(package/name)==digest,name
    data=json.loads((ROOT/'inputs.json').read_text())
    for name,digest in data['references'].items():assert sha(records/name)==digest,name
    assert not data['truth_scene_rgb_used'] and not data['unused_raw_power_opened']
    derived_power_equal=old_arithmetic(records)
    # Re-run discovery in a fresh owned scratch directory. Original records and
    # operator/package sources remain untouched. No original raw is copied.
    parent=ROOT/'_verification_tmp';parent.mkdir(exist_ok=True)
    scratch=Path(tempfile.mkdtemp(prefix='birth-',dir=parent))
    try:
        for name in ['protocol.json','run_birth.py']:shutil.copyfile(ROOT/name,scratch/name)
        run_birth.ROOT=scratch
        run_birth.discover(records)
        actual=json.loads((scratch/'inputs.json').read_text())
        assert len(actual['modes'])==len(data['modes'])
        for a,b in zip(actual['modes'],data['modes']):
            for key in ['cell','distinct_views','memberships']:assert a[key]==b[key]
            for key in ['centre','ground_prior_z']:geometry_equal(a[key],b[key])
            np.testing.assert_allclose(a['rank_score'],b['rank_score'],rtol=32*np.finfo(np.float32).eps)
        for a,b in zip(actual['frames'],data['frames']):
            assert a['pixels']==b['pixels'] and a['index']==b['index']
            assert [r['label'] for r in a['birth_regions']]==[r['label'] for r in b['birth_regions']]
        with np.load(ROOT/'proposal-images.npz',allow_pickle=False) as saved,np.load(scratch/'proposal-images.npz',allow_pickle=False) as rebuilt:
            assert saved.files==rebuilt.files
            for key in saved.files:
                if key.endswith(('_mean','_arithmetic_envelope')):
                    if key.endswith('_mean') and '_check_' not in key:
                        envelope=saved[key.replace('_mean','_arithmetic_envelope')]
                        # Keep phase cancellation's local envelope separate
                        # from cross-platform float32 source-normal arithmetic.
                        budget=32*np.finfo(np.float32).eps*abs(saved[key])+envelope
                        assert np.all(abs(rebuilt[key]-saved[key])<=budget)
                    else:np.testing.assert_allclose(rebuilt[key],saved[key],rtol=32*np.finfo(np.float32).eps,atol=0.)
                elif saved[key].dtype.kind in 'biu':np.testing.assert_array_equal(rebuilt[key],saved[key])
                else:np.testing.assert_allclose(rebuilt[key],saved[key],rtol=2e-6,atol=1e-12)
        with np.load(ROOT/'scenes.npz',allow_pickle=False) as saved,np.load(scratch/'scenes.npz',allow_pickle=False) as rebuilt:
            assert saved.files==rebuilt.files
            for key in saved.files:geometry_equal(rebuilt[key],saved[key])
        assert json.loads(gzip.decompress((scratch/'memberships.json.gz').read_bytes()))==json.loads(gzip.decompress((ROOT/'memberships.json.gz').read_bytes()))
        print('All native proposal images, region labels, distinct-view cells and finite scenes reproduced',flush=True)
    finally:
        run_birth.ROOT=ROOT;shutil.rmtree(scratch)
    scenes=load_scenes();report=json.loads((ROOT/'fit-output.json').read_text())
    assert report['training_only_selection'] and not report['map_accepted'] and not report['native_update']
    views=[raw_view(records,row,row['pixels'])[0] for row in data['frames']]
    scores=[];budget=32*np.finfo(np.float32).eps
    with np.load(ROOT/'fit-output.npz',allow_pickle=False) as saved:
        for i,(scene,case) in enumerate(zip(scenes,report['alternatives'])):
            fit=SceneHeightFit(scene['vertices'],scene['faces'],scene['reflectivity'],views)
            result=fit.evaluate(fit.vertices[:,2],moments=i in [0,report['selected_index']])
            scores.append(result['score'])
            np.testing.assert_allclose(result['score'],case['score'],rtol=budget)
            for frame,row in zip(data['frames'],result['views']):
                prefix=f'scene{i}_frame{frame["index"]}_'
                np.testing.assert_array_equal(row['pixels'],saved[prefix+'pixels'])
                derived_power_equal(row['observed'],saved[prefix+'observed'])
                for name in ['mean','residual']:
                    np.testing.assert_allclose(row[name],saved[prefix+name],rtol=budget,
                        atol=budget*max(float(np.max(abs(saved[prefix+name]))),1e-30))
                if i in [0,report['selected_index']]:
                    np.testing.assert_allclose(row['moments']['covariance'],saved[prefix+'covariance'],rtol=budget,atol=0.)
            print('scene',i,'original training scores/moments match',flush=True)
    assert int(np.argmin(scores))==report['selected_index']==3
    # Recompute the unused predictions from fixed training geometry, without
    # selecting a candidate or accessing any assessment reference.
    scratch=Path(tempfile.mkdtemp(prefix='unused-',dir=parent))
    try:
        names=set(json.loads((ROOT/'discovery-seal.json').read_text())['files'])|set(json.loads((ROOT/'construction-seal.json').read_text())['files'])
        names.update(['construction-seal.json'])
        for name in names:shutil.copyfile(ROOT/name,scratch/name)
        run_birth.ROOT=scratch;run_birth.unused(records)
        with np.load(ROOT/'unused-output.npz',allow_pickle=False) as saved,np.load(scratch/'unused-output.npz',allow_pickle=False) as rebuilt:
            for key in saved.files:
                if key.endswith('_pixels'):np.testing.assert_array_equal(rebuilt[key],saved[key])
                elif key.endswith('_observed'):derived_power_equal(rebuilt[key],saved[key])
                else:np.testing.assert_allclose(rebuilt[key],saved[key],rtol=budget,
                    atol=budget*max(float(np.max(abs(saved[key]))),1e-30))
        assert json.loads((scratch/'unused-output.json').read_text())['selected_index_fixed_from_training']==3
    finally:run_birth.ROOT=ROOT;shutil.rmtree(scratch)
    _,arrays=assess_scene.evaluate(records,save=False)
    with np.load(ROOT/'assessed-clouds.npz',allow_pickle=False) as saved:
        for key,array in arrays.items():
            if array.dtype.kind in 'biu':np.testing.assert_array_equal(array,saved[key])
            else:np.testing.assert_allclose(array,saved[key],rtol=1e-10,atol=1e-12)
        for name,count in zip(assess_scene.FAMILIES,[3939,4019]):
            lines=(ROOT/(name+'-complete.ply')).read_text().splitlines()
            points=np.array([list(map(float,s.split())) for s in lines[lines.index('end_header')+1:]])
            assert len(points)==count
            np.testing.assert_array_equal(points,saved[name+'_points'])
    painter=json.loads((ROOT/'plot-helper.json').read_text());assert sha(records/painter['path'])==painter['sha256']
    if full:
        result,arrays=full_image_audit.run(records,save=False)
        original=json.loads((ROOT/'full-image-audit.json').read_text())
        with np.load(ROOT/'full-image-profiles.npz',allow_pickle=False) as saved:
            for key,array in arrays.items():np.testing.assert_allclose(array,saved[key],rtol=budget,
                atol=budget*max(float(np.max(abs(saved[key]))),1e-30))
        for a,b in zip(result['results'],original['results']):
            for key in ['score','gain','residual_rms']:np.testing.assert_allclose(a[key],b[key],rtol=budget)
        assert result['selected_index_unchanged']==3
        reference,profiles=reference_image_audit.run(records,save=False)
        original=json.loads((ROOT/'reference-image-audit.json').read_text())
        with np.load(ROOT/'reference-image-profiles.npz',allow_pickle=False) as saved:
            for key,array in profiles.items():np.testing.assert_allclose(array,saved[key],rtol=budget,
                atol=budget*max(float(np.max(abs(saved[key]))),1e-30))
        for a,b in zip(reference['results'],original['results']):
            for key in ['score','gain','residual_rms','residual_max']:np.testing.assert_allclose(a[key],b[key],rtol=budget)
        native,arrays=native_full_audit.run(records,save=False)
        assert native['full_image_agreement_passed'] and native['failed_pixels']==0
        with np.load(ROOT/'native-full-audit.npz',allow_pickle=False) as saved:
            for key,array in arrays.items():
                if array.dtype.kind in 'biu':np.testing.assert_array_equal(array,saved[key])
                else:np.testing.assert_allclose(array,saved[key],rtol=budget,atol=0.)
        supplied=native_reference_audit.verify(records,save=False)
        assert supplied['full_image_agreement_passed'] and supplied['failed_pixels']==0
    print('Every complete failed candidate remains; no native update/map acceptance',flush=True)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--records',type=Path,default=ROOT.parent)
    parser.add_argument('--full-image',action='store_true');args=parser.parse_args();main(args.records,args.full_image)
