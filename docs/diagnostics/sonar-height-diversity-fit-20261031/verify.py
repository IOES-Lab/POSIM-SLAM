# SPDX-License-Identifier: Apache-2.0
"""Recompute complete fixed-navigation fit evidence, without optimization."""
import argparse
import importlib
import importlib.util
import json
from pathlib import Path
import shutil
import tempfile
import numpy as np
import posim_slam
from run_fit import ROOT, BASE, FAMILIES, sha, raw_path, helpers, build


def main(records):
    for name,digest in json.loads((ROOT/'sha256.json').read_text()).items():
        assert sha(ROOT/name)==digest,name
    seal = json.loads((ROOT/'construction-seal.json').read_text())
    assert seal['truth_scene_rgb_used'] is False
    for name,digest in seal['files'].items():assert sha(ROOT/name)==digest,name
    sources = json.loads((ROOT/'source-helpers.json').read_text())
    for name,digest in sources['modules'].items():assert sha(records/BASE/name)==digest,name
    package = Path(posim_slam.__file__).resolve().parent
    for name,digest in sources['package'].items():assert sha(package/name)==digest,name
    f = build(records)
    for name,digest in f.inputs['previous_record_files'].items():assert sha(records/BASE/name)==digest
    assert sha(ROOT/'protocol.json')==f.inputs['protocol_sha256']
    assert sha(records/'sonar-translated-shadow-20261031/input-provenance.json.gz')==f.inputs['provenance_sha256']
    report = json.loads((ROOT/'fit-output.json').read_text())
    assert not report['map_accepted'] and not report['native_fusion']
    assert [r['index'] for r in f.frames]==[27,49,81,88,151,51,90,153]
    previous_inputs = json.loads((records/BASE/'inputs.json').read_text())
    for row in f.frames:
        assert sha(raw_path(records,row))==row['raw_sha256']
        if row['index'] not in [27,81]:
            old = next(r for r in previous_inputs['frames'] if r['index']==row['index'])
            assert row['pixels']==old['pixels']
    # The preceding verifier's arithmetic comparison is imported by filename
    # under a distinct name, avoiding this module's own script name.
    old_manifest = json.loads((records/BASE/'sha256.json').read_text())
    assert sha(records/BASE/'verify.py')==old_manifest['verify.py']
    spec = importlib.util.spec_from_file_location('previous_geometry_checks',records/BASE/'verify.py')
    previous_verify = importlib.util.module_from_spec(spec);spec.loader.exec_module(previous_verify)
    with np.load(ROOT/'fit-output.npz',allow_pickle=False) as saved:
        for label in FAMILIES:
            case = report['families'][label]
            value,details,arrays,triangles,vertices = f.evaluate(np.array(case['parameters']),True,f.frames,True)
            previous_verify.derived_geometry_equal(triangles,saved[label+'_triangles'])
            previous_verify.derived_geometry_equal(vertices,saved[label+'_vertices'])
            for name,array in arrays.items():
                expected = saved[label+'_'+name]
                np.testing.assert_allclose(array,expected,rtol=2e-7,
                    atol=2e-10*max(float(np.max(abs(expected))),1e-30))
            for row,original in zip(details,case['frames']):
                assert row['index']==original['index'] and row['pixels']==original['pixels']
                for key in ['score','gain','normalized_residual_rms']:
                    np.testing.assert_allclose(row[key],original[key],rtol=2e-7,atol=1e-10)
            for group,indices in [('original_three',[49,88,151]),('new_height_two',[27,81]),
                                  ('height_diverse_five',[27,49,81,88,151]),('unused_temporal',[51,90,153])]:
                np.testing.assert_allclose(sum(r['score'] for r in details if r['index'] in indices),
                                           case['score_groups'][group],rtol=2e-7)
            print(label,'all eight full moment/score records match',flush=True)
    labels = {}
    current = None
    for line in (ROOT/'trace.jsonl').read_text().splitlines():
        row = json.loads(line)
        if row.get('event')=='family':
            current = row['name'];labels[current] = []
        else:labels[current].append(row)
    for name in FAMILIES[1:]:
        rows = labels[name]
        np.testing.assert_array_equal(rows[0]['parameters'],f.inputs['initial_parameters'])
        accepted = [r for r in rows if r['accepted']]
        assert all(b['score']<a['score'] for a,b in zip(accepted,accepted[1:]))
        np.testing.assert_array_equal(accepted[-1]['parameters'],report['families'][name]['parameters'])
    helpers(records)
    assessment = importlib.import_module('assess');assessment.ROOT = ROOT
    actual,arrays = assessment.evaluate(records,save=False,families=FAMILIES)
    original = json.loads((ROOT/'assessment.json').read_text())
    assert actual['references']==original['references']
    with np.load(ROOT/'assessed-clouds.npz',allow_pickle=False) as z:
        for name,array in arrays.items():
            if name.endswith(('_points','_world')):previous_verify.derived_geometry_equal(array,z[name])
            else:np.testing.assert_allclose(array,z[name],rtol=1e-10,atol=1e-12)
        for name in FAMILIES:
            case,expected = actual['families'][name],original['families'][name]
            assert case['points']==expected['points']==548
            assert case['nearest_surface_counts']==expected['nearest_surface_counts']
            for kind in ['all_surface_error_m','target_surface_error_m']:
                assert case[kind]['beyond_10cm']==expected[kind]['beyond_10cm']
                for key in ['rmse','median','p90','maximum']:
                    np.testing.assert_allclose(case[kind][key],expected[kind][key],rtol=1e-10,atol=1e-12)
            ply = (ROOT/(name+'-complete.ply')).read_text().splitlines()
            assert 'element vertex 548' in ply
            points = np.array([list(map(float,r.split())) for r in ply[ply.index('end_header')+1:]])
            np.testing.assert_array_equal(points,z[name+'_points'])
    # A fresh bounded scratch directory reproduces the write-once forward audit.
    # Original records and all inferred states stay untouched.
    import run_fit
    forward = importlib.import_module('forward_audit')
    scratch = ROOT/'_verification_tmp'
    scratch.mkdir(exist_ok=True)
    original_root = run_fit.ROOT
    with tempfile.TemporaryDirectory(prefix='field-forward-',dir=scratch) as temporary:
        target = Path(temporary)/ROOT.name;target.mkdir()
        for name in list(seal['files'])+['construction-seal.json','forward-audit-protocol.json']:
            shutil.copy2(ROOT/name,target/name)
        (target/'raw').mkdir()
        for source in (ROOT/'raw').glob('*.npz'):shutil.copy2(source,target/'raw'/source.name)
        run_fit.ROOT = target;forward.ROOT = target
        try:
            forward.run(records)
        finally:
            run_fit.ROOT = original_root;forward.ROOT = original_root
            helpers(records)
        with np.load(target/'field-forward-audit.npz',allow_pickle=False) as a, \
                np.load(ROOT/'field-forward-audit.npz',allow_pickle=False) as b:
            for name in b.files:
                if name.endswith('_hit_triangles'):
                    # Internal shared-edge triangle IDs can differ at roundoff;
                    # retain the complete grid but compare physical hit support.
                    np.testing.assert_array_equal(a[name]>=0,b[name]>=0)
                elif name.endswith('_normals'):
                    np.testing.assert_allclose(a[name],b[name],rtol=0,atol=2e-5)
                elif name.endswith('_ranges'):
                    np.testing.assert_allclose(a[name],b[name],rtol=1e-7,atol=1e-6)
                else:
                    np.testing.assert_allclose(a[name],b[name],rtol=2e-7,
                        atol=2e-10*max(float(np.max(abs(b[name]))),1e-30))
        actual = json.loads((target/'field-forward-audit.json').read_text())
        expected = json.loads((ROOT/'field-forward-audit.json').read_text())
        assert actual['references']==expected['references']
        assert actual['source_sha256']==expected['source_sha256']
        for a,b in zip(actual['frames'],expected['frames']):
            assert (a['index'],a['arm'],a['available'])==(b['index'],b['arm'],b['available'])
            for key in ['gain','composite_score','normalized_residual_rms','normalized_residual_max']:
                np.testing.assert_allclose(a[key],b[key],rtol=2e-7,atol=1e-10)
    ablation = importlib.import_module('support_ablation');ablation.ROOT = ROOT
    actual,arrays = ablation.evaluate(records)
    expected = json.loads((ROOT/'scene-support-ablation.json').read_text())
    assert actual['references']==expected['references']
    assert actual['source_sha256']==expected['source_sha256']
    with np.load(ROOT/'scene-support-ablation.npz',allow_pickle=False) as z:
        for name,array in arrays.items():
            np.testing.assert_allclose(array,z[name],rtol=2e-7,
                atol=2e-10*max(float(np.max(abs(z[name]))),1e-30))
    for a,b in zip(actual['frames'],expected['frames']):
        assert (a['index'],a['arm'])==(b['index'],b['arm'])
        for key in ['gain','composite_score','normalized_residual_rms']:
            np.testing.assert_allclose(a[key],b[key],rtol=2e-7,atol=1e-10)
    print('Assessment-only full-scene/pose and support ablations reproduced; no reference entered inference')
    print('Same initial state/search schedule, shared raw hashes and every model sample verified; no map admission')


if __name__=='__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--records',type=Path,default=ROOT.parent)
    main(parser.parse_args().records)
