# SPDX-License-Identifier: Apache-2.0
"""Verify sealed conditional geometry and exact correlated power evidence.

Does not repeat optimization, reacquire sensors, or accept a map/native factor.
"""
import json
from pathlib import Path
import numpy as np
import posim_slam
from fit import Fit, geometry, sha

ROOT = Path(__file__).resolve().parent


def main(records):
    seal = json.loads((ROOT/'construction-seal.json').read_text())
    assert seal['truth_scene_rgb_used'] is False
    for name,digest in seal['files'].items():
        assert sha(ROOT/name)==digest,name
    manifest = json.loads((ROOT/'sha256.json').read_text())
    for name,digest in manifest.items():
        assert sha(ROOT/name)==digest,name
    components = json.loads((ROOT/'component-source.json').read_text())
    package = Path(posim_slam.__file__).resolve().parent
    for name,digest in components['modules'].items():
        assert sha(package/Path(name).name)==digest,name
    f = Fit()
    assert sha(ROOT/'protocol.json')==f.inputs['protocol_sha256']
    for name,digest in f.inputs['source_hashes'].items():
        assert sha(records/'sonar-translated-shadow-20261031'/name)==digest,name
    for row in f.frames:
        assert sha(ROOT/'raw'/row['file'])==row['raw_sha256']
    report = json.loads((ROOT/'fit-output.json').read_text())
    assert report['native_fusion'] is False and report['map_accepted'] is False
    with np.load(ROOT/'fit-output.npz',allow_pickle=False) as recorded:
        for label,case in report['families'].items():
            prism,causal = 'prism' in label,label=='causal_plane'
            p = np.asarray(case['parameters'])
            _,vertices = geometry(p,f.seed,prism)
            np.testing.assert_array_equal(vertices,recorded[label+'_vertices'])
            value,details,arrays,triangles,_ = f.evaluate(p,prism,f.frames,True,causal)
            np.testing.assert_allclose(triangles,recorded[label+'_triangles'],rtol=0,atol=0)
            for key,a in arrays.items():
                expected = recorded[label+'_'+key]
                # Same cross-platform source-normal/power reproduction budget
                # as the preceding native-power field baseline. Not map noise.
                np.testing.assert_allclose(a,expected,rtol=2e-7,
                    atol=2e-10*max(float(np.max(abs(expected))),1e-30))
            for row,original in zip(details,case['frames']):
                assert row['index']==original['index'] and row['pixels']==original['pixels']
                for key in ['score','gain','normalized_residual_rms']:
                    np.testing.assert_allclose(row[key],original[key],rtol=2e-7,atol=1e-10)
            np.testing.assert_allclose(sum(r['score'] for r in details[:3]),case['train_score'])
            np.testing.assert_allclose(sum(r['score'] for r in details[3:]),case['unused_temporal_score'])
            print(label,'all six full correlated moment records match',flush=True)
    trace = [json.loads(line) for line in (ROOT/'trace.jsonl').read_text().splitlines()]
    for family,label in [('plane','shared_plane'),('prism','fitted_prism')]:
        accepted = [r for r in trace if r['family']==family and r['accepted']]
        assert all(b['score']<a['score'] for a,b in zip(accepted,accepted[1:]))
        np.testing.assert_array_equal(accepted[-1]['parameters'],report['families'][label]['parameters'])
    from assess import evaluate
    assessed,arrays = evaluate(records,save=False)
    original = json.loads((ROOT/'assessment.json').read_text())
    assert assessed['references']==original['references']
    for name,case in assessed['families'].items():
        expected = original['families'][name]
        assert case['points']==expected['points']
        assert case['nearest_surface_counts']==expected['nearest_surface_counts']
        assert case['assessment_nearest_center_object']==expected['assessment_nearest_center_object']
        for kind in ['all_surface_error_m','target_surface_error_m']:
            assert case[kind]['beyond_10cm']==expected[kind]['beyond_10cm']
            for key in ['rmse','median','p90','maximum']:
                np.testing.assert_allclose(case[kind][key],expected[kind][key],rtol=1e-10,atol=1e-12)
    with np.load(ROOT/'assessed-clouds.npz',allow_pickle=False) as z:
        for name,array in arrays.items():np.testing.assert_allclose(array,z[name],rtol=1e-12,atol=1e-12)
    for name in ['initial_prism','fitted_prism']:
        ply = (ROOT/(name+'-complete.ply')).read_text().splitlines()
        count = original['families'][name]['points']
        assert f'element vertex {count}' in ply
        points = np.array([list(map(float,row.split())) for row in ply[ply.index('end_header')+1:]])
        np.testing.assert_array_equal(points,arrays[name+'_points'])
    print('Sealed fit, original raw hashes, complete traces and conditional moments verified; map unqualified')


if __name__=='__main__':
    import argparse
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--records',type=Path,default=ROOT.parent)
    main(parser.parse_args().records)
