# SPDX-License-Identifier: Apache-2.0
"""Recompute sealed inverse-fit outputs without re-running the optimizer."""
import argparse
import json
from pathlib import Path
import numpy as np
import posim_slam
from run_fit import ROOT, FAMILIES, BASE, PREVIOUS, sha, build
from assess import evaluate


def main(records):
    for name, digest in json.loads((ROOT/'sha256.json').read_text()).items():
        assert (ROOT/name).resolve().is_relative_to(ROOT.resolve())
        assert sha(ROOT/name) == digest, name
    seal = json.loads((ROOT/'construction-seal.json').read_text())
    assert not seal['truth_scene_rgb_used']
    for name, digest in seal['files'].items():
        assert sha(ROOT/name) == digest, name
    package = Path(posim_slam.__file__).resolve().parent
    for name, digest in json.loads((ROOT/'component-source.json').read_text()).items():
        assert sha(package/name) == digest, name
    inputs = json.loads((ROOT/'inputs.json').read_text())
    for name, digest in inputs['references'].items():
        assert sha(records/name) == digest, name
    assert inputs['protocol_sha256'] == sha(ROOT/'protocol.json')
    review = json.loads((ROOT/'source-review.json').read_text())
    assert {r['number'] for r in review['source_scope']} == {6,31,32,33,34,35,36}
    assert all(r['state']=='open' and not r['merged'] for r in review['source_scope'])
    fit, all_views, frames = build(records)
    report = json.loads((ROOT/'fit-output.json').read_text())
    assert not report['map_accepted'] and not report['native_fusion']
    assert not report['truth_scene_rgb_used']
    original = json.loads((records/PREVIOUS/'inputs.json').read_text())
    assert [r['index'] for r in frames] == [27,49,81,88,151,51,90,153]
    for row, old in zip(frames, original['frames']):
        assert row == old
    with np.load(ROOT/'seed.npz',allow_pickle=False) as seed:
        with np.load(records/BASE/'seed-candidates.npz',allow_pickle=False) as source:
            for name in ['points','labels']:
                np.testing.assert_array_equal(seed[name],source[name])
        assert set(seed['labels']) == {1,2}
        assert len(seed['vertices']) == 339 and len(seed['faces']) == 600
    # Float32 source-normal arithmetic is distinct from measurement noise. All
    # serialized arrays/files still have exact hashes. Derived means/covariances
    # use a 32-float32-epsilon relative arithmetic budget, never map admission.
    budget = 32*np.finfo(np.float32).eps
    with np.load(ROOT/'fit-output.npz',allow_pickle=False) as saved:
        for name in FAMILIES:
            actual = all_views.evaluate(saved[name+'_vertices'][:,2], moments=True)
            np.testing.assert_array_equal(actual['vertices'], saved[name+'_vertices'])
            np.testing.assert_array_equal(actual['faces'], saved[name+'_faces'])
            np.testing.assert_array_equal(actual['face_ray_counts'],saved[name+'_face_ray_counts'])
            for frame, row, old in zip(frames, actual['views'], report['families'][name]['frames']):
                prefix = name+f'_frame{frame["index"]}_'
                for key in ['pixels','observed']:
                    np.testing.assert_array_equal(row[key], saved[prefix+key])
                for key in ['mean','residual']:
                    np.testing.assert_allclose(row[key],saved[prefix+key],rtol=budget,
                        atol=budget*max(float(np.max(abs(saved[prefix+key]))),1e-30))
                np.testing.assert_allclose(row['moments']['covariance'],saved[prefix+'covariance'],
                                           rtol=budget,atol=0.)
                np.testing.assert_allclose([row['score'],row['gain']],[old['score'],old['gain']],rtol=budget)
            print(name,'all eight original power/moment/geometry records match',flush=True)
        final = saved[FAMILIES[1]+'_vertices'][:,2]
        current = fit.vertices[:,2].copy()
        score = None
        trace = [json.loads(line) for line in (ROOT/'trace.jsonl').read_text().splitlines()]
        for row in trace:
            if row['kind']=='current':
                np.testing.assert_array_equal(row['heights'], current)
                score = row['score']
            elif row['accepted']:
                assert row['score'] < score
                current,score = np.array(row['heights']),row['score']
            if row['kind']=='proposal':
                assert np.max(abs(np.array(row['heights'])-fit.vertices[:,2])) <= .4+1e-14
        np.testing.assert_array_equal(current,final)
        expected = report['families'][FAMILIES[1]]
        np.testing.assert_allclose(score,expected['training_score']+expected['prior_penalty'],rtol=budget)
        ground_faces = inputs['initialization']['grid_faces']
        face = int(np.argmax(saved[FAMILIES[1]+'_face_ray_counts'][:ground_faces]))
        vertex = int(saved[FAMILIES[1]+'_faces'][face,0])
        changed = final.copy();changed[vertex] += .001
        negative = all_views.evaluate(changed)
        assert any(not np.allclose(row['mean'],saved[FAMILIES[1]+f'_frame{frame["index"]}_mean'],
                                  rtol=budget,atol=0.)
                   for frame,row in zip(frames,negative['views']))
    actual, arrays = evaluate(records, save=False)
    original = json.loads((ROOT/'assessment.json').read_text())
    assert actual['references'] == original['references']
    with np.load(ROOT/'assessed-clouds.npz',allow_pickle=False) as saved:
        for key, value in arrays.items():
            if value.dtype.kind in 'biu':
                np.testing.assert_array_equal(value,saved[key])
            else:
                np.testing.assert_allclose(value,saved[key],rtol=1e-10,atol=1e-12)
        for name in FAMILIES:
            ply = (ROOT/(name+'-complete.ply')).read_text().splitlines()
            points = np.array([list(map(float,line.split())) for line in ply[ply.index('end_header')+1:]])
            np.testing.assert_array_equal(points,saved[name+'_points'])
            assert len(points) == 3939
    print('All complete finite hypotheses preserved; no accepted map/native update',flush=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--records', type=Path, default=ROOT.parent)
    main(parser.parse_args().records)
