"""Independent prepared-data reproduction; NumPy and public POSIM-SLAM only."""
from pathlib import Path
import ast
import gzip
import hashlib
import json
import os
import shutil
import subprocess
import sys
import tempfile
import numpy as np

base = Path(__file__).resolve().parent
checks = 0
def check(condition, label):
    global checks
    if not condition:
        raise AssertionError(label)
    checks += 1
def read(relative):
    path = base / relative
    return json.loads(gzip.decompress(path.read_bytes()) if path.suffix == '.gz' else path.read_text())
def run(script, data, output):
    result = subprocess.run([sys.executable, str(base/script), '--input', str(data), '--output', str(output)],
        text=True, capture_output=True)
    if result.returncode:
        raise RuntimeError(result.stderr[-4000:])
def compare_arrays(actual, expected):
    with np.load(actual) as a, np.load(expected) as b:
        check(set(a.files) == set(b.files), str(actual)+' fields')
        for key in a.files:
            np.testing.assert_allclose(a[key], b[key], rtol=1e-10, atol=1e-12)
            check(a[key].shape == b[key].shape, str(actual)+' '+key)

manifest = read('manifest.json')
for relative, digest in manifest['sha256'].items():
    check(hashlib.sha256((base/relative).read_bytes()).hexdigest() == digest, relative+' digest')
for script in ('reproduce_support.py', 'reproduce_uncertainty.py'):
    imports = [node.module or '' for node in ast.walk(ast.parse((base/script).read_text())) if isinstance(node, ast.ImportFrom)]
    check(all(not name.startswith(('runtime', 'www', 'scripts')) for name in imports), script+' public imports')
    check('/Volumes/' not in (base/script).read_text(), script+' portable paths')

from posim_slam.native_surface import NativeSurfaceSnapshot
from posim_slam.surface_prior import Variable
from posim_slam.inertial import exp_so3
source = base.parent/'sonar-wall-corner-20261030/replay-navigation/snapshot-52500000000.json.gz'
snapshot = NativeSurfaceSnapshot.read(json.loads(gzip.decompress(source.read_bytes())))
with np.load(base/'data/joint-poses.npz') as data:
    stamps = data['stamps_ns'].copy()
    order = [Variable('state', int(t), 6, snapshot.reference('state', int(t)).reference[:12]) for t in stamps]
    joint = snapshot.joint_marginal(order)
    np.testing.assert_allclose(joint['observable_covariance'], data['covariance'], rtol=1e-10, atol=1e-12)
    np.testing.assert_allclose(joint['deltas'].reshape(-1,6), data['deltas'], rtol=0, atol=1e-12)
    for i, variable in enumerate(order):
        delta = data['deltas'][i]
        np.testing.assert_allclose(data['positions'][i], np.array(variable.reference[:3])+delta[:3], atol=1e-12)
        np.testing.assert_allclose(data['rotations'][i], exp_so3(delta[3:6])@np.array(variable.reference[3:12]).reshape(3,3), atol=1e-12)
    check(joint['unbounded_basis'].shape[1] == 0 and joint['workspace_bound_bytes'] <= 32*1024*1024, 'finite bounded native joint posterior')
check(len(stamps) == 41 and stamps[0] == 32_500_000_000 and stamps[-1] == 52_500_000_000, 'declared window')

with tempfile.TemporaryDirectory(prefix='posim-slam-source-support-') as temporary:
    temporary = Path(temporary)
    output = temporary/'reproduction'
    run('reproduce_support.py', base/'data', output)
    run('reproduce_uncertainty.py', base/'data', output)
    for name in ('chronological', 'blocked'):
        for prefix in ('heldout-', 'heldout-joint-'):
            compare_arrays(output/(prefix+name+'.npz'), base/'results'/(prefix+name+'.npz'))
    # Intervene on each fold's held-out images. Every construction/selection
    # array must remain unchanged, while validation support must react.
    frames = {r['stamp_ns']: r for r in read('data/source-image-summary.json')['records']}
    for name in ('chronological', 'blocked'):
        changed = temporary/('changed-'+name)
        shutil.copytree(base/'data', changed)
        with np.load(base/'results'/('heldout-'+name+'.npz')) as expected:
            heldout_stamps = expected['stamps_ns'][~expected['train']]
            for stamp in heldout_stamps:
                file = changed/'source-masks'/frames[int(stamp)]['file']
                with np.load(file) as packed:
                    arrays = {key: packed[key].copy() for key in packed.files}
                arrays['contrast'] = np.bitwise_not(arrays['contrast'])
                np.savez_compressed(file, **arrays)
            altered = temporary/('altered-'+name)
            run('reproduce_support.py', changed, altered)
            with np.load(altered/('heldout-'+name+'.npz')) as actual:
                for key in ('cells','points','stamps_ns','train','selected','optimistic','optimistic_three_sigma_m'):
                    np.testing.assert_array_equal(actual[key], expected[key])
                    check(True, name+' held-out intervention preserves '+key)
                check(not np.array_equal(actual['positive'][:,~actual['train']], expected['positive'][:,~expected['train']]), name+' validation reacts')

assessment = read('results/heldout-assessment.json')
alignment = assessment['alignment']
def distances(points):
    values = []
    for box in assessment['boxes']:
        delta = abs(points-box['centre'])-np.array(box['size'])/2
        values.append(np.where(np.max(delta,axis=1)>0, np.linalg.norm(np.maximum(delta,0),axis=1), np.min(-delta,axis=1)))
    return np.min(values,axis=0), np.argmin(values,axis=0)
for name in ('chronological','blocked'):
    with np.load(base/'results'/('heldout-'+name+'.npz')) as data, np.load(base/'results'/('heldout-joint-'+name+'.npz')) as information:
        world = data['points']@np.array(alignment['rotation']).T+alignment['translation']
        error, nearest = distances(world)
        with np.load(base/'results'/('heldout-assessed-'+name+'.npz')) as expected:
            np.testing.assert_allclose(world, expected['world'], atol=1e-12)
            np.testing.assert_allclose(error, expected['error_m'], atol=1e-12)
            np.testing.assert_array_equal(nearest, expected['nearest_surface'])
            check(True,name+' all candidate geometry')
        claimed = set()
        for index in information['exclusive_indices']:
            support = np.flatnonzero(data['train'] & data['positive'][index])
            keys = {(int(data['stamps_ns'][i]),int(data['pixels'][index,i])) for i in support}
            check(not keys & claimed,name+' distinct original support')
            claimed.update(keys)
        check(not data['optimistic'].any() and not np.any(information['correlated_pose_three_sigma_m']<=.1),name+' no promoted map')

from curve_model import fan_plane_curve
curves = read('legacy/curve-fits.json.gz')
for row in curves['rows']:
    measured, angles = np.array(row['range_m']), np.array(row['azimuth_rad'])
    predictions = [fan_plane_curve(mode, angles, vertical_fov_rad=np.pi/9)['ranges_m'] for mode in row['plane_modes_inverse_m']]
    np.testing.assert_allclose(np.sqrt(np.mean((measured-predictions[0])**2)), row['all_row_rms_m'], rtol=1e-8, atol=1e-10)
    for prediction in predictions[1:]:np.testing.assert_allclose(prediction,predictions[0],atol=1e-10)
    check(True,'curve fit including mirror ambiguity')
check(len(curves['rows'])==509 and sum(len(r['plane_modes_inverse_m'])>1 for r in curves['rows'])==368,'complete curve controls')
multiview = read('legacy/multiview.json.gz')
best = []
for row in multiview['rows']:
    rms = []
    for mode in row.get('modes',[]):
        if mode['status'] != 'nominal_prediction':continue
        error = np.array(mode['observed_range_m'])-mode['predicted_range_m']
        np.testing.assert_allclose(error,mode['residual_m'],atol=1e-12)
        np.testing.assert_allclose(np.sqrt(np.mean(error**2)),mode['rms_m'],atol=1e-12)
        rms.append(mode['rms_m'])
    if rms:best.append(min(rms))
np.testing.assert_allclose(np.percentile(best,[50,90]),multiview['best_mode_prediction_rms_m_p50_p90'])
check(len(best)==147 and sum(value<=.1 for value in best)==59,'complete cross-view curve failures')
legacy = read('legacy/hypothesis-assessment.json.gz')
error, _ = distances(np.array([r['world'] for r in legacy['rows']]))
np.testing.assert_allclose(error,[r['surface_error_m'] for r in legacy['rows']],atol=1e-12)
check(len(error)==2494 and np.sum(error>.1)==1033,'all conditional curve hypotheses assessed')

sources = read('data/beam-source-bounds.json')
geometry = {r['stamp_ns']:r for r in read('legacy/geometry-assessment.json.gz')['rows']}
groups = {'all':[],'positive':[],'unresolved':[]}
for row in sources['rows']:
    reference = geometry[row['stamp_ns']]
    for beam, lower in zip(row['beams'],row['lower']):
        values = [reference['any_visible_ray_abs_error_m'][beam], reference['any_surface_first_abs_error_m'][beam], abs(reference['measured_range_m'][beam]-reference['geometric_first_m'][beam])]
        if any(value is None for value in values):continue
        groups['all'].append(values)
        groups['positive' if lower>0 else 'unresolved'].append(values)
expected = read('results/beam-bound-assessment.json')
for name,values in groups.items():
    values = np.array(values)
    for column,key in enumerate(('any_visible_ray_m_p50_p90','any_surface_first_m_p50_p90','global_first_m_p50_p90')):
        np.testing.assert_allclose(np.percentile(values[:,column],[50,90]),expected[name][key])
    check(len(values)==expected[name]['count'],name+' source attribution')

for mode in ('source','wheel'):
    control = read('controls/'+mode+'-receipt.json')
    check(control['tests_run']==526 and control['failures']==control['errors']==0 and len(control['skipped'])==16,mode+' package test receipt')
figure = read('comparison-heldout.json')
check(figure['total_candidates']==figure['visible_candidates']==1714 and figure['qualified_map_points']==0,'complete matched figure')
check(figure['sonar_stamp_ns']==figure['camera_stamp_ns']==35_000_000_000 and figure['marker_area_pt2']==64,'figure source time and point size')
print('PASS:',checks,'checks; all candidates retained; held-out interventions preserve construction; no map qualified.')
