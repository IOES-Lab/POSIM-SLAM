"""Build and qualify the experimental headless Basalt VIO research target.

This does not select a runtime backend or alter an application/SDK. Dependencies
are exact, clean checkouts; keep this source directory beside build receipts.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
RECIPE = ROOT / 'native/basalt'


def run(command, **kwargs):
    return subprocess.run([str(v) for v in command], check=True, **kwargs)


def verify_sources(source):
    pins = json.loads((RECIPE / 'sources.json').read_text())['dependencies']
    for name, pin in pins.items():
        checkout = source / name
        revision = subprocess.check_output(
            ['git', '-C', str(checkout), 'rev-parse', 'HEAD'], text=True).strip()
        dirty = subprocess.check_output(
            ['git', '-C', str(checkout), 'status', '--porcelain'], text=True)
        if revision != pin['revision'] or dirty:
            raise ValueError('basalt_source_pin_or_cleanliness:' + name)
        if not (checkout / pin['notice']).is_file():
            raise ValueError('basalt_source_notice_missing:' + name)
    return pins


def fetch_sources(source):
    """Fetch only missing pinned repositories; never reset an existing tree."""
    pins = json.loads((RECIPE / 'sources.json').read_text())['dependencies']
    source.mkdir(parents=True, exist_ok=True)
    for name, pin in pins.items():
        checkout = source / name
        if checkout.exists():
            continue
        run(['git', 'init', checkout])
        run(['git', '-C', checkout, 'remote', 'add', 'origin', pin['repository']])
        run(['git', '-C', checkout, 'fetch', '--depth=1', 'origin', pin['revision']])
        run(['git', '-C', checkout, 'checkout', '--detach', 'FETCH_HEAD'])
    verify_sources(source)


def metrics(path):
    import numpy as np
    data = np.genfromtxt(path, delimiter=',', names=True)
    if data.ndim != 1 or len(data) != 61:
        raise ValueError('basalt_missing_states')
    values = np.column_stack([data[n] for n in data.dtype.names])
    if not np.isfinite(values).all() or not np.all(np.diff(data['time']) > 0):
        raise ValueError('basalt_invalid_states')
    p = np.column_stack([data[n] for n in ('x', 'y', 'z')])
    truth = np.column_stack([data[n] for n in ('gtx', 'gty', 'gtz')])
    q = np.column_stack([data[n] for n in ('qx', 'qy', 'qz', 'qw')])
    gtq = np.column_stack([data[n] for n in ('gtqx', 'gtqy', 'gtqz', 'gtqw')])
    q /= np.linalg.norm(q, axis=1)[:, None]
    angles = 2 * np.arccos(np.clip(np.abs((q * gtq).sum(axis=1)), 0, 1))
    bg = np.array([data[n][-1] for n in ('bgx', 'bgy', 'bgz')])
    ba = np.array([data[n][-1] for n in ('bax', 'bay', 'baz')])
    result = dict(states=len(data), position_rmse_m=float(np.sqrt(np.mean(
        np.sum((p - truth)**2, axis=1)))), rotation_rmse_rad=float(
        np.sqrt(np.mean(angles**2))), final_gyro_bias_error_rad_s=float(
        np.linalg.norm(bg - [.003, -.002, .004])), final_accel_bias_error_m_s2=float(
        np.linalg.norm(ba - [.025, -.015, .01])))
    # A short deterministic synthetic regression, not product accuracy limits.
    if (result['position_rmse_m'] > .12 or result['rotation_rmse_rad'] > .03
            or result['final_gyro_bias_error_rad_s'] > .001
            or result['final_accel_bias_error_m_s2'] > .025):
        raise ValueError('basalt_synthetic_regression:' + json.dumps(result))
    return result


def build(engine, source, output, jobs=2):
    engine, source, output = (Path(p).resolve() for p in (engine, source, output))
    if output.exists():
        raise ValueError('basalt_output_exists')
    if not 1 <= jobs <= 4:
        raise ValueError('basalt_invalid_parallelism')
    pins = verify_sources(source)
    cmake = engine / 'bin/cmake'
    output.mkdir(parents=True)
    run([cmake, '-S', RECIPE, '-B', output / 'build',
         '-DCMAKE_BUILD_TYPE=Release', '-DCMAKE_POLICY_VERSION_MINIMUM=3.5', '-DCMAKE_PREFIX_PATH=' + str(engine),
         '-DSOURCE_ROOT=' + str(source)])
    run([cmake, '--build', output / 'build', '--parallel', jobs])
    run([cmake, '--build', output / 'build', '--parallel', jobs,
         '--target', 'basalt_surface_owner_probe'])
    run([sys.executable, RECIPE / 'test_surface_owner.py'], timeout=120,
        env={**os.environ, 'POSIM_SLAM_SURFACE_OWNER_PROBE': str(output / 'build/basalt_surface_owner_probe'),
             'PYTHONPATH': str(ROOT / 'src') + os.pathsep + os.environ.get('PYTHONPATH', '')})
    run([output / 'build/basalt_initial_prior_probe'], timeout=30)
    run([output / 'build/basalt_reference_patch_probe'], timeout=60)
    results = {}
    for mode in ('tracks', 'images'):
        path = output / (mode + '.csv')
        command = [output / 'build/basalt_synthetic_probe', path, 5]
        if mode == 'images':
            command.append('images')
        run(command, timeout=60)
        results[mode] = metrics(path)
    notices = output / 'notices'
    notices.mkdir()
    for name, pin in pins.items():
        shutil.copyfile(source / name / pin['notice'], notices / (name + '.txt'))
    receipt = dict(schema='posim-slam-basalt-research-1', passed=True,
                   initial_bias_prior_check='actual_constructor_and_physical_state_increments_v1',
                   reference_patch_check='retire_unobserved_all_camera_union_400_frames_v1',
                   surface_owner_check='synthetic_joint_marginal_late_increment_trial_retirement_v1',
                   scope='12-second synthetic 6DoF VIO; no ROS/native-world or package acceptance',
                   camera_hz=5, imu_hz=200, truth_input=False,
                   source_root=str(source), sources=pins, results=results,
                   recipe_sha256={str(p.relative_to(RECIPE)): hashlib.sha256(
                       p.read_bytes()).hexdigest() for p in RECIPE.rglob('*') if p.is_file()})
    binary = output / 'build/basalt_synthetic_probe'
    receipt['binary_sha256'] = hashlib.sha256(binary.read_bytes()).hexdigest()
    (output / 'receipt.json').write_text(json.dumps(receipt, indent=2) + '\n')
    return receipt


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('engine', 'source', 'output'):
        parser.add_argument('--' + name, type=Path, required=True)
    parser.add_argument('--jobs', type=int, default=2)
    parser.add_argument('--fetch', action='store_true', help='Fetch missing pinned sources')
    args = parser.parse_args()
    if args.fetch:
        fetch_sources(args.source.resolve())
    print(json.dumps(build(args.engine, args.source, args.output, args.jobs), indent=2))
