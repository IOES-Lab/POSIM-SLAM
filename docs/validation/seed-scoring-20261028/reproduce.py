"""Compare scalar and batched startup scoring using measured camera/IMU only."""
import argparse
import gzip
import hashlib
import importlib.util
import json
import platform
from pathlib import Path
import sys
import time
import numpy as np


def main():
    sys.dont_write_bytecode = True
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--component', type=Path, help='POSIM-SLAM repository directory')
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--repeat', type=int, default=3, choices=range(1, 11))
    a = p.parse_args()
    here = Path(__file__).resolve().parent
    repository = here.parents[2]
    component = a.component or (repository if (repository/'src/posim_slam').is_dir()
                                else repository/'libraries/posim-slam')
    sys.path.insert(0, str(component/'src'))
    import posim_slam.visual_inertial_seed as seed
    import posim_slam.visual_seed_consensus as batched
    from posim_slam.inertial import ImuWindow
    spec = importlib.util.spec_from_file_location('seed_scalar_reference', here/'scalar_reference.py')
    scalar = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(scalar)
    fixture = json.loads(gzip.decompress((here/'fixture.json.gz').read_bytes()))
    assert fixture['schema'] == 'posim-seed-measured-windows-1'
    reports = []
    original = seed.consensus_solution
    try:
        for data in fixture['windows']:
            window = ImuWindow(max_samples=4096, max_gap=.020000001)
            for row in data['raw_imu']:
                window.add(row['stamp_ns']/1e9, row['angular_velocity'], row['specific_force'])
            expected = None
            runs = []
            maximum = 0.
            for mode in ['scalar', 'batched']*a.repeat:
                seed.consensus_solution = scalar.consensus_solution if mode == 'scalar' else batched.consensus_solution
                start = time.perf_counter()
                result = seed.tracked_imu_seed(data['frames'], window, **fixture['settings'])
                elapsed = time.perf_counter()-start
                if expected is None:
                    expected = result
                for key, value in result.items():
                    if isinstance(value, (np.ndarray, float)):
                        np.testing.assert_allclose(value, expected[key], atol=1e-10, rtol=1e-10)
                        maximum = max(maximum, float(np.max(np.abs(np.asarray(value)-expected[key]))))
                    else:
                        assert value == expected[key], key
                runs.append(dict(mode=mode, elapsed_s=elapsed, selected_tracks=result['selected_tracks'],
                    inlier_tracks=result['inlier_tracks'], rejected_tracks=result['rejected_tracks']))
            reports.append(dict(window=data['name'], max_absolute_numeric_difference=maximum, runs=runs))
    finally:
        seed.consensus_solution = original
    sources = [here/'fixture.json.gz', here/'scalar_reference.py', Path(__file__),
               Path(seed.__file__), Path(batched.__file__)]
    report = dict(passed=True, scope=__doc__, truth_input=False, tolerance=1e-10,
        hypotheses=256, pixel_sigma=.5, python=platform.python_version(), numpy=np.__version__,
        architecture=platform.machine(), sha256={file.name:hashlib.sha256(file.read_bytes()).hexdigest()
        for file in sources}, windows=reports)
    a.output.write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(dict(passed=True, windows=len(reports),
        max_absolute_numeric_difference=max(row['max_absolute_numeric_difference'] for row in reports))))


if __name__ == '__main__':main()
