"""Verify the complete recorded input contract with NumPy and POSIM-SLAM."""
from pathlib import Path
import hashlib
import gzip
import json
import math

import numpy as np
from posim_slam.sonar_input import validate_calibrated_frame


ROOT = Path(__file__).resolve().parent


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    manifest = json.loads((ROOT / 'sha256.json').read_text())
    for name, expected in manifest.items():
        assert digest(ROOT / name) == expected, name
    profile = json.loads((ROOT / 'profile.json').read_text())
    assert digest(ROOT / 'inputs.npz') == profile['inputs_sha256']
    rows = iter(profile['inputs'])
    count = empty = 0
    with np.load(ROOT / 'inputs.npz', allow_pickle=False) as data:
        for backend, expected_count in [('wgpu', 192), ('cpu', 5)]:
            power = data[backend + '_intensity_db']
            stamps = data[backend + '_stamp']
            assert len(power) == len(stamps) == expected_count
            last_stamp = -1
            for values, stamp in zip(power, stamps):
                row = next(rows)
                assert row['backend'] == backend
                stamp_ns = int(stamp[0]) * 10**9 + int(stamp[1])
                assert stamp_ns == row['stamp_ns'] and stamp_ns > last_stamp
                last_stamp = stamp_ns
                original_hash = hashlib.sha256(values.astype('<f4').tobytes()).hexdigest()
                assert original_hash == row['raw_sha256']
                frame = dict(intensity_db=values, stamp=stamp,
                             ranges=data['ranges'], beam_directions=data['beam_directions'])
                validate_calibrated_frame(frame, **profile['calibration'])
                assert hashlib.sha256(values.astype('<f4').tobytes()).hexdigest() == original_hash
                count += 1
                empty += not np.isfinite(values).any()
    assert next(rows, None) is None
    equivalent = json.loads((ROOT / 'finite-equivalence.json').read_text())
    assert equivalent['identical'] and equivalent['frames'] == 95
    native = json.loads((ROOT / 'native-control-final.json').read_text())
    assert native['passed'] and native['synthetic_auxiliaries']
    assert native['sonar_frames_received'] == 192
    assert native['native_last_stamp_ns'] == native['last_input_stamp_ns']
    assert native['optimized']['dvl'] == native['optimized']['pressure'] == 187
    assert native['shutdown_requested'] and not native['forced_sigkill']
    assert not native['navigation_qualified'] and not native['map_qualified']
    assessment = json.loads((ROOT / 'vio-map-assessment-final.json').read_text())
    assert len(assessment) == 10 and all(not row['coordinates_filtered'] for row in assessment)
    for row in assessment:
        directory = ROOT / 'vio' / row['profile']
        suffix = '-recovered' if row['phase'] == 'recovered' else ''
        report = json.loads(gzip.decompress((directory / ('visual-surface-evaluation' + suffix + '.json.gz')).read_bytes()))
        comparison = json.loads(gzip.decompress((directory / ('comparison' + suffix + '.json.gz')).read_bytes()))
        points = report['per_point']
        assert len(points) == row['points'] == report['points'] == comparison['total_points']
        assert not comparison['coordinates_filtered'] and not report['map_qualified']
        assert sorted(point['export_vertex_index'] for point in points) == list(range(row['points']))
        errors = [point['distance_m'] for point in points]
        assert sum(error > .1 for error in errors) == row['above_10cm']
        assert math.isclose(math.sqrt(math.fsum(error * error for error in errors) / len(errors)), row['surface_rmse_m'], abs_tol=1e-14)
        ply = directory / ('map.ply' if suffix else 'initial-map.ply')
        with ply.open('rb') as stream:
            for _ in range(100):
                line = stream.readline()
                if line.startswith(b'element vertex '):
                    assert int(line.split()[-1]) == row['points']
                    break
            else:
                raise AssertionError('Missing PLY vertex count')
    print(f'PASS: {count} unchanged input frames ({empty} empty); '
          '95-frame finite control; native continuity; 10 complete VIO map assessments.')


if __name__ == '__main__':
    main()
