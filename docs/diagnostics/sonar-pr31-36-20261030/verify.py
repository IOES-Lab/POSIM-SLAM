"""Check saved native complex outputs and bounded input-audit metadata with NumPy."""
import hashlib
import json
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent


def load(name):
    return json.loads((ROOT / name).read_text())


def main():
    for name, expected in load('sha256.json').items():
        path = (ROOT / name).resolve()
        if not path.is_relative_to(ROOT) or hashlib.sha256(path.read_bytes()).hexdigest() != expected:
            raise ValueError('saved_evidence_hash_mismatch: ' + name)
    report = load('endpoint-ffi.json')
    spacing = report['sound_speed_m_s'] / (2 * report['bandwidth_hz'])
    assert np.isclose(spacing, report['range_spacing_m'], rtol=0, atol=1e-14)
    results = {598: [], 599: []}
    with np.load(ROOT / 'endpoint-ffi-all.npz', allow_pickle=False) as output:
        assert len(output.files) == len(report['rows']) == 36
        for row in report['rows']:
            count, seed, distance = row['samples'], row['seed'], row['depth_m']
            values = output[f'n{count}_s{seed}_d{distance}']
            assert values.shape == (17, count, 2) and np.isfinite(values).all()
            amplitude = np.linalg.norm(values[-1], axis=-1)
            peak = int(amplitude.argmax()) if amplitude.max() > 0 else None
            error = abs(peak * spacing - distance) if peak is not None else None
            passed = (error is not None and error <= spacing/2 + 1e-5
                      if distance <= report['max_distance_m'] else np.count_nonzero(values) == 0)
            assert peak == row['peak_bin'] and passed == row['physical_grid_pass']
            assert (error is None and row['range_error_m'] is None
                    or error is not None and np.isclose(error, row['range_error_m'], rtol=0, atol=1e-12))
            results[count].append(passed)
    assert len(results[599]) == 18 and all(results[599])
    assert sum(not result for result in results[598]) == 6
    audit = load('field-audit.json')
    rows = audit['all_frames']
    assert len(rows) == audit['field']['sonar_frames'] == 95
    assert np.all(np.diff([row['stamp_ns'] for row in rows]) > 0)
    assert all(row['shape'] == [599, 256] and abs(row['reference_delta_ns']) <= 50_000_000
               and row['truth_bracket_ns'] <= 50_000_000
               and row['finite_returns'] + row['negative_infinity_no_return'] == 599*256 for row in rows)
    assert audit['range_last_m'] == 15 and audit['reference_mount_equals_sonar']
    assert audit['mission_status'] == 'completed'
    assert audit['dvl'] == [dict(valid=True, reason='bottom_locked', locked_beams=4, count=47)]
    for name in ('display-cpu.json', 'display-wgpu.json'):
        display = load(name)
        assert display['pass'] and display['exit'] == 0 and len(display['phases']) == 5
        assert all(len(phase['pairs']) == 25 for phase in display['phases'])
        assert sum(len(phase['pairs']) for phase in display['phases']) == 125
    print('PASS: all 36 saved operator outputs, 250 paired checks and 95-frame audit metadata; raw capture is not re-executed.')


if __name__ == '__main__':
    main()
