"""Verify the source-bound arithmetic record and retained raw sensor example."""
from pathlib import Path
import hashlib
import json

import numpy as np
from posim_slam.sonar_input import validate_calibrated_frame

ROOT = Path(__file__).resolve().parent


def read(name):
    return json.loads((ROOT / name).read_text())


def main():
    for name, expected in read('sha256.json').items():
        path = (ROOT / name).resolve()
        assert path.is_relative_to(ROOT)
        assert hashlib.sha256(path.read_bytes()).hexdigest() == expected, name
    identity = read('source-identity.json')
    assert identity['source_integration'] == 'ffecee9be9976f68c9e883d6cf3bbd6d40eb7729'
    assert {row['number'] for row in identity['source_scope']} == {6, 31, 32, 33, 34, 35, 36}
    assert identity['all_heads_are_ancestors'] and identity['loaded_in_vehicle']
    assert not identity['installed_sdk_changed'] and not identity['estimator_started']
    report = read('source-comparison.json')
    assert report['protocol']['source_revision'] == identity['source_integration']
    assert report['protocol']['binary_sha256'] == identity['rendering_library_sha256']
    assert report['protocol']['shape'] == [256, 256, 599]
    audit = read('field-audit.json')
    rows = audit['all_frames']
    assert audit['field']['passed'] and len(rows) == audit['field']['sonar_frames'] == 92
    assert audit['mission_status'] == 'completed'
    assert audit['lifecycle']['exit'] == 0 and not audit['lifecycle']['forced']
    assert all(not row['live_members'] for row in audit['lifecycle']['groups'])
    assert np.all(np.diff([row['stamp_ns'] for row in rows]) > 0)
    assert all(abs(row['reference_delta_ns']) <= 10_000_000 for row in rows)
    assert all(row['shape'] == [599, 256] for row in rows)
    selected = audit['figure_frame']
    raw_path = ROOT / 'field' / selected['file']
    assert hashlib.sha256(raw_path.read_bytes()).hexdigest() == selected['sha256']
    with np.load(raw_path, allow_pickle=False) as data:
        frame = {key: data[key] for key in data.files}
        validate_calibrated_frame(frame, **read('sampling-contract.json')['calibration'])
        ranges = data['ranges'].copy()
    assert ranges.shape == (599,) and abs(ranges[-1] - 15.) < 2e-6
    tail = ranges >= 10.
    row = next(row for row in report['rows'] if row['frame'] == 20)
    with np.load(ROOT / 'operator-frame20.npz', allow_pickle=False) as data:
        assert data['native'].dtype == np.complex64
        assert data['floating'].dtype == data['quantized'].dtype == np.complex128
        arrays = {key: data[key].astype(np.complex128) for key in data.files}
    assert all(value.shape == (256, 599) and np.isfinite(value).all()
               for value in arrays.values())
    actual = arrays['native']
    for key, value in arrays.items():
        np.testing.assert_allclose(np.mean(abs(value[:, tail]) ** 2),
                                   row[key + '_tail_mean_power'], rtol=2e-6)
    for key in ('floating', 'quantized'):
        for region, mask in [('whole', slice(None)), ('tail', tail)]:
            error = np.linalg.norm(actual[:, mask] - arrays[key][:, mask]) / np.linalg.norm(actual[:, mask])
            np.testing.assert_allclose(error, row[region + '_complex_residual_' + key], rtol=2e-6)
    assert row['whole_complex_residual_floating'] < 2e-5
    assert row['tail_complex_residual_floating'] < .004
    assert row['tail_complex_residual_quantized'] > 1
    for comparison in (report, read('previous-operator-timing.json')):
        benchmark = comparison['benchmark']
        times = np.asarray(benchmark['times_s'])
        assert len(times) == benchmark['samples'] == 16 and np.all(times > 0)
        np.testing.assert_allclose(np.median(times), benchmark['median_s'], rtol=1e-12)
        np.testing.assert_allclose(np.quantile(times, .95), benchmark['p95_s'], rtol=1e-12)
    display = read('display-wgpu.json')
    assert display['pass'] and sum(len(phase['pairs']) for phase in display['phases']) == 125
    assert read('optical-profile.json')['visibility_m'] == 2
    print('PASS: complete arithmetic frame, timing statistics, 92-frame audit, '
          'retained original raw example and 125-pair field receipt. No SLAM map is qualified.')


if __name__ == '__main__':
    main()
