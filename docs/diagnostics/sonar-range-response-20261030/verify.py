"""Recompute saved source-response diagnostics using public NumPy APIs only."""
from pathlib import Path
import argparse
import hashlib
import json
import numpy as np
from posim_slam.sonar_input import validate_calibrated_frame
from posim_slam.sonar_range_response import PosimRangeResponse

ROOT = Path(__file__).resolve().parent


def load(name):
    return json.loads((ROOT/name).read_text())


def expected_panel(xyz, normal, angles, samples, maximum, stride):
    ranges = np.linalg.norm(xyz.astype(float), axis=2).T
    incidence = normal[..., 2].T.astype(float)/255
    alpha = .0354*np.log(10)/20
    variance = np.exp(-4*alpha*ranges)*incidence**2/ranges**3
    selected = np.zeros(ranges.shape[1], dtype=bool)
    selected[::stride] = True
    variance[:, ~selected] = 0
    h = (angles[-1]-angles[0])/(len(angles)-1)
    mixing = abs(np.sinc(.884/h*np.sin(angles[:, None]-angles[None, :])))
    mixing /= np.linalg.norm(mixing)
    response = PosimRangeResponse(samples, 29900., 1500.)
    power = response.mean_power(ranges, variance, mixing, range_max_m=maximum)
    middle = len(angles)//2
    peak = int(power[middle].argmax())
    queries = np.array([[middle, peak], [middle, peak+1], [middle+1, peak]])
    moments = response.power_moments(ranges, variance, mixing, queries, range_max_m=maximum)
    correlation = moments['covariance']/(moments['mean'][:, None]*moments['mean'][None, :])
    return power, correlation


def relative_residual(measured, expected):
    scale = np.sum(measured*expected)/np.sum(expected**2)
    return float(np.linalg.norm(measured-scale*expected)/np.linalg.norm(measured)), scale*expected


def verify_operator(directory):
    report = json.loads((directory/'endpoint-ffi.json').read_text())
    count = 0
    with np.load(directory/'endpoint-ffi-all.npz', allow_pickle=False) as data:
        for row in report['rows']:
            n, seed, distance = row['samples'], row['seed'], row['depth_m']
            output = data[f'n{n}_s{seed}_d{distance}']
            if distance > report['max_distance_m']:
                assert not output.any()
                continue
            values = output[-1, :, 0]+1j*output[-1, :, 1]
            template = PosimRangeResponse(n, 29900., 1500.).complex_response([distance])[0]
            gain = np.vdot(template, values)/np.vdot(template, template)
            error = np.linalg.norm(values-gain*template)/np.linalg.norm(values)
            assert error < .0002, (n, seed, distance, error)
            count += 1
    assert count == 32


def verify_panel():
    root = ROOT/'captures/panel'
    observation = json.loads((root/'observations.json').read_text())
    source = json.loads((root/'result.json').read_text())
    aliases = json.loads((root/'aliases.json').read_text())
    assert source['pass'] and source['exit'] == 0 and not observation['error']
    assert all(r['exit'] == 0 and not r['forced'] for r in observation['bridge_cleanup'])
    rows = {kind: {r['stamp_ns']: r for r in observation['rows'] if r['kind'] == kind}
            for kind in ['raw', 'normal', 'depth']}
    assert len(rows['raw']) == 180 and set(rows['raw']) == set(rows['normal']) == set(rows['depth'])
    recorded = load('renderer-conditioned-panel.json')['all_rows']
    with np.load(root/'arrays.npz', allow_pickle=False) as data:
        def value(key):
            return data[aliases[key]]
        for stamp, row in rows['raw'].items():
            frame = dict(intensity_db=value(row['key']), ranges=value(row['key']+'_ranges'),
                         beam_directions=value(row['key']+'_directions'), stamp=[stamp//10**9, stamp % 10**9])
            validate_calibrated_frame(frame, beams=32, horizontal_fov=2.26894,
                range_max=10., bandwidth_hz=29900., sound_speed_m_s=1500.)
            assert hashlib.sha256(frame['intensity_db'].astype('<f4').tobytes()).hexdigest() == row['raw_payload_sha256']
        for phase, expected in zip([source['phases'][0], source['phases'][-1]], recorded):
            stamps = [pair['stamp_ns'] for pair in phase['pairs']]
            assert len(stamps) == 25
            xyz = value(rows['depth'][stamps[0]]['key'])
            normal = value(rows['normal'][stamps[0]]['key'])
            assert all(value(rows['depth'][t]['key']).tobytes() == xyz.tobytes() for t in stamps)
            directions = value(rows['raw'][stamps[0]]['key']+'_directions')
            angles = np.arctan2(directions[:, 1], directions[:, 0])
            measured = np.mean([10.**(value(rows['raw'][t]['key']).astype(float).T/10) for t in stamps], axis=0)
            predicted, correlation = expected_panel(xyz, normal, angles, 400, 10., 10)
            residual, _ = relative_residual(measured, predicted)
            np.testing.assert_allclose(residual, expected['rendered_input_mean_relative_l2'], atol=1e-12)
            np.testing.assert_allclose(correlation, expected['selected_power_correlation'], atol=1e-12)


def verify_sonoptix():
    root = ROOT/'captures/sonoptix'
    receipt = json.loads((root/'observations.json').read_text())
    representation = json.loads((root/'representation.json').read_text())
    assert receipt['passed'] and not receipt['error'] and len(receipt['matched_stamps_ns']) == 25
    assert all(r['exit'] == 0 and not r['forced'] for r in receipt['cleanup'])
    report = load('sonoptix-conditioned-panel.json')
    with np.load(root/'summary.npz', allow_pickle=False) as data:
        assert data['mean_power'].shape == data['second_power_moment'].shape == (256, 599)
        assert len(data['stamp_ns']) == 25 and data['stamp_ns'].tolist() == receipt['matched_stamps_ns']
        assert hashlib.sha256(data['raw_db'].astype('<f4').tobytes()).hexdigest() == representation['raw_payload_sha256']
        stamp = representation['original_stamp_ns']
        validate_calibrated_frame(dict(intensity_db=data['raw_db'], ranges=data['ranges'],
            beam_directions=data['beam_directions'], stamp=[stamp//10**9, stamp % 10**9]), **receipt['calibration'])
        directions = data['beam_directions']
        predicted, correlation = expected_panel(data['xyz'], data['normal_rgb8'],
            np.arctan2(directions[:, 1], directions[:, 0]), 599, 15., 1)
        residual, scaled = relative_residual(data['mean_power'], predicted)
        np.testing.assert_allclose(residual, report['rendered_input_mean_relative_l2'], atol=1e-12)
        np.testing.assert_allclose(correlation, report['selected_power_correlation'], atol=1e-12)
        rmse = np.sqrt(np.mean((data['xyz'][..., 0].astype(float)-4.)**2))
        np.testing.assert_allclose(rmse, report['rendered_wall_x_error_rmse_m'], atol=1e-12)
        tail = load('weak-tail.json')
        mask = data['ranges'] >= 10
        assert int(mask.sum()*256) == tail['cells']
        np.testing.assert_allclose(data['mean_power'][:, mask].mean()/scaled[:, mask].mean(), tail['ratio'], atol=1e-12)
        assert tail['weak_tail_calibration_complete'] is False


def verify_fixed_point():
    report = load('fixed-point-control.json')
    row = next(value for value in report['rows'] if value['frame'] == 20)
    with np.load(ROOT/'captures/sonoptix/summary.npz', allow_pickle=False) as data:
        tail = data['ranges'] >= 10.
    with np.load(ROOT/'fixed-point-frame20.npz', allow_pickle=False) as data:
        assert data['native'].dtype == np.complex64
        assert data['quantized'].dtype == data['floating'].dtype == np.complex128
        actual, quantized, floating = [data[key].astype(complex) for key in ['native', 'quantized', 'floating']]
    assert actual.shape == quantized.shape == floating.shape == (256, 599)
    assert row['tail_cells'] == 256*int(tail.sum()) == 51200
    for key, values in [('native', actual), ('quantized', quantized), ('floating', floating)]:
        np.testing.assert_allclose(np.mean(abs(values[:, tail])**2), row[key+'_tail_mean_power'], rtol=2e-6)
    for key, values in [('quantized', quantized), ('floating', floating)]:
        for prefix, mask in [('whole', slice(None)), ('tail', tail)]:
            error = np.linalg.norm(actual[:, mask]-values[:, mask])/np.linalg.norm(actual[:, mask])
            np.testing.assert_allclose(error, row[prefix+'_complex_residual_'+key], rtol=2e-6)
    assert report['limits']['new_shader_or_estimator_patch'] is False


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--operator-record', type=Path, default=ROOT.parent/'sonar-pr31-36-20261030')
    args = parser.parse_args()
    for name, digest in load('sha256.json').items():
        path = (ROOT/name).resolve()
        assert path.is_relative_to(ROOT) and hashlib.sha256(path.read_bytes()).hexdigest() == digest
    verify_operator(args.operator_record)
    verify_panel()
    verify_sonoptix()
    verify_fixed_point()
    with np.load(ROOT/'all-profile-fits.npz', allow_pickle=False) as data:
        for label, report in load('profile-summary.json').items():
            scores = data[label+'_residual'][data[label+'_has_returns']]
            assert len(scores) == report['nonempty_profiles']
            for q, expected in report['normalized_power_residual_quantiles'].items():
                np.testing.assert_allclose(np.quantile(scores, float(q)), expected, atol=1e-12)
    print('PASS: 32 native transfer controls, 180 controlled raw frames and full-resolution '
          '25-frame Sonoptix statistics/moments, plus the complete matched arithmetic frame. '
          'Field raw extraction and native acquisition are not rerun.')


if __name__ == '__main__':
    main()
