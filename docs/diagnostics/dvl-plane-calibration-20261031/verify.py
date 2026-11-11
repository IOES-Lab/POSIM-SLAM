"""Reproduce the conditional calibration from every original range packet."""
from pathlib import Path
import hashlib
import json
import gzip
import xml.etree.ElementTree as ET
import numpy as np
from posim_slam.dvl_surface import cone_plane_range_factor
from posim_slam.range_direction import calibrate_return_directions

ROOT = Path(__file__).resolve().parent


def normal(case):
    roll, pitch = np.deg2rad([case['roll_deg'], case['pitch_deg']])
    cr, sr, cp, sp = np.cos(roll), np.sin(roll), np.cos(pitch), np.sin(pitch)
    rotation = np.array([[cp, sp*sr, sp*cr], [0., cr, -sr], [-sp, cp*sr, cp*cr]])
    return rotation.T @ np.array([0., 0., -1.])


def original_ranges(document, case):
    rows = [row for row in document['rows'] if row['case'] == case]
    assert len(rows) == 26
    ranges = []
    for row in rows:
        message = row['message']
        assert message['target']['type'] == 'DVL_TARGET_BOTTOM'
        assert 'variance' not in message['target']['range']
        assert len(row['protobuf_sha256']) == 64
        beams = sorted(message['beams'], key=lambda b: b['id'])
        assert [beam['id'] for beam in beams] == [1, 2, 3, 4]
        assert all(beam['locked'] and 'variance' not in beam['range'] for beam in beams)
        ranges.append([beam['range']['mean'] for beam in beams])
    return np.array(ranges)


def metrics(errors):
    return dict(count=int(errors.size), rmse_m=float(np.sqrt(np.mean(errors**2))),
                max_abs_error_m=float(np.max(np.abs(errors))))


def check_fixture_xml(root, fixture):
    ns = '{http://gazebosim.org}'
    for filename, cases, digest_key in [
        ('reference-world.sdf', fixture['reference_cases'], 'original_world_sha256'),
        ('unused-world.sdf', fixture['unused_cases'], 'unused_world_sha256')]:
        path = root/filename
        assert hashlib.sha256(path.read_bytes()).hexdigest() == fixture[digest_key]
        assert path.read_text().count('gz:type="dvl"') == len(cases)
        world = ET.parse(path).getroot().find('world')
        for case in cases:
            model = world.find(f"model[@name='{case['name']}']")
            pose = np.fromstring(model.findtext('pose'), sep=' ')
            np.testing.assert_allclose(pose[2:5], [case['height_m'], *np.deg2rad([case['roll_deg'], case['pitch_deg']])], rtol=0, atol=2e-15)
            assert model.findtext('static') == 'true'
            sensor = model.find('link/sensor')
            assert sensor.get(ns+'type') == 'dvl'
            assert sensor.findtext('pose') == '0 0 0 0 0 0'
            assert float(sensor.findtext('update_rate')) == fixture['settings']['rate_hz']
            assert sensor.findtext('topic') == case['topic']
            dvl = sensor.find(ns+'dvl')
            for tag, key in [('resolution', 'resolution_m_at_1m'), ('minimum_range', 'minimum_range_m'),
                              ('maximum_range', 'maximum_range_m')]:
                assert float(dvl.findtext(tag)) == fixture['settings'][key]
            assert float(dvl.findtext('tracking/bottom_mode/noise/stddev')) == fixture['settings']['velocity_noise_stddev_m_s']
            for i, beam in enumerate(dvl.findall('arrangement/beam')):
                assert int(beam.get('id')) == i+1
                assert float(beam.findtext('rotation')) == fixture['settings']['rotations_deg'][i]
                assert float(beam.findtext('tilt')) == fixture['settings']['tilt_deg']
                assert float(beam.findtext('aperture')) == fixture['settings']['aperture_deg']
    # Prefix spelling differs, but the parsed names, attributes and values do
    # not. Preserve the zero-packet failure rather than patch the product.
    def expanded(element):
        return (element.tag, sorted(element.attrib.items()), (element.text or '').strip(),
                [expanded(child) for child in element])
    assert 'ns0:type="dvl"' in (root/'failed-world.sdf').read_text()
    assert expanded(ET.parse(root/'failed-world.sdf').getroot()) == expanded(ET.parse(root/'unused-world.sdf').getroot())


def reproduce(root=ROOT):
    def read(name):
        path = root/name
        if path.suffix == '.gz':
            return json.loads(gzip.decompress(path.read_bytes()))
        return json.loads(path.read_text())
    for name, digest in read('sha256.json').items():
        assert hashlib.sha256((root/name).read_bytes()).hexdigest() == digest, name
    fixture = read('fixtures.json')
    for name, digest in read('data-identities.json').items():
        assert hashlib.sha256(gzip.decompress((root/name).read_bytes())).hexdigest() == digest
    reference, unused = read('observations-reference.json.gz'), read('observations-unused.json.gz')
    saved = read('reference-assessment.json.gz')
    frozen = read('frozen-directions.json')
    nominal = read('nominal-sampling-control.json.gz')
    held = read('unused-assessment.json.gz')
    assert len(fixture['reference_cases']) == 9 and len(fixture['unused_cases']) == 6
    assert fixture['no_sdk_or_physical_settings_changed']
    assert fixture['source_plane_is_calibration_fixture']
    assert fixture['physical_branch_outside_tested_controls_unqualified']
    check_fixture_xml(root, fixture)
    assert hashlib.sha256((root/'frozen-directions.json').read_bytes()).hexdigest() == fixture['frozen_before_unused_sha256']
    for document, count in [(reference, 234), (unused, 156)]:
        assert document['passed'] and not document['error'] and not document['callback_errors']
        assert document['cleanup'] == dict(exit=0, forced=False)
        assert len(document['rows']) == count and set(document['counts'].values()) == {26}
    assert not frozen['map_qualified'] and not frozen['native_fusion']
    assert not frozen['range_sigma_available'] and not frozen['pose_sigma_available']
    assert frozen['uses_first_packet_only'] and frozen['training_cases'] == [0, 1, 2]
    assert not held['ray_calibration_refitted'] and not held['range_variance_invented']
    first = fixture['reference_cases'][:3]
    n = np.array([normal(case) for case in first])
    h = np.array([case['height_m'] for case in first])
    observed = np.array([original_ranges(reference, case['index'])[0] for case in first])
    fitted = calibrate_return_directions(plane_normals=n, plane_distances_m=h,
                                         measured_ranges_m=observed)
    assert fitted['available'] and not fitted['covariance_available']
    assert not fitted['physical_branch_verified'] and not fitted['map_qualified']
    rays = np.array([beam['ray'] for beam in frozen['beams']])
    # Derived linear algebra may differ by roundoff. The frozen rays remain
    # the actual validation input; no original observation is replaced.
    np.testing.assert_allclose(fitted['rays'], rays, rtol=0, atol=2e-14)
    np.testing.assert_allclose(fitted['linear_solution_norms'],
                               [beam['linear_solution_norm'] for beam in frozen['beams']], rtol=0, atol=2e-14)
    reference_rows = {(r['case'], r['packet'], r['beam_id']): r for r in saved['all_beam_observations']}
    nominal_rows = {(r['case'], r['packet'], r['beam_id']): r for r in nominal['all_ranges']}
    assert len(reference_rows) == len(nominal_rows) == 936
    tilt = np.deg2rad(fixture['settings']['tilt_deg'])
    angle = np.deg2rad(fixture['settings']['rotations_deg'])
    axes = np.column_stack((-np.sin(tilt)*np.cos(angle), np.sin(tilt)*np.sin(angle),
                            np.full(4, -np.cos(tilt))))
    cases = []; groups = dict(training=[], repeated_training_planes=[], unused_distances=[], unused_orientations=[])
    unused_cursor = 0
    for kind, source, geometry in [('reference', reference, fixture['reference_cases']),
                                    ('unused', unused, fixture['unused_cases'])]:
        for case in geometry:
            n = normal(case); height = case['height_m']
            ranges = original_ranges(source, case['index'])
            predicted = height/(n@rays.T)
            errors = ranges-predicted
            row = dict(group=kind, case=case['index'], height_m=height, roll_deg=case['roll_deg'],
                       pitch_deg=case['pitch_deg'], calibrated=metrics(errors))
            if kind == 'reference':
                cones = np.array([cone_plane_range_factor(position=np.zeros(3), attitude=np.eye(3),
                    imu_to_dvl=np.zeros(3), dvl_to_imu=np.eye(3), beam_axis=axis,
                    aperture_rad=np.deg2rad(fixture['settings']['aperture_deg']), plane_normal=n,
                    plane_distance=height, measured_range=ranges[0, i], range_min=.1,
                    range_max=20.)['predicted_range'] for i, axis in enumerate(axes)])
                sampled = np.array([np.min(height/(np.array(p['rays'])@n)) for p in nominal['profile']])
                row.update(continuous_cone=metrics(ranges-cones), nominal_sampled=metrics(ranges-sampled))
                for packet, values in enumerate(ranges):
                    for beam, value in enumerate(values, 1):
                        old = reference_rows[(case['index'], packet, beam)]
                        sample = nominal_rows[(case['index'], packet, beam)]
                        assert old['observed_range_m'] == sample['observed_range_m'] == value
                        np.testing.assert_allclose(cones[beam-1], old['predicted_continuous_cone_range_m'], rtol=0, atol=4e-14)
                        np.testing.assert_allclose(sampled[beam-1], sample['predicted_sampled_range_m'], rtol=0, atol=4e-14)
                if case['index'] < 3:
                    groups['training'].extend(errors[0]); groups['repeated_training_planes'].extend(errors[1:].ravel())
                else:
                    groups['unused_distances'].extend(errors.ravel())
            else:
                groups['unused_orientations'].extend(errors.ravel())
                for packet in ranges:
                    for beam, value in enumerate(packet, 1):
                        old = held['all_observations'][unused_cursor]; unused_cursor += 1
                        assert old['case'] == case['index'] and old['beam'] == beam and old['observed'] == value
                        np.testing.assert_allclose(predicted[beam-1], old['predicted'], rtol=0, atol=4e-14)
            cases.append(row)
    assert unused_cursor == len(held['all_observations']) == 624
    assert [len(groups[key]) for key in groups] == [12, 300, 624, 624]
    failure = read('fixture-setup-failure.json')
    assert not failure['passed'] and sum(failure['counts'].values()) == 0
    assert failure['cleanup'] == dict(exit=0, forced=False)
    return dict(scope='Static known-plane forward-model residuals only; no map or hardware accuracy claim',
                cases=cases, groups={k: metrics(np.array(v)) for k, v in groups.items()},
                original_packets=390, original_beam_ranges=1560, calibration_ranges=12,
                covariance_available=False, native_fusion=False, map_qualified=False)


if __name__ == '__main__':
    actual = reproduce()
    expected = json.loads((ROOT/'summary.json').read_text())
    assert actual.keys() == expected.keys()
    assert actual['groups'].keys() == expected['groups'].keys()
    for a, b in zip(actual['cases'], expected['cases']):
        assert a.keys() == b.keys()
        for key in a:
            if isinstance(a[key], dict):
                assert a[key]['count'] == b[key]['count']
                np.testing.assert_allclose([a[key]['rmse_m'], a[key]['max_abs_error_m']],
                    [b[key]['rmse_m'], b[key]['max_abs_error_m']], rtol=0, atol=4e-14)
            else:
                assert a[key] == b[key]
    for key in actual['groups']:
        assert actual['groups'][key]['count'] == expected['groups'][key]['count']
        np.testing.assert_allclose([actual['groups'][key][field] for field in ['rmse_m', 'max_abs_error_m']],
                                  [expected['groups'][key][field] for field in ['rmse_m', 'max_abs_error_m']], rtol=0, atol=4e-14)
    for key in ['scope', 'original_packets', 'original_beam_ranges', 'calibration_ranges',
                'covariance_available', 'native_fusion', 'map_qualified']:
        assert actual[key] == expected[key]
    print('PASS: all 390 packets / 1,560 beam ranges; 12 training ranges; frozen unused-plane predictions; uncertainty remains unavailable')
