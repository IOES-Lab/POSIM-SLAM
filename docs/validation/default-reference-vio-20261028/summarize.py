"""Verify measured exports and equal-input frontend controls with stdlib only."""
import gzip
import hashlib
import json
import math
from pathlib import Path

ROOT = Path(__file__).resolve().parent


def read(name):
    raw = (ROOT/name).read_bytes()
    return json.loads(gzip.decompress(raw) if name.endswith('.gz') else raw)


def quantile(values, fraction):
    index = fraction*(len(values)-1); lo = int(index); hi = min(lo+1, len(values)-1)
    return values[lo]+(values[hi]-values[lo])*(index-lo)


def map_summary(prefix, expected):
    header, raw = (ROOT/prefix/'map.ply').read_bytes().split(b'end_header\n', 1)
    count = int(next(line.split()[-1] for line in header.splitlines()
                     if line.startswith(b'element vertex ')))
    assert b'format binary_little_endian 1.0' in header
    assessment = read(prefix+'/assessment.json.gz'); rows = assessment['per_point']
    assert count == expected == assessment['points'] == len(rows)
    assert len(raw) == 12*count
    assert [r['export_vertex_index'] for r in rows] == list(range(count))
    errors = sorted(r['distance_m'] for r in rows)
    assert all(math.isfinite(e) and e >= 0 for e in errors)
    return dict(points=count, surface_rmse_m=math.sqrt(sum(e*e for e in errors)/count),
                p95_m=quantile(errors, .95), p99_m=quantile(errors, .99),
                maximum_m=errors[-1], above_10cm=sum(e > .1 for e in errors))


def main():
    for name, checksum in read('manifest.json').items():
        assert hashlib.sha256((ROOT/name).read_bytes()).hexdigest() == checksum, name
    results = {}
    for phase in ['initial', 'recovered']:
        receipts = []
        configs = []
        for frontend in ['frame', 'patch']:
            prefix = 'control-'+phase+'-'+frontend
            proof = read(prefix+'/receipt.json.gz'); config = read(prefix+'/calibration.json')
            assert proof['passed'] and proof['complete_map_exported']
            assert proof['frozen_sources_unchanged'] and proof['threads_stopped']
            assert proof['joint_pressure'] and not proof['joint_ping'] and not proof['joint_sonar']
            assert config['max_image_pixels'] == 120000 and config['max_feature_cells'] == 200
            assert proof['ready']['optical_flow_type'] == config['optical_flow_type']
            assert proof['pressure_optimized_states'] > 0
            results[prefix] = map_summary(prefix, proof['export']['points'])
            receipts.append(proof); configs.append(config)
        assert receipts[0]['input_sha256'] == receipts[1]['input_sha256']
        assert receipts[0]['seed_sha256'] == receipts[1]['seed_sha256']
        assert receipts[0]['worker_sha256'] == receipts[1]['worker_sha256']
        assert receipts[0]['camera_frames'] == receipts[1]['camera_frames']
        assert receipts[0]['native_states'] == receipts[1]['native_states']
        assert [k for k in configs[0] if configs[0][k] != configs[1][k]] == ['optical_flow_type']
    for group, reason, ping in [('ros-no-ping', 'imu_stale', False),
                                 ('ros-ping', 'pressure_stale', True)]:
        proof = read(group+'/receipt.json.gz')
        assert proof['passed'] and proof['frozen_sources_unchanged']
        assert proof['joint_ping'] is ping and proof['joint_pressure'] and not proof['joint_sonar']
        assert proof['processing_profile'] == dict(optical_flow_type='patch',
            max_image_pixels=120000, max_feature_cells=200)
        assert proof['native_frontend_ack'] == 'patch'
        assert [f['reason'] for f in proof['failures']] == [reason]
        assert proof['recovery']['old_frame'] != proof['recovery']['new_frame']
        assert proof['recovery']['empty_archive_before_resume']
        assert proof['pressure_fusion']['optimized_states'] > 0
        assert proof['recovery']['pressure_optimized_states'] > 0
        assert all(w['threads_stopped'] for w in proof['cleanup'])
        if not ping:
            assert all('ping' not in topic.lower() for topic in proof['subscribed_topics'])
        for phase, exported in [('initial', proof['visual_export']),
                                 ('recovered', proof['recovery']['export'])]:
            prefix = group+'/'+phase
            results[prefix] = map_summary(prefix, exported['points'])
            comparison = read(prefix+'/comparison.json')
            assert comparison['coordinates_filtered'] is False
            results[prefix]['trajectory_rmse_m'] = comparison['metrics']['position_rmse_m']
    proof = read('managed/receipt.json'); summary = read('managed/summary.json')
    assert proof['passed'] and proof['clean_shutdown']
    assert proof['processing_profile'] == dict(optical_flow_type='patch',
        max_image_pixels=120000, max_feature_cells=200)
    assert summary['passed'] and summary['error'] is None
    assert summary['mission_status'] == 'completed' and summary['segment_stable']
    assert summary['live_poses'] == 1125 and summary['max_transmitted_imu_lead_ns'] == 10_000_000
    assert len(summary['failures']) == 1  # Deliberate worker interruption.
    assert read('managed/lifecycle.json') == dict(exit_code=0, forced_stop=False)
    assert all(not group['live_members'] for group in read('managed/owned-processes.json'))
    results['managed'] = map_summary('managed', proof['native_export']['points'])
    comparison = read('managed/comparison.json')
    assert comparison['coordinates_filtered'] is False
    results['managed']['trajectory_rmse_m'] = comparison['metrics']['position_rmse_m']
    print(json.dumps(dict(evidence_verified=True, general_map_accuracy_qualified=False,
                          maps=results), indent=2))


if __name__ == '__main__':
    main()
