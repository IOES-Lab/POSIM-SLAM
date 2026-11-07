"""Verify exact exports, all-point statistics and transport/recovery evidence."""
import gzip
import hashlib
import json
import math
from pathlib import Path

ROOT = Path(__file__).resolve().parent


def read(name):
    raw = (ROOT/name).read_bytes()
    return json.loads(gzip.decompress(raw) if name.endswith('.gz') else raw)


def percentile(values, fraction):
    index = fraction*(len(values)-1)
    lo = int(index); hi = min(lo+1, len(values)-1)
    return values[lo]+(values[hi]-values[lo])*(index-lo)


def main():
    for name, checksum in read('manifest.json').items():
        assert hashlib.sha256((ROOT/name).read_bytes()).hexdigest() == checksum, name
    before = read('before-failure-summary.json')
    assert before['observed_failure'] == 'worker:imu_backlog'
    assert before['native_queue_capacity_samples'] == 256
    assert before['last_pre_reset_input_lead_s'] > 2.56
    managed = read('managed-proof.json'); field = read('managed-summary.json')
    assert managed['passed'] and managed['clean_shutdown']
    assert managed['joint_pressure'] and managed['joint_ping'] and not managed['joint_sonar']
    assert managed['pressure_fusion']['optimized_states'] > 0
    assert field['passed'] and field['error'] is None
    assert field['mission']['segment_stable'] and field['mission_status'] == 'completed'
    assert field['mission']['live_poses'] == 1125
    assert field['maximum_native_imu_lead_ns'] == 10_000_000
    assert len(field['failures']) == 1  # Deliberately killed startup worker.
    assert read('managed-lifecycle.json') == dict(exit_code=0, forced_stop=False)
    assert all(not group['live_members'] for group in read('managed-owned-processes.json'))
    high = read('ros-high-proof.json.gz'); default = read('default-no-ping-proof.json.gz')
    for proof, reason in [(high, 'pressure_stale'), (default, 'imu_stale')]:
        assert proof['passed'] and proof['frozen_sources_unchanged']
        assert 'src/posim_slam/event_imu.py' in proof['component_source_files']
        assert [f['reason'] for f in proof['failures']] == [reason]
        assert proof['recovery']['old_frame'] != proof['recovery']['new_frame']
        assert proof['recovery']['empty_archive_before_resume']
        assert proof['pressure_fusion']['optimized_states'] > 0
        assert proof['recovery']['pressure_optimized_states'] > 0
        assert all(worker['threads_stopped'] for worker in proof['cleanup'])
    assert not default['joint_ping'] and not default['joint_sonar'] and default['joint_pressure']
    assert default['processing_profile']['max_image_pixels'] == 120000
    assert default['processing_profile']['max_feature_cells'] == 200
    assert default['processing_profile']['optical_flow_type'] == 'frame_to_frame'
    assert all('ping' not in topic.lower() for topic in default['subscribed_topics'])
    results = {}
    for prefix in ['managed', 'default-initial', 'default-recovered']:
        header, raw = (ROOT/(prefix+'-map.ply')).read_bytes().split(b'end_header\n', 1)
        count = int(next(line.split()[-1] for line in header.splitlines()
                         if line.startswith(b'element vertex ')))
        assessment = read(prefix+'-surface.json.gz')
        comparison = read(prefix+'-comparison.json')
        rows = assessment['per_point']
        assert b'format binary_little_endian 1.0' in header
        assert len(raw) == 12*count and len(rows) == count == assessment['points']
        assert comparison['total_points'] == count and comparison['coordinates_filtered'] is False
        assert [row['export_vertex_index'] for row in rows] == list(range(count))
        errors = sorted(row['distance_m'] for row in rows)
        assert all(math.isfinite(v) and v >= 0 for v in errors)
        results[prefix] = dict(points=count,
            surface_rmse_m=math.sqrt(sum(v*v for v in errors)/count),
            median_m=percentile(errors, .5), p95_m=percentile(errors, .95),
            p99_m=percentile(errors, .99), maximum_m=errors[-1],
            outside_10cm=sum(v > .1 for v in errors),
            trajectory_rmse_m=comparison['metrics']['position_rmse_m'])
    assert results['managed']['points'] == managed['native_export']['points'] == 9138
    assert results['default-initial']['points'] == default['visual_export']['points'] == 1417
    assert results['default-recovered']['points'] == default['recovery']['export']['points'] == 1197
    print(json.dumps(dict(evidence_verified=True, map_accuracy_qualified=False,
                          maps=results), indent=2))


if __name__ == '__main__':
    main()
