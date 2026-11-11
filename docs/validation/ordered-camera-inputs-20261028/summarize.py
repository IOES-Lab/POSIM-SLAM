"""Verify retained exports, every-point statistics and failed-field evidence."""
import gzip
import hashlib
import json
import math
from pathlib import Path

ROOT = Path(__file__).resolve().parent


def read(name):
    path = ROOT/name
    raw = path.read_bytes()
    return json.loads(gzip.decompress(raw) if name.endswith('.gz') else raw)


def percentile(values, fraction):
    position = fraction*(len(values)-1)
    lo = int(position)
    hi = min(lo+1, len(values)-1)
    return values[lo]+(values[hi]-values[lo])*(position-lo)


def main():
    manifest = read('manifest.json')
    for name, expected in manifest.items():
        assert hashlib.sha256((ROOT/name).read_bytes()).hexdigest() == expected, name
    proof = read('receipt.json.gz')
    assert proof['passed'] and proof['frozen_sources_unchanged']
    assert len(proof['component_source_files']) == 74
    assert len(proof['runtime_source_files']) == 9
    assert [v['reason'] for v in proof['failures']] == ['imu_stale']
    assert len(proof['cleanup']) == 2
    assert all(v['threads_stopped'] for v in proof['cleanup'])
    assert proof['input_delivery']['rejected'] == 0
    assert proof['recovery']['new_frame'] != proof['recovery']['old_frame']
    results = {}
    for phase, suffix, ply in [('initial', '', 'initial-map.ply'),
                               ('recovered', '-recovered', 'recovered-map.ply')]:
        assessment = read('visual-surface-evaluation'+suffix+'.json.gz')
        comparison = read('comparison'+suffix+'.json.gz')
        header, data = (ROOT/ply).read_bytes().split(b'end_header\n', 1)
        assert b'format binary_little_endian 1.0' in header
        count = int(next(line.split()[-1] for line in header.splitlines()
                         if line.startswith(b'element vertex ')))
        rows = assessment['per_point']
        assert len(data) == 12*count and len(rows) == count == assessment['points']
        assert comparison['total_points'] == count
        assert [row['export_vertex_index'] for row in rows] == list(range(count))
        assert comparison['coordinates_filtered'] is False
        distances = sorted(row['distance_m'] for row in rows)
        assert all(math.isfinite(v) and v >= 0 for v in distances)
        results[phase] = dict(points=count,
            surface_rmse_m=math.sqrt(sum(v*v for v in distances)/count),
            p95_m=percentile(distances, .95), p99_m=percentile(distances, .99),
            maximum_m=max(distances), outside_10cm=sum(v > .1 for v in distances),
            trajectory_rmse_m=comparison['metrics']['position_rmse_m'])
    field = read('field-live-vio-probe.json.gz')
    assert field['passed'] is False and field['error']['type'] == 'AssertionError'
    assert field['startup']['mission_started_before_tracking'] is True
    assert field['recovery']['new_pose_count'] >= 5
    assert field['mission']['segment_stable'] is True
    assert field['panel']['snapshot']['available'] is False
    assert max(s.get('landmarks', 0) for s in field['statuses']) == 0
    lifecycle = read('field-lifecycle.json.gz')
    assert lifecycle == dict(exit_code=0, forced_stop=False)
    assert all(not group['live_members'] for group in read('field-owned-process-check.json.gz'))
    motion = read('field-motion-assessment.json.gz')
    assert motion['accepted'] is True
    print(json.dumps(dict(passed=True, maps=results,
        field_map_accepted=False, field_motion_rmse_m=motion['metrics']['position_rmse_m']), indent=2))


if __name__ == '__main__':
    main()
