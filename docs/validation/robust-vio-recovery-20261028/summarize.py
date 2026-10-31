"""Summarize complete exported maps; standard library only, no point removal."""
import gzip
import json
import math
from pathlib import Path

root = Path(__file__).resolve().parent


def read(folder, name):
    path = folder/name
    if path.is_file():
        return json.loads(path.read_text())
    return json.loads(gzip.decompress(path.with_suffix(path.suffix+'.gz').read_bytes()))


report = dict(schema='posim-robust-vio-recovery-review-1',
    scope='Recorded ROS initialization and IMU-loss recovery; no release/platform qualification',
    universal_marine_threshold=False, points_removed=False, cases={})
for folder in sorted(path for path in root.iterdir() if path.is_dir()):
    if not (folder/'receipt.json.gz').is_file():
        continue
    receipt = read(folder, 'receipt.json')
    case = dict(ros_passed=receipt['passed'], error=receipt.get('error'),
        replay_speed=receipt['replay_speed'], source_frozen=receipt['frozen_sources_unchanged'],
        initialization=read(folder, 'segment-1-bootstrap.json'),
        recovery=receipt.get('recovery'), cleanup=receipt['cleanup'], maps={})
    for phase, suffix in (('initial',''), ('recovered','-recovered')):
        if not (folder/('visual-surface-evaluation'+suffix+'.json.gz')).is_file():
            continue
        surface = read(folder, 'visual-surface-evaluation'+suffix+'.json')
        errors = sorted(value['distance_m'] for value in surface['per_point'])
        if len(errors) != surface['points'] or not errors:
            raise ValueError('complete_map_required')
        comparison = read(folder, 'comparison'+suffix+'.json')
        if comparison['total_points'] != len(errors):
            raise ValueError('complete_map_count_mismatch')
        quantile = lambda fraction: errors[max(0, math.ceil(len(errors)*fraction)-1)]
        case['maps'][phase] = dict(points=len(errors),
            surface_rmse_m=math.sqrt(math.fsum(value*value for value in errors)/len(errors)),
            median_m=quantile(.5), p95_m=quantile(.95), p99_m=quantile(.99),
            maximum_m=errors[-1], outside_10cm=sum(value>.1 for value in errors),
            outside_10cm_percent=100*sum(value>.1 for value in errors)/len(errors),
            trajectory_rmse_m=comparison['metrics']['position_rmse_m'],
            rpe_1s_position_rmse_m=comparison['metrics']['rpe_1s_position_rmse_m'],
            surface_support_within_10cm=surface['diagnostic_thresholds_m']['0.1']['nearest_surface_counts'])
    report['cases'][folder.name] = case
(root/'summary.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps({name:dict(ros_passed=value['ros_passed'],maps=value['maps'])
                  for name,value in report['cases'].items()},indent=2))
