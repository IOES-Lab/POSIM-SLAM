"""Reproduce complete-map error statistics; no estimator input or point filtering."""
import gzip
import hashlib
import json
import math
from pathlib import Path

HERE=Path(__file__).resolve().parent
SOURCE=HERE.parent/'orb-redetection-control-20261028/patch-native-resolution'
raw=(SOURCE/'all-points.json.gz').read_bytes()
assessment=json.loads(gzip.decompress(raw))
errors=sorted(float(row['distance_m']) for row in assessment['per_point'])
if (len(errors)!=assessment['points'] or not errors or
        any(not math.isfinite(value) or value<0 for value in errors)):
    raise ValueError('invalid_complete_map_assessment')

def quantile(p):
    index=(len(errors)-1)*p;low=math.floor(index);high=math.ceil(index)
    return errors[low]+(errors[high]-errors[low])*(index-low)

comparison=json.loads((SOURCE/'comparison.json').read_text())
outliers=sum(value>.1 for value in errors)
result=dict(schema='posim-recorded-demo-map-review-1',
    scope='Retrospective geometry review of one recorded camera/IMU/Ping map',
    metrics=dict(points=len(errors),surface_rmse_m=math.sqrt(math.fsum(value*value for value in errors)/len(errors)),
        median_m=quantile(.5),p95_m=quantile(.95),p99_m=quantile(.99),p999_m=quantile(.999),
        maximum_m=errors[-1],outside_10cm=outliers,outside_10cm_percent=100*outliers/len(errors),
        within_10cm_percent=100*(len(errors)-outliers)/len(errors),
        trajectory_position_rmse_m=comparison['metrics']['position_rmse_m']),
    assessment=dict(recorded_demo_geometry='accepted_with_reported_tail_errors',
        basis='Aggregate accuracy and observed object support; not a zero-outlier requirement',
        user_clarification='Three of 6213 points beyond 10 cm need not fail the complete map',
        universal_marine_threshold=False,operational_acceptance=False,
        hydrographic_compliance_assessed=False,point_removal=False,
        remaining=['initialization','loss_and_recovery','runtime_profiles','other_scenes_and_sensor_combinations']),
    sources={name:hashlib.sha256((SOURCE/name).read_bytes()).hexdigest()
        for name in ['all-points.json.gz','comparison.json','worker-map.ply']})
print(json.dumps(result,indent=2))
