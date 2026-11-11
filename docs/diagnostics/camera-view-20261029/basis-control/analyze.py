from pathlib import Path
import json
import numpy as np
from scipy.spatial.transform import Rotation
import argparse
parser=argparse.ArgumentParser(description="Assess the retained change-of-IMU-basis controls")
parser.add_argument("--output",type=Path,required=True)
args=parser.parse_args()
base=args.output.resolve()
summary=json.loads((base/'summary.json').read_text());baseline={}
for row in summary['reports']:
    folder=base/(row['case']+'-'+str(row['recent_states']))
    states=json.loads((folder/'states.json').read_text());cal=json.loads((folder/'calibration.json').read_text())
    B=Rotation.from_quat(cal['T_i_c_q_xyzw']).as_matrix()
    C=Rotation.from_quat(states[0]['orientation_xyzw']).as_matrix()@B
    traces=[json.loads(line) for line in (folder/'calibration.visual-tracks.jsonl').read_text().splitlines()]
    points={(f['stamp_ns'],t['id']):(np.array(t['point'])@C,t.get('joint_position_sigma_m'))
            for f in traces for t in f['tracks'] if t.get('point') is not None}
    ref=baseline.setdefault(row['recent_states'],points)
    assert points.keys()==ref.keys()
    row.update(triangulated_observations_compared=len(points),
        max_active_point_delta_m=max(float(np.linalg.norm(p-ref[key][0])) for key,(p,sigma) in points.items()),
        max_joint_position_sigma_delta_m=max(abs(sigma-ref[key][1]) for key,(p,sigma) in points.items() if sigma is not None),
        same_visual_track_ids=True,
        archive_equivalence_tested=False)
    row['passed']=(row['max_position_delta_m']<1e-8 and row['max_camera_rotation_delta_rad']<1e-8 and
        row['max_active_point_delta_m']<1e-6 and row['max_joint_position_sigma_delta_m']<1e-6)
    assert row['passed'],row
    print(json.dumps(row),flush=True)
summary['passed']=all(r['passed'] for r in summary['reports'])
(base/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
