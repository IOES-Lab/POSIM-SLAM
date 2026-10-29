"""Read-only actual conditional recovery; live-state uncertainty excluded."""
import hashlib,json,time
from pathlib import Path
import numpy as np
from posim_slam.retired_pose_archive import RetiredPoseArchive
base=Path(__file__).resolve().parent;root=base.parents[1]
packets=json.loads((base/'archive-packets.json').read_text())
audit=root/'libraries/posim-slam/docs/diagnostics/joint-surface-ranges-20261029/surface-pose-retention.json'
rows=json.loads(audit.read_text())['rows'];by_stamp={}
for i,row in enumerate(rows):by_stamp.setdefault(row['snapshot_stamp_ns'],[]).append((i,row))
a=RetiredPoseArchive(packets[0]['epoch']);results=[];failures=[];peak_nodes=peak_matrices=peak_workspace=0;start=time.perf_counter()
for packet in packets:
 try:
  a.ingest(packet)
 except Exception as error:
  failures.append(dict(stamp_ns=packet['stamp_ns'],phase='ingest',error=repr(error)));break
 peak_nodes=max(peak_nodes,len(a.nodes));peak_matrices=max(peak_matrices,sum(n.matrix_bytes for n in a.nodes.values()))
 for i,row in by_stamp.get(packet['stamp_ns'],[]):
  try:
   got=a.recover(row['required_pose_stamps_ns']);free=got['unbounded_basis'];cov=got['observable_conditional_covariance']
   results.append(dict(row=i,stamp_ns=packet['stamp_ns'],pose_stamps=row['required_pose_stamps_ns'],
    retired_sonar_pose=row['sonar_stamp_ns'] in row['missing_pose_stamps_ns'],
    maximum_conditional_position_sigma_m=float(np.sqrt(max(0,np.max(np.diag(cov).reshape(-1,6)[:,:3])))),
    maximum_conditional_rotation_sigma_rad=float(np.sqrt(max(0,np.max(np.diag(cov).reshape(-1,6)[:,3:])))),
    conditional_covariance_cross_norm=float(np.linalg.norm(cov-np.diag(np.diag(cov)))),
    unbounded_direction_columns=free.shape[1],unbounded_pose_norm=float(np.linalg.norm(free)),
    joint_covariance_dimension=cov.shape[0]))
   peak_workspace=max(peak_workspace,a._workspace_bytes())
  except Exception as error:failures.append(dict(row=i,stamp_ns=packet['stamp_ns'],phase='recover',error=repr(error)))
 # A fixed diagnostic 30 s query horizon matches the source audit's longest lag.
 # Protect the exact cutoff boundary of already-admitted support, never change
 # the live estimator window or admit a late optimizer measurement here.
 cutoff=max(a.closed_through_ns,packet['stamp_ns']-30_000_000_000)
 protection={('state',t) for _,r in by_stamp.get(packet['stamp_ns'],[]) for t in r['required_pose_stamps_ns'] if t<=cutoff}
 a.expire_through(cutoff,protect=protection)
summary=dict(expected_predictions=len(rows),recovered_predictions=len(results),failures=failures,
 recovered_retired_sonar_predictions=sum(r['retired_sonar_pose'] for r in results),
 peak_nodes=peak_nodes,peak_matrix_bytes=peak_matrices,peak_recovery_workspace_bytes=peak_workspace,
 elapsed_s=time.perf_counter()-start,live_coordinates_treated_as_known=True,
 independent_pose_observations=False,late_factor_installed=False,native_state_updated=False,map_qualified=False,
 source_sha256={str(p.relative_to(root)):hashlib.sha256(p.read_bytes()).hexdigest() for p in (audit,base/'archive-packets.json',Path(__file__))},rows=results)
(base/'recovery.json').write_text(json.dumps(summary,indent=2)+'\n')
print(json.dumps({k:v for k,v in summary.items() if k not in ('rows','source_sha256')},indent=2))
if failures or len(results)!=len(rows):raise SystemExit(1)
