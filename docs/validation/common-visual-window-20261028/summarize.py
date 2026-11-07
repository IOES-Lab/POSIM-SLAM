"""Verify retained evidence and recompute complete-map statistics with Python."""
import gzip
import hashlib
import json
import math
from pathlib import Path

ROOT=Path(__file__).resolve().parent
def read(path):
    path=ROOT/path
    return json.loads(gzip.decompress(path.read_bytes()) if path.suffix=='.gz' else path.read_bytes())

def quantile(values,q):
    position=(len(values)-1)*q;i=int(position);j=min(i+1,len(values)-1)
    return values[i]+(values[j]-values[i])*(position-i)

def summarize():
    result={}
    for case,suffix,filename in (
            ('shared-early-window3','','worker-map.ply'),
            ('shared-early-window16','','worker-map.ply'),
            ('shared-late-window16','','worker-map.ply'),
            ('shared-ping-ros-1x','','initial-map.ply'),
            ('shared-ping-ros-1x','-recovered','map.ply')):
        folder=ROOT/case;receipt=read(Path(case)/'receipt.json.gz')
        comparison=read(Path(case)/('comparison'+suffix+'.json'))
        assessment=read(Path(case)/('visual-surface-evaluation'+suffix+'.json.gz'))
        raw=(folder/filename).read_bytes();digest=hashlib.sha256(raw).hexdigest()
        assert digest==comparison['input_sha256']['measured_ply']
        assert digest in assessment['source_sha256'].values()
        header,end=raw.split(b'end_header\n',1)
        count=int(next(line.split()[2] for line in header.splitlines() if line.startswith(b'element vertex ')))
        errors=sorted(v['distance_m'] for v in assessment['per_point'])
        assert len(end)==12*count and len(errors)==count==comparison['total_points']
        assert receipt['passed'] and comparison['metrics']['evaluation_alignment']['scale']==1
        if case!='shared-ping-ros-1x':
            events=read(Path(case)/'events.json.gz');states=[v for v in events if v['type']=='state']
            assert len(states)==receipt['native_states']==receipt['camera_frames']
            assert receipt['ping_fused_states']==sum(bool(v['ping_fused']) for v in states)==0
            assert all(v['sonar'] is None and v['dvl'] is None and v['pressure'] is None for v in states)
            assert receipt['frozen_sources_unchanged'] and receipt['threads_stopped']
        else:
            assert receipt['ros_domain_id']==116
            assert all(v['threads_stopped'] for v in receipt['cleanup'])
            assert [v['reason'] for v in receipt['failures']]==['imu_stale']
        result[case+(suffix or '-initial')]=dict(points=count,
            surface_rmse_m=math.sqrt(sum(v*v for v in errors)/count),
            surface_p95_m=quantile(errors,.95),surface_p99_m=quantile(errors,.99),
            surface_max_m=errors[-1],beyond_10cm=sum(v>.1 for v in errors),
            trajectory_rmse_m=comparison['metrics']['position_rmse_m'],
            nearest_surfaces_within_10cm=assessment['diagnostic_thresholds_m']['0.1']['nearest_surface_counts'])
    return result

if __name__=='__main__':
    manifest=read(Path('checksums.json'))
    for name,digest in manifest.items():
        assert hashlib.sha256((ROOT/name).read_bytes()).hexdigest()==digest,name
    result=summarize()
    assert result==read(Path('summary.json'))
    print(json.dumps(result,indent=2))
