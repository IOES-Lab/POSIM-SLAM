"""Standalone, read-only verification of the public measured-map evidence."""
import gzip
import hashlib
import json
import math
from pathlib import Path
import struct

ROOT=Path(__file__).resolve().parent


def read(name):
    path=(ROOT/name).resolve()
    if not path.is_relative_to(ROOT):raise ValueError('external evidence path')
    raw=path.read_bytes()
    return json.loads(gzip.decompress(raw) if path.suffix=='.gz' else raw)


def verify():
    manifest=read('manifest.json')
    for name,expected in manifest['sha256'].items():
        path=(ROOT/name).resolve()
        assert path.is_relative_to(ROOT),name
        assert hashlib.sha256(path.read_bytes()).hexdigest()==expected,name
    rows=read('maps-summary.json');inputs=None;component=None;runtime=None
    for profile,joint,ping in [('joint-ping',True,True),('receive-only-ping',False,True),('no-ping',False,False)]:
        proof=read(profile+'/receipt.json.gz')
        assert proof['passed'] is True and proof.get('error') is None
        assert proof['frozen_sources_unchanged'] is True
        assert proof['joint_ping'] is joint and proof['joint_pressure'] is True
        assert proof['ping_fusion']['input_enabled'] is ping
        assert proof['ping_fusion']['optimized_states']>0 if joint else proof['ping_fusion']['optimized_states']==0
        assert [failure['reason'] for failure in proof['failures']]==['pressure_stale']
        assert proof['stale_visual_map_cleared'] is True
        assert proof['stale_visual_export_rejected']=='vio_map_unavailable_or_busy'
        assert proof['status_file_cleanup']['running'] is False
        writer=proof['status_file_cleanup']
        assert writer['pending'] is False and writer['error'] is None
        assert writer['written']+writer['coalesced']==writer['submitted']
        assert proof['input_delivery']['rejected']==0
        assert all(w['threads_stopped'] and w['returncode'] in (0,-15) for w in proof['cleanup'])
        if inputs is None:inputs=proof['input_sha256'];component=proof['component_source_files'];runtime=proof['runtime_source_files']
        assert proof['input_sha256']==inputs
        assert proof['component_source_files']==component and proof['runtime_source_files']==runtime
        assert 'practice_slam/status_file.py' in runtime
        assert component['src/posim_slam/visual_seed_consensus.py']==read('benchmark.json')['sha256']['visual_seed_consensus.py']
        for phase in ('initial','recovered'):
            prefix=profile+'/'+phase
            data=(ROOT/prefix/'map.ply').read_bytes();head,body=data.split(b'end_header\n',1)
            lines=head.decode().splitlines()
            count=next(int(line.split()[-1]) for line in lines if line.startswith('element vertex '))
            assert 'format binary_little_endian 1.0' in lines
            assert len(body)==count*12 and all(math.isfinite(v) for point in struct.iter_unpack('<fff',body) for v in point)
            exported=proof['visual_export'] if phase=='initial' else proof['recovery']['export']
            assert count==exported['points'] and 'comment frame_id '+exported['frame'] in lines
            assessment=read(prefix+'/assessment.json.gz');points=assessment['per_point']
            assert assessment['points']==count==len(points)
            assert sorted(p['export_vertex_index'] for p in points)==list(range(count))
            errors=[p['distance_m'] for p in points]
            assert all(math.isfinite(v) and v>=0 for v in errors)
            row=next(r for r in rows if r['profile']==profile and r['phase']==phase)
            assert row['points']==count and row['above_10cm']==sum(v>.1 for v in errors)
            assert math.isclose(row['maximum_m'],max(errors),abs_tol=1e-12)
            assert math.isclose(row['rmse_m'],math.sqrt(math.fsum(v*v for v in errors)/count),abs_tol=1e-12)
    benchmark=read('benchmark.json')
    assert benchmark['passed'] is True and benchmark['truth_input'] is False
    assert benchmark['hypotheses']==256 and benchmark['pixel_sigma']==.5
    assert all(row['max_absolute_numeric_difference']<=1e-10 for row in benchmark['windows'])
    for profile in ('cold-03','cold-04'):
        trial=read(profile+'/trial-result.json');status=read(profile+'/final-status.json.gz')
        assert trial['passed'] is True and not trial['failures'] and not status['failures']
        assert read(profile+'/mission-summary.json')['status']=='completed'
        assert status['ping']['fused_states']>0 and status['pressure']['fused_states']>0
        raw=(ROOT/profile/'map.ply').read_bytes();head,body=raw.split(b'end_header\n',1)
        count=next(int(line.split()[-1]) for line in head.decode().splitlines() if line.startswith('element vertex '))
        assert count==trial['points']==status['landmarks'] and len(body)==count*12
        assert read(profile+'/lifecycle.json')==dict(exit_code=0,forced_stop=False,error=None)
        assert all(not row['live_members'] for row in read(profile+'/owned-process-check.json'))
    long=read('cold-05-long/trial-result.json')
    assert long['passed'] is False and not long['failures'] and long['completed_missions']==3
    assert read('cold-05-long/failed-mission.json.gz')['error']=='navigation_lost'
    print(json.dumps(dict(passed=True,ros_maps=len(rows),assessed_vertices=sum(r['points'] for r in rows),
        all_ros_vertices_evaluated=True,completed_native_surveys=2,long_system_trial_passed=False)))


if __name__=='__main__':verify()
