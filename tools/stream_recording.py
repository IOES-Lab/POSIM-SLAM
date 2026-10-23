"""Exercise the bounded live worker using hash-checked recorded sensor packets.

This tool paces packets by sensor time; the live ROS owner is tested separately.
Only the recorder's lossless grayscale/filter-zero PNG encoding is supported.
"""
import argparse,csv,hashlib,json,runpy,struct,sys,time,zlib
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src'))
from posim_slam.live_client import LiveClient
from posim_slam.map_snapshot import annotate_snapshot


def pixels(path,width,height):
    data=path.read_bytes()
    if data[:8]!=b'\x89PNG\r\n\x1a\n':raise ValueError('not_png')
    offset=8;compressed=bytearray()
    while offset<len(data):
        size=struct.unpack('>I',data[offset:offset+4])[0]
        kind=data[offset+4:offset+8];value=data[offset+8:offset+8+size]
        crc=data[offset+8+size:offset+12+size]
        if len(crc)!=4 or zlib.crc32(kind+value)!=struct.unpack('>I',crc)[0]:raise ValueError('png_crc')
        if kind==b'IHDR' and value!=struct.pack('>IIBBBBB',width,height,8,0,0,0,0):raise ValueError('png_shape')
        if kind==b'IDAT':compressed.extend(value)
        offset+=size+12
    decoder=zlib.decompressobj();raw=decoder.decompress(compressed,(width+1)*height+1)
    if len(raw)!=(width+1)*height or not decoder.eof:raise ValueError('png_size')
    if any(raw[y*(width+1)]!=0 for y in range(height)):raise ValueError('png_filter')
    return b''.join(raw[y*(width+1)+1:(y+1)*(width+1)] for y in range(height))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    for name in ('binary','inputs','recording-receipt','output'):p.add_argument('--'+name,type=Path,required=True)
    p.add_argument('--speed',type=float,default=1)
    a=p.parse_args()
    if a.output.exists() or not .5<=a.speed<=2:p.error('Use a new output and speed 0.5–2')
    root=a.inputs.resolve();receipt=json.loads(a.recording_receipt.read_text())
    runpy.run_path(str(Path(__file__).with_name('replay_vio.py')))['verify'](root,receipt)
    config=json.loads((root/'calibration.json').read_text())
    frames=json.loads((root/'frames.json').read_text())
    with (root/'imu.csv').open() as f:imu=list(csv.DictReader(f))
    events=[(int(r['stamp_ns']),0,r) for r in imu]+[(r['stamp_ns'],1,r) for r in frames]
    events.sort(key=lambda r:(r[0],r[1]));a.output.mkdir(parents=True)
    states=[];maps=[];ready=[];exports=[];requested=[]
    calibration=a.output/'calibration.json';calibration.write_bytes((root/'calibration.json').read_bytes())
    def poll():
        for value in client.poll():
            if value['type']=='state':states.append(value)
            elif value['type']=='map':
                maps.append(value)
                if value['landmarks_window']>1000 and 1 not in requested:
                    client.snapshot(1);requested.append(1)
            elif value['type']=='export':
                assert value['success'],value
                data=annotate_snapshot((a.output/'worker-map.ply').read_bytes(),nonce='recorded',
                    frame='recorded/vio_segment_0',request_id=value['request_id'],stamp_ns=value['stamp_ns'],points=value['points'])
                (a.output/('map-%d.ply'%value['request_id'])).write_bytes(data);exports.append(value)
            else:ready.append(value)
    with (a.output/'worker.log').open('w') as log:
        client=LiveClient(a.binary,calibration,log)
        try:
            deadline=time.monotonic()+3
            while not client.ready.is_set():
                poll()
                if time.monotonic()>=deadline:raise RuntimeError('worker_ready_timeout')
                time.sleep(.005)
            begin=time.monotonic();first=events[0][0]
            for stamp,kind,row in events:
                due=begin+(stamp-first)*1e-9/a.speed
                while time.monotonic()<due:
                    poll();time.sleep(min(.005,max(0,due-time.monotonic())))
                poll()
                if kind==0:client.imu(stamp,[float(row[k]) for k in ('gx','gy','gz')],[float(row[k]) for k in ('ax','ay','az')])
                else:client.image(stamp,config['width'],config['height'],pixels(root/row['file'],config['width'],config['height']))
            deadline=time.monotonic()+5
            while time.monotonic()<deadline and (len(states)<len(frames) or len(maps)<len(frames)):
                poll();time.sleep(.005)
            assert len(states)==len(frames) and len(maps)==len(frames),(len(states),len(maps))
            assert ready and ready[0]['inertial_fused'] is True
            assert all(len(m['points'])<=1000 for m in maps)
            client.snapshot(2);requested.append(2);deadline=time.monotonic()+2
            while time.monotonic()<deadline and len(exports)<len(requested):poll();time.sleep(.005)
            assert len(exports)==len(requested)
            final=next(e for e in exports if e['request_id']==2)
            assert final['points']==maps[-1]['landmarks_window'] and final['stamp_ns']==maps[-1]['stamp_ns']
        finally:client.close()
    with (a.output/'states.csv').open('w') as f:
        w=csv.writer(f,lineterminator='\n');w.writerow(['stamp_ns','x','y','z','qx','qy','qz','qw','vx','vy','vz','bgx','bgy','bgz','bax','bay','baz'])
        for s in states:w.writerow([s['stamp_ns'],*s['position'],*s['orientation_xyzw'],*s['velocity'],*s['gyro_bias'],*s['accel_bias']])
    proof=dict(passed=True,scope='Paced recorded packets through the live worker; ROS ownership separate',
               states=len(states),maps=len(maps),exports=exports,max_points=max(len(m['points']) for m in maps),
               final_landmarks_window=maps[-1]['landmarks_window'],worker_ready=ready,
               binary_sha256=hashlib.sha256(a.binary.read_bytes()).hexdigest(),
               worker_returncode=client.process.returncode,worker_threads_stopped=not(client.reader.is_alive() or client.writer.is_alive()))
    (a.output/'receipt.json').write_text(json.dumps(proof,indent=2)+'\n');print(json.dumps(proof))
    (a.output/'map-support.json').write_text(json.dumps([dict(stamp_ns=m['stamp_ns'],points=len(m['points']),
         observations=m['observations'],landmarks_window=m['landmarks_window']) for m in maps],indent=2)+'\n')
