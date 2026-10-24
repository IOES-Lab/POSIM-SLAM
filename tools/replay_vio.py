"""Verify recorded input hashes and run the independent pixel/raw-IMU replay.

No truth input, application service, SDK modification or runtime selection.
"""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def verify(inputs, receipt):
    if receipt.get('passed') is not True:
        raise ValueError('recording_not_qualified')
    hashes=receipt.get('input_files',{})
    base={'calibration.json','frames.json','imu.csv'}
    if set(hashes) not in (base,base|{'ping-calibration.json','ping.csv'}):
        raise ValueError('recording_hash_inventory_missing')
    for name, expected in hashes.items():
        path=inputs/name
        if path.stat().st_size>64*1024*1024 or digest(path)!=expected:
            raise ValueError('recording_input_changed:'+name)
    frames=json.loads((inputs/'frames.json').read_text())
    if not 5<=len(frames)<=900:raise ValueError('recording_frame_limit')
    size=0
    for frame in frames:
        path=(inputs/frame['file']).resolve(strict=True)
        if not path.is_relative_to(inputs):raise ValueError('recording_frame_outside_inputs')
        size+=path.stat().st_size
        if path.stat().st_size>10*1024*1024 or size>512*1024*1024:
            raise ValueError('recording_pixel_limit')
        if digest(path)!=frame['sha256']:raise ValueError('recording_pixel_changed')
    return len(frames)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ('binary','inputs','recording-receipt','output'):
        parser.add_argument('--'+name,type=Path,required=True)
    args=parser.parse_args()
    if args.output.exists():parser.error('Use a new output directory')
    inputs=args.inputs.resolve(strict=True)
    receipt=json.loads(args.recording_receipt.read_text())
    count=verify(inputs,receipt)
    args.output.mkdir(parents=True)
    binary=args.binary.resolve(strict=True)
    proof=dict(scope='Recorded pixel/raw-IMU replay only; accuracy is evaluated separately',
               truth_input=False,input_files=receipt['input_files'],frames=count,
               binary_sha256=digest(binary))
    with (args.output/'replay.log').open('w') as log:
        try:
            result=subprocess.run([str(binary),str(inputs),str(args.output/'states.csv')],
                                  stdout=log,stderr=subprocess.STDOUT,timeout=60,check=False)
            proof.update(passed=result.returncode==0,returncode=result.returncode)
        except subprocess.TimeoutExpired:
            proof.update(passed=False,error='replay_timeout')
    (args.output/'receipt.json').write_text(json.dumps(proof,indent=2)+'\n')
    print(json.dumps(proof,indent=2))
    raise SystemExit(0 if proof['passed'] else 1)
