# SPDX-License-Identifier: Apache-2.0
"""Compile only changed native objects, reusing read-only qualified dependencies."""
import argparse
import hashlib
import json
from pathlib import Path
import shlex
import subprocess

ROOT = Path(__file__).resolve().parent


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def build(source, previous, output_name='native'):
    output = ROOT/output_name
    output.mkdir()
    original = previous/'build/multibeam_sonar_system/multibeam_sonar'
    make = original/'CMakeFiles/multibeam_sonar.dir'
    flags = {}
    for line in (make/'flags.make').read_text().splitlines():
        if ' = ' in line:
            key, value = line.split(' = ', 1); flags[key] = shlex.split(value)
    folder = source/'gazebo/posim_gz_multibeam_sonar/multibeam_sonar'
    inputs = [folder/'MultibeamSonarSensor.cc', folder/'sonar_compute_wgpu.cc', ROOT/'native_replay.cc']
    objects = []
    for index, path in enumerate(inputs):
        obj = output/(path.name+'.o'); objects.append(obj)
        command = ['/usr/bin/c++', '-I'+str(folder)] + flags['CXX_DEFINES'] + flags['CXX_INCLUDES'] + flags['CXX_FLAGS']
        command += ['-c', str(path), '-o', str(obj)]
        with (output/f'compile-{index}.log').open('w') as log:
            subprocess.run(command, stdout=log, stderr=subprocess.STDOUT, check=True)
    command = shlex.split((make/'link.txt').read_text())
    for index, arg in enumerate(command):
        if arg.endswith('multibeam_sonar.dir/MultibeamSonarSensor.cc.o'): command[index] = str(objects[0])
        if arg.endswith('multibeam_sonar.dir/sonar_compute_wgpu.cc.o'): command[index] = str(objects[1])
    binary = output/'libmultibeam_sonar.dylib'
    command[command.index('-o')+1] = str(binary)
    command.append(str(objects[2]))
    # LLVM response syntax, not a shell; prevent argument-length failures.
    response = output/'link.rsp'
    response.write_text('\n'.join('"'+x.replace('\\','\\\\').replace('"','\\"')+'"' for x in command[1:])+'\n')
    with (output/'link.log').open('w') as log:
        subprocess.run([command[0], '@'+str(response)], cwd=original, stdout=log, stderr=subprocess.STDOUT, check=True)
    record = dict(binary_sha256=sha(binary), based_on_source='ffecee9be9976f68c9e883d6cf3bbd6d40eb7729',
                  installed_sdk_changed=False, new_renderer=False,
                  source_sha256={p.name: sha(p) for p in inputs + [folder/'sonar_native_trace.hh',folder/'MultibeamSonarSensor.hh']},
                  binary_bytes=binary.stat().st_size)
    receipt_name='build-receipt.json' if output_name=='native' else f'build-receipt-{output_name}.json'
    (ROOT/receipt_name).write_text(json.dumps(record,indent=2)+'\n')
    print(json.dumps(record),flush=True)


if __name__ == '__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--source',required=True,type=Path)
    p.add_argument('--previous',required=True,type=Path);p.add_argument('--output-name',default='native')
    a=p.parse_args();build(a.source,a.previous,a.output_name)
