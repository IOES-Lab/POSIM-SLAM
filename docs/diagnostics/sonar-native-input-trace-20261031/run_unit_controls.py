# SPDX-License-Identifier: Apache-2.0
"""Execute isolated native diagnostic controls, including refused/partial writes."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import numpy as np
from trace import HEADER, LIMIT, read_trace


def run(binary, output):
    output.mkdir()
    results = []
    data = np.arange(32, dtype=np.float32) / np.float32(7)
    data.view(np.uint32)[1] = 0x7fc01234
    expected = -np.arange(32, dtype=np.float32) / np.float32(3)
    expected.view(np.uint32)[2] = 0x7fc01234
    cases = [('disabled', 0), ('basic', 4), ('timing', 3), ('no-frame', 0),
             ('collision', 0), ('bad-dir', 0), ('oversized', 0), ('threads', 4),
             ('null-output', 4), ('nonfinite-pose', 4), ('reverse', 4), ('throw-output', 4),
             ('bad-start', 0), ('overflow-start', 0), ('bad-spacing', 0), ('byte-limit', 0)]
    for mode, count in cases:
        folder = output / mode
        folder.mkdir()
        env = {k: v for k, v in os.environ.items() if not k.startswith('POSIM_SONAR_NATIVE_TRACE_')}
        if mode != 'disabled':
            env['POSIM_SONAR_NATIVE_TRACE_DIR'] = str(folder if mode != 'bad-dir' else folder/'missing')
        if mode == 'timing':
            env.update(POSIM_SONAR_NATIVE_TRACE_START_NS='20', POSIM_SONAR_NATIVE_TRACE_SPACING_NS='20')
        if mode == 'bad-start':env['POSIM_SONAR_NATIVE_TRACE_START_NS']='-1'
        if mode == 'overflow-start':env['POSIM_SONAR_NATIVE_TRACE_START_NS']='9223372036854775808'
        if mode == 'bad-spacing':env['POSIM_SONAR_NATIVE_TRACE_SPACING_NS']='3x'
        if mode == 'collision':
            (folder/'sonar-native-0.bin').write_bytes(b'keep-original')
        result = subprocess.run([str(binary), mode], env=env, text=True, capture_output=True, check=True)
        (folder/'process.log').write_text(result.stdout + result.stderr)
        files = sorted(folder.glob('*.bin'))
        if mode == 'collision':
            assert len(files) == 1 and files[0].read_bytes() == b'keep-original'
        else:
            assert len(files) == count, (mode, len(files), count)
        if mode in ('null-output', 'throw-output'):
            for file in files:
                assert file.stat().st_size > HEADER.size, 'incomplete data not flushed/closed'
                try:
                    read_trace(file)
                except ValueError:
                    pass
                else:
                    raise AssertionError('null-output record accepted')
        elif mode != 'collision':
            for file in files:
                trace = read_trace(file)
                arrays = trace['arrays']
                assert trace['dimensions'] == (2, 3, 4, 7)
                assert trace['seed'] == 73 and trace['debug'] == 0
                assert trace['metadata']['capture_stamp_ns'] == 10 * (trace['frame_index'] - 100)
                assert trace['metadata']['sensor_render_name'] == 'test"sensor\n\\name'
                for key, array in arrays.items():
                    target = expected if key == 'output' else data
                    np.testing.assert_array_equal(array.ravel().view(np.uint32), target[:array.size].view(np.uint32))
                if mode == 'nonfinite-pose':
                    assert trace['metadata']['render_sensor_world_xyz_wxyz'][2] == {'f64_bits': 0x7ff8000000001234}
                else:
                    assert trace['metadata']['render_sensor_world_xyz_wxyz'] == [1, 2, 3, 1, 0, 0, 0]
        assert sum(p.stat().st_size for p in files) <= LIMIT
        if mode == 'timing':
            assert [read_trace(p)['metadata']['capture_stamp_ns'] for p in files] == [20, 40, 60]
        results.append(dict(mode=mode, expected_records=count, passed=True))
    # File-level negative controls: incomplete, foreign dimensions, trailing data.
    original = (output/'basic/sonar-native-0.bin').read_bytes()
    import struct
    invalid = [original[:-1], original + b'x', original[:12] + struct.pack('<I', 0) + original[16:]]
    for i, payload in enumerate(invalid):
        path = output/f'invalid-{i}.bin'; path.write_bytes(payload)
        try:
            read_trace(path)
        except ValueError:
            pass
        else:
            raise AssertionError('invalid trace accepted')
    (output/'result.json').write_text(json.dumps(dict(cases=results, reader_negative_controls=3,
        original_function_calls_and_ieee_bytes_preserved=True), indent=2)+'\n')
    print(f'PASS: {len(cases)} isolated native cases and 3 reader rejection controls')


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--binary', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    a = p.parse_args(); run(a.binary, a.output)
