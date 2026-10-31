# SPDX-License-Identifier: Apache-2.0
"""Check every recorded native operand and output against independent receipts."""
import argparse
import hashlib
import json
from pathlib import Path
import struct
import numpy as np
from trace import LIMIT, read_trace

ROOT = Path(__file__).resolve().parent


def digest(data):
    return hashlib.sha256(data).hexdigest()


def verify(operand, save=False, suffix=''):
    off = json.loads((ROOT/f'native-disabled{suffix}/result.json').read_text())
    on = json.loads((ROOT/f'native-enabled{suffix}/result.json').read_text())
    assert off == on, 'recording changes complete native output or input identity'
    assert off['input_sha256'] == digest(operand.read_bytes())
    assert off['first_frame_matches_prior_unmodified_native_bytes']
    with np.load(operand, allow_pickle=False) as z:
        expected = {name: z[name].tobytes() for name in ['depth', 'normal', 'mixing', 'window']}
        rho = np.full(z['depth'].shape, .001, dtype='<f4').tobytes()
        scalars = [1500, 15, 220, float(z['alpha']), 1, 29900, 1, float(z['area']),
                   float(z['angles'][-1]-z['angles'][0]), float(np.pi/9)]
    files = sorted((ROOT/f'captured{suffix}').glob('sonar-native-*.bin*'))
    assert len(files) == 4
    total = 0
    rows = []
    for i, file in enumerate(files):
        record = read_trace(file)
        total += record['bytes']
        assert record['dimensions'] == (256, 256, 599, 1)
        assert record['frame_index'] == off['frames'][i]['frame'] == 60000+i
        assert record['seed'] == 73 and record['debug'] == 0
        assert struct.pack('<10f', *record['scalars'].values()) == struct.pack('<10f', *scalars)
        for name, data in expected.items():
            assert record['arrays'][name].tobytes() == data, (i, name)
        assert record['arrays']['reflectivity'].tobytes() == rho
        output = record['arrays']['output']
        assert digest(output.tobytes()) == off['frames'][i]['sha256']
        metadata = record['metadata']
        stamp = 100_000_000_000 + i*500_000_000
        assert metadata['context_source'] == 'offline_supplied_grid'
        assert metadata['capture_stamp_ns'] == metadata['render_begin_stamp_ns'] == stamp
        assert metadata['render_sensor_world_xyz_wxyz'] == metadata['render_begin_sensor_world_xyz_wxyz'] == [1,2,-2,1,0,0,0]
        assert metadata['render_sensor_local_xyz_wxyz'] == [0,0,0,1,0,0,0]
        assert metadata['sensor_render_name'] == 'offline-replay'
        assert metadata['render_begin_available'] and not metadata['render_parent_available']
        rows.append(dict(frame=record['frame_index'], bytes=record['bytes'], sha256=record['sha256'],
                         output_sha256=digest(output.tobytes())))
    assert total <= LIMIT
    result = dict(native_frames=6, complete_output_byte_identity=True, captured_frames=len(rows),
                  every_input_array_and_scalar_byte_identical=True, complete_captured_outputs_match=True,
                  total_recorded_bytes=total, frame_and_byte_quotas_preserved=True, records=rows,
                  supplied_grid_only=True, renderer_pose_is_synthetic=True, actual_field_input_acquired=False,
                  installed_sdk_changed=False, new_renderer=False)
    if save:
        (ROOT/f'native-verification{suffix}.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result), flush=True)
    return result


if __name__ == '__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--operand',required=True,type=Path)
    p.add_argument('--save',action='store_true');p.add_argument('--suffix',default='')
    a=p.parse_args();verify(a.operand,a.save,a.suffix)
