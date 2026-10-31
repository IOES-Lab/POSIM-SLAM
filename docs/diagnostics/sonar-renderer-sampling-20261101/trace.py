# SPDX-License-Identifier: Apache-2.0
"""Read bounded exact sonar-ABI records, preserving all IEEE array values."""
import hashlib
import gzip
import json
import struct
from pathlib import Path
import numpy as np

LIMIT = 40 * 1024 * 1024
HEADER = struct.Struct('<8sI4I10f2QB')


def read_trace(path):
    path = Path(path)
    size = path.stat().st_size
    if size > LIMIT or size < HEADER.size + 8:
        raise ValueError('trace size is outside the bounded format')
    if path.suffix == '.gz':
        with gzip.open(path, 'rb') as stream:
            data = stream.read(LIMIT + 1)
        if len(data) > LIMIT:
            raise ValueError('decompressed trace exceeds the bounded format')
    else:
        data = path.read_bytes()
    fields = HEADER.unpack_from(data)
    magic, length = fields[:2]
    beams, rays, frequencies, skips = fields[2:6]
    if magic != b'PSNWG001' or not 0 < length <= 65536:
        raise ValueError('unsupported trace header')
    if not (0 < beams <= 8192 and 0 < rays <= 8192 and 0 < frequencies <= 4096):
        raise ValueError('invalid trace dimensions')
    cells = beams * rays
    shapes = {'depth': (beams, rays), 'normal': (beams, rays, 3),
              'reflectivity': (beams, rays), 'mixing': (beams, beams),
              'window': (frequencies,), 'output': (beams, frequencies, 2)}
    count = 5 * cells + beams * beams + frequencies + 2 * beams * frequencies
    expected = HEADER.size + length + 4 * count + 8
    if len(data) != expected or data[-8:] != b'COMPLETE':
        raise ValueError('incomplete trace or unexpected trailing bytes')
    metadata = json.loads(data[HEADER.size:HEADER.size + length])
    if not isinstance(metadata.get('capture_stamp_ns'), int):
        raise ValueError('capture stamp is missing')
    offset = HEADER.size + length
    arrays = {}
    for key, shape in shapes.items():
        count = int(np.prod(shape))
        arrays[key] = np.frombuffer(data, dtype='<f4', count=count, offset=offset).reshape(shape)
        offset += count * 4
    scalar_names = ('speed', 'distance', 'source', 'attenuation', 'gain', 'bandwidth',
                    'corrector_sum', 'area', 'hfov', 'vfov')
    return dict(metadata=metadata, dimensions=(beams, rays, frequencies, skips),
                scalars=dict(zip(scalar_names, fields[6:16])),
                frame_index=fields[16], seed=fields[17], debug=fields[18], arrays=arrays,
                sha256=hashlib.sha256(data).hexdigest(), bytes=len(data))
