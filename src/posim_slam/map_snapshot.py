"""Provenance-checked local-window and accumulated-landmark PLY contracts, independent of ROS/apps.

These sparse odometry snapshots have explicit scope and no loop closure. Provenance is checked at each process boundary; raw worker output carries
request identity and acquisition time before its owner adds session/frame tags.
"""
import re
import numpy as np

MAX_POINTS=100000
MAX_BYTES=MAX_POINTS*12+4096
SCOPES={'complete_active_local_vio_window','accumulated_local_vio_landmarks',
        'accumulated_local_sonar_hypotheses'}


def sonar_snapshot(points, *, request_id, stamp_ns):
    """Encode the complete bounded acoustic archive with an explicit scope."""
    array=np.asarray(points,dtype='<f4')
    if (array.ndim!=2 or array.shape[1]!=3 or not 0<len(array)<=MAX_POINTS
            or not np.isfinite(array).all() or type(request_id) is not int
            or type(stamp_ns) is not int or not 0<request_id<2**63 or not 0<stamp_ns<2**63):
        raise ValueError('invalid_acoustic_snapshot')
    header=('ply\nformat binary_little_endian 1.0\n'
        'comment scope accumulated_local_sonar_hypotheses\n'
        'comment map_qualified false\n'
        f'comment request_id {request_id}\ncomment stamp_ns {stamp_ns}\n'
        f'element vertex {len(array)}\nproperty float x\nproperty float y\nproperty float z\nend_header\n')
    data=header.encode('ascii')+array.tobytes()
    validate_snapshot(data,dict(scope='accumulated_local_sonar_hypotheses'))
    return data


def validate_snapshot(data,expected):
    if not isinstance(data,bytes) or len(data)>MAX_BYTES:raise ValueError('snapshot_size')
    end=data.find(b'end_header\n',0,4096)
    if end<0:raise ValueError('snapshot_header')
    header=data[:end].decode('ascii').splitlines()
    if header[:2]!=['ply','format binary_little_endian 1.0']:
        raise ValueError('snapshot_format')
    metadata={};count=None;properties=[]
    for line in header[2:]:
        if line.startswith('comment '):
            key,value=line[8:].split(' ',1)
            if key in metadata:raise ValueError('snapshot_duplicate_metadata')
            metadata[key]=value
        elif line.startswith('element vertex ') and count is None:
            count=int(line[15:])
        elif line.startswith('property '):properties.append(line)
        else:raise ValueError('snapshot_schema')
    if (count is None or not 0<count<=MAX_POINTS or
            properties!=['property float x','property float y','property float z'] or
            len(data)!=end+11+count*12 or metadata.get('scope') not in SCOPES):
        raise ValueError('snapshot_schema')
    for key in ('request_id','stamp_ns'):
        if not metadata.get(key,'').isdigit() or not 0<int(metadata[key])<2**63:
            raise ValueError('snapshot_identity')
    if any(metadata.get(key)!=str(value) for key,value in expected.items()):
        raise ValueError('snapshot_provenance')
    if not np.isfinite(np.frombuffer(data[end+11:],dtype='<f4')).all():
        raise ValueError('snapshot_coordinates')
    return dict(scope=metadata['scope'],points=count,stamp_ns=int(metadata['stamp_ns']),request_id=int(metadata['request_id']))


def annotate_snapshot(data,*,nonce,frame,request_id,stamp_ns,points):
    if (not isinstance(nonce,str) or not re.fullmatch(r'[A-Za-z0-9_-]{1,128}',nonce) or
            not isinstance(frame,str) or not re.fullmatch(r'[A-Za-z0-9_./-]{1,256}',frame)):
        raise ValueError('snapshot_owner_identity')
    metadata=validate_snapshot(data,dict(request_id=request_id,stamp_ns=stamp_ns))
    if type(points) is not int or points!=metadata['points']:raise ValueError('snapshot_count')
    prefix=b'ply\nformat binary_little_endian 1.0\n'
    comments=(f'comment session_nonce {nonce}\ncomment frame_id {frame}\ncomment units m\n'
              'comment library basalt\ncomment navigation_ready false\n').encode('ascii')
    result=prefix+comments+data[len(prefix):]
    validate_snapshot(result,dict(session_nonce=nonce,frame_id=frame,units='m'))
    return result
