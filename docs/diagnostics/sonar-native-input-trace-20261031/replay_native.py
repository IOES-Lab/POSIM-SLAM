# SPDX-License-Identifier: Apache-2.0
"""Use the actual C++ WGPU adapter on a retained supplied grid; no world starts."""
import argparse
import ctypes as ct
import hashlib
import json
from pathlib import Path
import numpy as np

ROOT = Path(__file__).resolve().parent


def digest(array):
    return hashlib.sha256(np.ascontiguousarray(array).tobytes()).hexdigest()


def run(binary, operand, output):
    output.mkdir()
    native = ct.CDLL(str(binary.resolve()), mode=ct.RTLD_GLOBAL)
    ptr = ct.POINTER(ct.c_float)
    call = native.posim_sonar_trace_replay
    call.argtypes = [ptr]*4 + [ct.c_double]*2 + [ct.c_float, ct.c_uint64, ct.c_uint64, ct.c_int64, ptr]
    call.restype = ct.c_int
    class Info(ct.Structure):
        _fields_ = [('fname', ct.c_char_p), ('base', ct.c_void_p), ('symbol', ct.c_char_p), ('address', ct.c_void_p)]
    info=Info();lookup=ct.CDLL(None).dladdr;lookup.argtypes=[ct.c_void_p,ct.POINTER(Info)]
    assert lookup(ct.cast(call,ct.c_void_p),ct.byref(info))
    assert Path(info.fname.decode()).resolve()==binary.resolve()
    with np.load(operand, allow_pickle=False) as z:
        depth,normal,mixing,window=[np.ascontiguousarray(z[k],dtype='<f4') for k in ['depth','normal','mixing','window']]
        angles=z['angles'];alpha=float(z['alpha']);area=float(z['area'])
    hfov=float(angles[-1]-angles[0]);vfov=float(np.pi/9)
    pointers=[x.ctypes.data_as(ptr) for x in [depth,normal,mixing,window]]
    records=[]
    for i in range(6):
        result=np.zeros((256,599,2),dtype='<f4')
        code=call(*pointers,hfov,vfov,alpha,60000+i,73,100_000_000_000+i*500_000_000,result.ctypes.data_as(ptr))
        assert code==0,code
        assert np.isfinite(result).all()
        records.append(dict(frame=60000+i,sha256=digest(result)))
    # The first frame was retained before instrumentation. Compare all complex
    # samples bit-for-bit, not only power moments or selected pixels.
        if i==0:
            with np.load(operand.parent/'native-output.npz',allow_pickle=False) as z:
                old=z['60000']
            first=np.stack([old.real,old.imag],axis=-1)
            assert result.tobytes()==first.tobytes(),'instrumented adapter differs from retained unmodified ABI output'
    receipt=dict(frames=records,first_frame_matches_prior_unmodified_native_bytes=True,
        binary_sha256=hashlib.sha256(binary.read_bytes()).hexdigest(),
        input_sha256=hashlib.sha256(operand.read_bytes()).hexdigest(),
        inputs={k:digest(x) for k,x in [('depth',depth),('normal',normal),('mixing',mixing),('window',window)]},
        field_capture=False,supplied_reference_grid=True,render_pose_is_synthetic=True,
        installed_sdk_changed=False,new_renderer=False)
    (output/'result.json').write_text(json.dumps(receipt,indent=2)+'\n')
    print(json.dumps(receipt),flush=True)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--binary',type=Path,required=True)
    p.add_argument('--operand',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();run(a.binary,a.operand,a.output)
