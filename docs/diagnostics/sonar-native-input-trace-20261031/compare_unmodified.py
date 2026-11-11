# SPDX-License-Identifier: Apache-2.0
"""Compare all six complete replay images with the retained unmodified C ABI."""
import argparse
import ctypes as ct
import hashlib
import json
from pathlib import Path
import numpy as np
from trace import read_trace

ROOT=Path(__file__).resolve().parent


def run(binary):
    expected=json.loads((ROOT/'native-disabled-final/result.json').read_text())
    identity=json.loads((ROOT/'native-call-binding.json').read_text())
    binary_sha=hashlib.sha256(binary.read_bytes()).hexdigest()
    assert binary_sha==identity['binary_sha256']
    native=ct.CDLL(str(binary.resolve()),mode=ct.RTLD_GLOBAL);ptr=ct.POINTER(ct.c_float)
    compute,free=native.sonar_wgpu_compute,native.sonar_wgpu_free
    compute.argtypes=[ptr]*5+[ct.c_uint32]*4+[ct.c_float]*10+[ct.c_uint64]*2+[ct.c_bool]
    compute.restype=ptr;free.argtypes=[ptr,ct.c_size_t];free.restype=None
    class Info(ct.Structure):
        _fields_=[('fname',ct.c_char_p),('base',ct.c_void_p),('symbol',ct.c_char_p),('address',ct.c_void_p)]
    info=Info();lookup=ct.CDLL(None).dladdr;lookup.argtypes=[ct.c_void_p,ct.POINTER(Info)]
    assert lookup(ct.cast(compute,ct.c_void_p),ct.byref(info))
    assert Path(info.fname.decode()).resolve()==binary.resolve()
    record=read_trace(ROOT/'captured-final/sonar-native-0.bin')
    arrays=[record['arrays'][k] for k in ['depth','normal','reflectivity','mixing','window']]
    pointers=[a.ctypes.data_as(ptr) for a in arrays]
    rows=[]
    for item in expected['frames']:
        output=compute(*pointers,*record['dimensions'],*record['scalars'].values(),item['frame'],record['seed'],False)
        assert output
        try:
            data=np.ctypeslib.as_array(output,shape=(256*599*2,)).tobytes()
        finally:
            free(output,256*599*2)
        digest=hashlib.sha256(data).hexdigest()
        assert digest==item['sha256'],item['frame']
        rows.append(dict(frame=item['frame'],sha256=digest))
    result=dict(original_binary_sha256=binary_sha,actual_symbol_path_verified=True,
        complete_original_images_match_instrumented_adapter=True,frames=rows,
        original_source=identity['source'],field_capture=False,new_renderer=False,installed_sdk_changed=False)
    (ROOT/'unmodified-native-comparison.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result),flush=True)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--binary',required=True,type=Path)
    a=p.parse_args();run(a.binary)
