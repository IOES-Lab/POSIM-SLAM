"""Build the bounded acoustic OpenCV ABI against the selected macOS engine SDK."""
import argparse,hashlib,json,subprocess,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]

def digest(path):return hashlib.sha256(path.read_bytes()).hexdigest()
def build(engine,output):
    if sys.platform!='darwin':raise ValueError('acoustic_build_macos_only')
    engine=Path(engine).resolve();output=Path(output).resolve()
    if output.exists():raise ValueError('acoustic_build_output_exists')
    source=ROOT/'src/posim_slam/data/native/acoustic_cv.cc'
    required=[engine/'include/opencv4/opencv2/core/version.hpp']+[engine/'lib'/('libopencv_'+name+'.413.dylib') for name in ('core','imgproc','video')]
    if not all(p.is_file() for p in required):raise ValueError('acoustic_requires_sdk_opencv_413')
    target=output/'install/lib/libwwos_acoustic_cv.dylib';target.parent.mkdir(parents=True)
    command=['clang++','-std=c++17','-dynamiclib','-O2',str(source),'-I'+str(engine/'include/opencv4'),'-L'+str(engine/'lib'),'-lopencv_core','-lopencv_imgproc','-lopencv_video','-Wl,-rpath,'+str(engine/'lib'),'-Wl,-install_name,@rpath/'+target.name,'-o',str(target)]
    subprocess.run(command,check=True,capture_output=True)
    # ABI smoke runs inside SDK Python without importing the research cv2 wheel.
    probe='import ctypes,json; p=ctypes.CDLL('+repr(str(target))+'); p.wwos_acoustic_opencv_version.restype=ctypes.c_char_p; print(json.dumps(dict(abi=p.wwos_acoustic_abi(),opencv=p.wwos_acoustic_opencv_version().decode())))'
    smoke=json.loads(subprocess.check_output([str(engine/'bin/python3'),'-c',probe],text=True))
    if smoke!={'abi':1,'opencv':'4.13.0'}:raise ValueError('acoustic_native_abi_mismatch')
    receipt={'schema':'wwwposim-acoustic-native-1','passed':True,'scope':'Build and ABI smoke only; estimator/native qualification separate','library_sha256':digest(target),'source_sha256':digest(source),'sdk_files':{str(p.relative_to(engine)):digest(p) for p in required},'abi':1,'opencv_version':smoke['opencv'],'tracker_version':'0.1.0'}
    (output/'receipt.json').write_text(json.dumps(receipt,indent=2)+'\n')
    return output
if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    for name in ('engine','output'):p.add_argument('--'+name,type=Path,required=True)
    a=p.parse_args();print(build(a.engine,a.output))
