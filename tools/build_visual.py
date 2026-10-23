"""Build the pinned Eigen-only visual SLAM development stack in isolation.

--sources contains the five Git checkouts listed in sources.json. Exact Git
archives are staged before applying the recorded patch; source checkouts and
installed engines are never modified. The output directory must be new.
"""
from pathlib import Path
import argparse,hashlib,io,json,os,shutil,subprocess,tarfile
root=Path(__file__).resolve().parents[1]
parser=argparse.ArgumentParser(description=__doc__)
for name in ('sources','engine','output'):parser.add_argument('--'+name,type=Path,required=True)
args=parser.parse_args();out=args.output.resolve();engine=args.engine.resolve()
if out.exists():parser.error('Output must be new; existing builds are preserved.')
research=out/'source';research.mkdir(parents=True)
patch_root=root/'src/posim_slam/data/visual'
pins=json.loads((patch_root/'sources.json').read_text())
for name,revision in pins.items():
 archive=subprocess.check_output(['git','-C',str(args.sources/name),'archive',revision])
 destination=research/name;destination.mkdir(parents=True,exist_ok=True)
 with tarfile.open(fileobj=io.BytesIO(archive)) as tar:
  for member in tar.getmembers():
   if not (destination/member.name).resolve().is_relative_to(destination) or not (member.isdir() or member.isfile()):raise ValueError('Unexpected source archive entry')
  tar.extractall(destination)
subprocess.run(['patch','-f','-p1','-i',str(patch_root/'eigen-solvers.patch')],cwd=research/'stella_vslam',check=True)
native_source=out/'native-source'
shutil.copytree(root/'src/posim_slam/data/native/visual',native_source)
receipt={'sources':pins,'native_sources':{str(p.relative_to(native_source)):hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(native_source.glob('*')) if p.is_file()},'patch_sha256':hashlib.sha256((patch_root/'eigen-solvers.patch').read_bytes()).hexdigest(),'passed':False,'scope':'development build only; tracking acceptance is separate'}
(out/'receipt.json').write_text(json.dumps(receipt,indent=2)+'\n')
env=dict(os.environ,PATH=str(engine/'bin')+':/usr/bin:/bin:/usr/sbin:/sbin',CMAKE_PREFIX_PATH=str(out/'install')+':'+str(engine),AMENT_PREFIX_PATH=str(engine))
common=['-DCMAKE_CXX_FLAGS=-DEIGEN_MPL2_ONLY','-DPython3_EXECUTABLE='+str(engine/'bin/python3'),'-DCMAKE_BUILD_TYPE=Release','-DCMAKE_INSTALL_PREFIX='+str(out/'install'),'-DCMAKE_POLICY_VERSION_MINIMUM=3.5','-DCMAKE_BUILD_WITH_INSTALL_RPATH=ON','-DCMAKE_INSTALL_RPATH='+str(out/'install/lib')+';'+str(engine/'lib')]
for name,flags in [('eigen',['-DBUILD_TESTING=OFF']),('g2o',['-DEIGEN3_INCLUDE_DIR='+str(research/'eigen'),'-DEigen3_DIR='+str(out/'install/share/eigen3/cmake'),'-DG2O_USE_LGPL_LIBS=OFF','-DG2O_USE_CHOLMOD=OFF','-DG2O_USE_CSPARSE=OFF','-DG2O_USE_OPENGL=OFF','-DG2O_USE_OPENMP=OFF','-DG2O_BUILD_APPS=OFF','-DG2O_BUILD_EXAMPLES=OFF','-DG2O_BUILD_LINKED_APPS=OFF','-DBUILD_UNITTESTS=OFF']),('stella_vslam',['-DEigen3_DIR='+str(out/'install/share/eigen3/cmake'),'-DUSE_GTSAM=OFF','-DUSE_ARUCO=OFF','-DUSE_CCACHE=OFF','-DUSE_OPENMP=OFF','-DUSE_AVX=OFF','-DUSE_SSE=OFF','-DUSE_SSE2=OFF','-DUSE_SSE3=OFF','-DUSE_SSE4=OFF','-DUSE_SSE_ORB=OFF','-DUSE_SSE_FP_MATH=OFF','-DBUILD_TESTS=OFF','-DBUILD_EXAMPLES=OFF','-DOpenMP_C_FLAGS=-Xpreprocessor -fopenmp','-DOpenMP_CXX_FLAGS=-Xpreprocessor -fopenmp','-DOpenMP_C_LIB_NAMES=omp','-DOpenMP_CXX_LIB_NAMES=omp','-DOpenMP_omp_LIBRARY='+str(engine/'lib/libomp.dylib')]),('wwos_visual_slam',['-DEigen3_DIR='+str(out/'install/share/eigen3/cmake'),'-DOpenMP_CXX_FLAGS=-Xpreprocessor -fopenmp','-DOpenMP_CXX_LIB_NAMES=omp','-DOpenMP_omp_LIBRARY='+str(engine/'lib/libomp.dylib')])]:
 build=out/'build'/name
 source=native_source if name=='wwos_visual_slam' else research/name
 for cmd in [[str(engine/'bin/cmake'),'-S',str(source),'-B',str(build),*common,*flags],[str(engine/'bin/cmake'),'--build',str(build),'--parallel','2'],[str(engine/'bin/cmake'),'--install',str(build)]]:
  with (out/'build.log').open('a') as log:subprocess.run(cmd,env=env,stdout=log,stderr=subprocess.STDOUT,check=True)

library=out/'install/lib/libstella_vslam.dylib'
subprocess.run([str(engine/'bin/python3'),'-c','import ctypes,sys;ctypes.CDLL(sys.argv[1])',str(library)],env=env,check=True)
receipt.update(passed=True,eigen_mpl2_only=True,executable_sha256=hashlib.sha256((out/'install/bin/wwos_visual_slam').read_bytes()).hexdigest(),library_sha256=hashlib.sha256(library.read_bytes()).hexdigest())
(out/'receipt.json').write_text(json.dumps(receipt,indent=2)+'\n')
print(json.dumps(receipt))
