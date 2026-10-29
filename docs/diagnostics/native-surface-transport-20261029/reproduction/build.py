from pathlib import Path
import json,subprocess,time
root=Path.cwd();base=root/'.local/native-surface-transport-20261029';sdk=root/'.local/posim332f629-sdk'
commands=[ [str(sdk/'bin/cmake'),'-S',str(base/'library/native/basalt'),'-B',str(base/'native-build'),'-DCMAKE_BUILD_TYPE=Release',
 '-DCMAKE_POLICY_VERSION_MINIMUM=3.5','-DCMAKE_PREFIX_PATH='+str(sdk),'-DSOURCE_ROOT='+str(root/'.local/basalt-vio'),'-DPython3_EXECUTABLE='+str(sdk/'bin/python3')],
 [str(sdk/'bin/cmake'),'--build',str(base/'native-build'),'--parallel','2','--target','basalt_stream','basalt_surface_owner_probe','basalt_pressure_factor_probe','basalt_dvl_factor_probe','basalt_initial_prior_probe','basalt_imu_interval_probe']]
(base/'build-commands.json').write_text(json.dumps(commands,indent=2)+'\n')
for i,command in enumerate(commands):
 start=time.monotonic();path=base/('build-'+str(i)+'.log')
 with path.open('w') as log:r=subprocess.run(command,stdout=log,stderr=subprocess.STDOUT)
 print('Native build phase',i,'exit',r.returncode,'after',round(time.monotonic()-start,1),'s',flush=True)
 if r.returncode:
  print(path.read_text()[-7000:]);raise SystemExit(r.returncode)
