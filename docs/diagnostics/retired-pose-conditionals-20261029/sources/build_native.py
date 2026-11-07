from pathlib import Path
import json,subprocess
root=Path.cwd();base=root/'.local/conditional-surface-20261029';sdk=root/'.local/posim332f629-sdk';build=base/'native-build'
commands=[[str(sdk/'bin/cmake'),'-S',str(base/'library/native/basalt'),'-B',str(build),'-DCMAKE_BUILD_TYPE=Release','-DCMAKE_POLICY_VERSION_MINIMUM=3.5','-DCMAKE_PREFIX_PATH='+str(sdk),'-DSOURCE_ROOT='+str(root/'.local/basalt-vio'),'-DPython3_EXECUTABLE='+str(sdk/'bin/python3')],
 [str(sdk/'bin/cmake'),'--build',str(build),'--parallel','2','--target','basalt_stream','basalt_joint_information_probe','basalt_pressure_factor_probe','basalt_ping_joint_probe']]
(base/'build-commands.json').write_text(json.dumps(commands,indent=2)+'\n')
for command in commands:subprocess.run(command,check=True)
print('PASS: clean native stream and affected conditional/pressure/Ping probes built; existing SDK/binary untouched')
