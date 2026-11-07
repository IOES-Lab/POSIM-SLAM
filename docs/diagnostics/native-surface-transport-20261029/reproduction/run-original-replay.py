from pathlib import Path
import json,os,subprocess,sys,time
root=Path.cwd();base=root/'.local/native-surface-transport-20261029'
template=json.loads((root/'docs/validation/retired-pose-conditionals-20261029/replay-commands.json').read_text())[0]
commands=[]
for mode in ('off','on'):
    command=list(template);command[0]=sys.executable;command[1]=str(base/'replay-native-owner.py')
    for flag,value in [('--record','/Volumes/ExternalHDD/POSIM/validation-data/posim-slam/sonar-low-altitude-15m-20261029/inputs'),
        ('--binary',str(base/'native-build/basalt_stream')),('--output',str(base/('original-'+mode)))]:
        command[command.index(flag)+1]=value
    commands.append(command)
(base/'original-replay-commands.json').write_text(json.dumps(commands,indent=2)+'\n')
for mode,command in zip(('off','on'),commands):
    output=base/('original-'+mode)
    if output.exists():raise SystemExit('Preserve existing original replay: '+str(output))
    start=time.monotonic();path=base/('original-'+mode+'.log')
    with path.open('w') as log:
        result=subprocess.run(command,env={**os.environ,'SURFACE_OWNER_REPLAY':'1' if mode=='on' else '0'},stdout=log,stderr=subprocess.STDOUT)
    print('Original 15 m replay',mode,'exit',result.returncode,'after',round(time.monotonic()-start,1),'s',flush=True)
    if result.returncode:
        print(path.read_text()[-6000:]);raise SystemExit(result.returncode)
