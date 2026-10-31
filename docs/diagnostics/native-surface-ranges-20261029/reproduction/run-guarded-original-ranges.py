from pathlib import Path
import json,os,subprocess,sys,time
root=Path.cwd();base=root/'.local/native-surface-ranges-20261029';prior=base.parent/'native-surface-transport-20261029'
template=json.loads((prior/'original-replay-commands.json').read_text())[0]
commands=[]
for mode in ('dvl-only','dvl-fls'):
    command=list(template);command[0]=sys.executable;command[1]=str(base/'replay-original-ranges.py')
    command[command.index('--output')+1]=str(base/('guarded-'+mode));commands.append(command)
(base/'original-replay-commands.json').write_text(json.dumps(commands,indent=2)+'\n')
for mode,command in zip(('dvl-only','dvl-fls'),commands):
    if (base/('guarded-'+mode)).exists():raise SystemExit('Preserve existing diagnostic '+mode)
    start=time.monotonic();path=base/('guarded-'+mode+'.log')
    with path.open('w') as log:
        result=subprocess.run(command,env={**os.environ,'INCLUDE_ORIGINAL_FLS':'1' if mode=='dvl-fls' else '0'},stdout=log,stderr=subprocess.STDOUT)
    print('Original range diagnostic',mode,'exit',result.returncode,'after',round(time.monotonic()-start,1),'s',flush=True)
    if result.returncode:
        # Keep receipts, errors and every old input; do not restart a failed run.
        lines=path.read_text().splitlines();print('\n'.join(line[:600] for line in lines[-20:]));raise SystemExit(result.returncode)
