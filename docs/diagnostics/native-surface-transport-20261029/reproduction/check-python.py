from pathlib import Path
import os,subprocess,sys
base=Path(__file__).resolve().parent
commands=[([sys.executable,base/'verify-python.py','source'],str(base/'library/src')),
([sys.executable,'-m','pip','wheel','--no-cache-dir','--no-deps','--no-build-isolation',base/'library','--wheel-dir',base/'wheels'],None),
([sys.executable,'-m','pip','install','--no-cache-dir','--no-deps','--target',base/'wheel-install',base/'wheels/posim_slam-0.1.0-py3-none-any.whl'],None),
([sys.executable,base/'verify-python.py','wheel'],str(base/'wheel-install'))]
for index,(command,pythonpath) in enumerate(commands):
 env=dict(os.environ)
 for name in list(env):
  if name.startswith('POSIM_SLAM_'):env.pop(name)
 if pythonpath:env['PYTHONPATH']=pythonpath
 path=base/('python-qualification-'+str(index)+'.log')
 with path.open('w') as log:result=subprocess.run(command,env=env,stdout=log,stderr=subprocess.STDOUT)
 print('Source/wheel phase',index,'exit',result.returncode,flush=True)
 if result.returncode:
  print(path.read_text()[-6000:]);raise SystemExit(result.returncode)
