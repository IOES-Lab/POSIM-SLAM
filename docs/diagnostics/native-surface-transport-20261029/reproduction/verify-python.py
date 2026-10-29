from pathlib import Path
import hashlib,json,os,sys,unittest
base=Path(__file__).resolve().parent;mode=sys.argv[1];os.chdir(base)
import posim_slam
expected=base/('wheel-install' if mode=='wheel' else 'library/src')
assert Path(posim_slam.__file__).resolve().is_relative_to(expected)
suite=unittest.defaultTestLoader.discover(str(base/'library/tests'))
with (base/(mode+'-tests.log')).open('w') as log:result=unittest.TextTestRunner(stream=log).run(suite)
receipt=dict(tests_run=result.testsRun,failures=len(result.failures),errors=len(result.errors),
    skipped=[dict(test=str(test),reason=reason) for test,reason in result.skipped],installed_module=str(posim_slam.__file__),cwd=str(Path.cwd()))
if mode=='wheel':receipt['wheel_sha256']=hashlib.sha256((base/'wheels/posim_slam-0.1.0-py3-none-any.whl').read_bytes()).hexdigest()
(base/(mode+'-receipt.json')).write_text(json.dumps(receipt,indent=2)+'\n')
print(mode,':',result.testsRun,'tests;',len(result.skipped),'optional skips;',len(result.failures),'failures;',len(result.errors),'errors')
raise SystemExit(not result.wasSuccessful())
