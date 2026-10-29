from pathlib import Path
import csv,hashlib,json,subprocess,sys
import numpy as np
base=Path.cwd()/'.local/dvl-cone-overlap-20261029';out=base/'control-lf';out.mkdir(exist_ok=False)
raw=out/'inputs';raw.mkdir();stamp=2_000_000_000
ranges=np.linspace(.2,6,233);angles=np.linspace(-np.pi/3,np.pi/3,32)
frame=dict(ranges=ranges,beam_directions=np.column_stack((np.cos(angles),np.sin(angles),np.zeros(32))),
 intensity_db=np.repeat(np.where(ranges<2.8,0.,30.)[:,None],32,axis=1),stamp=np.array([2,0]))
np.savez_compressed(raw/'frame.npz',**frame)
(raw/'timeline.json').write_text(json.dumps(dict(sonar=[dict(stamp_ns=stamp,file='frame.npz')])))
rotation=[[np.cos(.35),0,np.sin(.35)],[0,1,0],[-np.sin(.35),0,np.cos(.35)]]
cal=dict(sonar=dict(imu_to_sonar=[0,0,0],sonar_to_imu=rotation,range_min=.2,range_max=6,
 horizontal_fov=2*np.pi/3,vertical_fov=np.pi/9))
(out/'calibration.json').write_text(json.dumps(cal))
with (out/'states.csv').open('w') as f:
 writer=csv.writer(f,lineterminator="\n");writer.writerow(['stamp_ns','x','y','z','qx','qy','qz','qw']);writer.writerow([stamp,0,0,0,0,0,0,1])
results={}
for name,height in (('supported',-1.),('outside',-8.)):
 support=[[x,y,height] for x in np.arange(1.8,3.5,.2) for y in np.arange(-2.8,2.9,.2)]
 report=dict(truth_is_estimator_input=False,online_state_updated=False,config=dict(surface_sigma_m=.05,neighborhood_m=.75),
 rows=[dict(stamp_ns=stamp,beams=[dict(source_id=1,association_rejection=None,plane_patch=dict(normal=[0,0,1],distance=height,
 support_points=support,support_stamps_ns=[1_000_000_000]*len(support)))])])
 source=out/(name+'-spatial.json');source.write_text(json.dumps(report))
 path=out/(name+'.json')
 subprocess.run([sys.executable,str(base/'library/tools/diagnose_dvl_sonar_overlap.py'),'--record',str(raw),'--states',str(out/'states.csv'),
 '--calibration',str(out/'calibration.json'),'--spatial-report',str(source),'--output',str(path)],check=True,stdout=subprocess.DEVNULL)
 results[name]=json.loads(path.read_text())['totals']['contemporaneous']
assert results['supported']['five_samples_supported']>0
assert results['outside']['five_samples_supported']==0
receipt=dict(synthetic=True,not_native_sensor_acceptance=True,results=results,
 tool_sha256=hashlib.sha256((base/'library/tools/diagnose_dvl_sonar_overlap.py').read_bytes()).hexdigest())
(out/'receipt.json').write_text(json.dumps(receipt,indent=2)+'\n');print(json.dumps(receipt))
