from pathlib import Path
import hashlib,json,sys
import numpy as np
root=Path.cwd();base=root/'.local/dvl-cone-moving-20261029';sys.path.insert(0,str(root/'scripts'))
from evaluate_sonar_map_surfaces import terrain_triangles
proof=json.loads((base/'world-proof.json').read_text())
import xml.etree.ElementTree as ET
world=ET.parse(proof['world']).getroot().find('world');terrain=world.find("model[@name='gebco_terrain']")
assert terrain.findtext('pose')=='0 0 0 0 0 0'
mesh=Path(terrain.findtext('.//collision/geometry/mesh/uri'));tri=terrain_triangles(mesh)
a=tri[:,0];b=tri[:,1]-a;c=tri[:,2]-a;den=b[:,0]*c[:,1]-b[:,1]*c[:,0]
truth=np.array(json.loads((base/'record/inputs/truth-assessment-only.json').read_text()))
rows=[]
for sample in truth[::25]:
 p=sample[1:4];delta=p[:2]-a[:,:2]
 u=np.divide(delta[:,0]*c[:,1]-delta[:,1]*c[:,0],den,out=np.full(len(den),np.nan),where=abs(den)>1e-12)
 v=np.divide(b[:,0]*delta[:,1]-b[:,1]*delta[:,0],den,out=np.full(len(den),np.nan),where=abs(den)>1e-12)
 mask=(u>=-1e-9)&(v>=-1e-9)&(u+v<=1+1e-9);assert mask.any()
 z=np.max((a[:,2]+u*b[:,2]+v*c[:,2])[mask]);height=float(p[2]-z)
 rows.append(dict(stamp_ns=int(sample[0]),body_above_seabed_m=height,flat_level_near_fan_intersection_m=height/np.sin(.35+np.deg2rad(10))))
alt=np.array([r['body_above_seabed_m'] for r in rows]);near=np.array([r['flat_level_near_fan_intersection_m'] for r in rows])
report=dict(assessment_only=True,truth_is_estimator_input=False,rows=rows,
 body_above_seabed_m=dict(minimum=float(alt.min()),median=float(np.median(alt)),maximum=float(alt.max())),
 flat_level_near_fan_intersection_m=dict(minimum=float(near.min()),median=float(np.median(near)),maximum=float(near.max())),
 scene_sha256=hashlib.sha256(mesh.read_bytes()).hexdigest(),
 scope='Terrain and independent truth assessment only. Flat/level fan estimate explains operational geometry; not an actual acoustic return or sensor altitude.')
(base/'operating-geometry.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps({k:v for k,v in report.items() if k!='rows'}))
