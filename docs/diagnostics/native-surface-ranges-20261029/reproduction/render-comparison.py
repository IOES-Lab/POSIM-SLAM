from pathlib import Path
import gzip,hashlib,json,math,sys
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.cm import ScalarMappable
from PIL import Image
from mpl_toolkits.mplot3d.art3d import Line3DCollection
import xml.etree.ElementTree as ET
root=Path.cwd();base=root/'.local/native-surface-ranges-20261029';old=root/'.local/sonar-low-altitude-15m-20261029'
data=Path('/Volumes/ExternalHDD/POSIM/validation-data/posim-slam/sonar-low-altitude-15m-20261029')
sys.path[:0]=[str(base/'library/src'),str(root/'scripts')]
from posim_slam.native_surface import NativeSurfaceSnapshot
from pointcloud_figure import height_colours,project_cloud
from sonar_metric_view import draw_sonar
inspection=json.loads((base/'nonlinear-inspection.json').read_text())
alignment=json.loads((old/'assessment-baseline.json').read_text())['trajectory']['evaluation_alignment']
ar=np.array(alignment['rotation']);ap=np.array(alignment['translation']);clouds={}
for mode in ('dvl-only','dvl-fls'):
    folder=base/('original-'+mode);request=json.loads((folder/'original-request.json').read_text())
    snapshot=NativeSurfaceSnapshot.read(json.loads(gzip.decompress((folder/'snapshot-after.json.gz').read_bytes())))
    pose={v['stamp_ns']:v for v in snapshot.recover(sorted({o['stamp_ns'] for o in request['observations']}))['poses']}
    observations={tuple(o[k] for k in ('stamp_ns','sensor_id','beam_id')):o for o in request['observations']}
    rows=[]
    for row in inspection[mode]['after_rows']:
        assert row['geometry_error'] is None
        o=observations[tuple(row['key'])];s=pose[o['stamp_ns']]
        origin=s['position']+s['attitude']@o['imu_to_sensor']
        # Keep the measured range; plane geometry supplies only its conditional
        # cone/fan direction. This is not an independently observed XYZ point.
        point=origin+(np.asarray(row['conditional_hit'])-origin)*(o['measured_range']/row['predicted_range_m'])
        rows.append(dict(key=row['key'],world=ar@point+ap))
    clouds[mode]=np.array([v['world'] for v in rows])
    (folder/'conditional-hypotheses.json').write_text(json.dumps(dict(scope='All conditional measured-range hypotheses; no independent XYZ or qualified map',
        rows=[dict(key=v['key'],world=v['world'].tolist()) for v in rows],map_qualified=False),indent=2)+'\n')
    header='ply\nformat ascii 1.0\ncomment Conditional measured-range hypotheses; map unqualified\nelement vertex '+str(len(rows))+'\nproperty float x\nproperty float y\nproperty float z\nend_header\n'
    (folder/'conditional-hypotheses.ply').write_text(header+''.join(' '.join(str(x) for x in v['world'])+'\n' for v in rows))
timeline=json.loads((data/'inputs/timeline.json').read_text());stamp=35_000_000_000
camera=next(r for r in timeline['camera'] if r['stamp_ns']==stamp)
sonar_row=next(r for r in timeline['sonar'] if r['stamp_ns']==stamp)
with np.load(data/'inputs'/camera['file']) as raw:
    h,w,step=map(int,(raw['height'],raw['width'],raw['step']));rgb=raw['data'].reshape(h,step)[:,:3*w].reshape(h,w,3)
with np.load(data/'inputs'/sonar_row['file']) as raw:sonar={k:raw[k].copy() for k in raw.files}
field=np.asarray(Image.open(old/'context-reference-35/isometric-rgb.png'))
fig=plt.figure(figsize=(18,11),layout='constrained')
ax=fig.add_subplot(2,3,1);ax.imshow(rgb);ax.axis('off');ax.set_title('Original RGB / sonar origin / 35.00 s')
h,w=rgb.shape[:2];f=w/(2*np.tan(np.pi/3));x=np.linspace(0,w,256)
dy=f*np.tan(np.pi/18)*np.sqrt(1+((x-w/2)/f)**2)
for y in (h/2-dy,h/2+dy):ax.plot(x,y,color='#ffb13b',lw=1.2)
ax=fig.add_subplot(2,3,2);draw_sonar(ax,sonar['intensity_db'],sonar['ranges'],sonar['beam_directions'],cmap='gray',vmin=-10,vmax=60)
ax.set_title('Original acoustic image / same 35.00 s exposure')
ax=fig.add_subplot(2,3,3);ax.imshow(field);ax.axis('off');ax.set_title('Isometric field context / 35 s body pose')
proof=json.loads((old/'world-proof.json').read_text());pose=list(map(float,proof['isometric_camera']['pose'].split()));pitch,yaw=pose[4:]
ry=np.array([[np.cos(pitch),0,np.sin(pitch)],[0,1,0],[-np.sin(pitch),0,np.cos(pitch)]])
rz=np.array([[np.cos(yaw),-np.sin(yaw),0],[np.sin(yaw),np.cos(yaw),0],[0,0,1]])
rig=dict(eye=pose[:3],camera_to_world=(rz@ry).tolist(),width=1000,height=800,horizontal_fov=1.35)
context=json.loads((old/'context-reference-35/receipt.json').read_text());p=np.array(context['body_position']);r=np.array(context['body_rotation'])
mount=np.array([[np.cos(.35),0,np.sin(.35)],[0,1,0],[-np.sin(.35),0,np.cos(.35)]])
origin=p+r@np.array([.24,0,-.1]);uv,_=project_cloud(np.array([p,origin,origin+r@mount@np.array([1.5,0,0])]),rig)
ax.annotate('BlueROV',uv[0],xytext=(-45,-25),textcoords='offset points',color='white',fontsize=9,arrowprops=dict(arrowstyle='-',color='white'))
ax.annotate('camera + sonar\n20.1 degrees down',uv[2],xytext=(-55,45),textcoords='offset points',ha='right',color='#ffd58c',fontsize=9,arrowprops=dict(arrowstyle='->',color='#ffb13b',lw=1.5))
ax.text(.02,.03,'Clear-water context; particles, caustics and wave water retained',transform=ax.transAxes,color='white',fontsize=8,bbox=dict(facecolor='black',alpha=.65,pad=3))
scene=ET.parse(proof['world']).getroot().find('world');edges=[];corners=[]
for model in scene.findall('model'):
    if not model.get('name','').startswith('acoustic_validation_'):continue
    center=np.array(list(map(float,model.findtext('pose').split()[:3])))
    size=np.array(list(map(float,model.findtext('.//collision/geometry/box/size').split())))
    vertices=np.array([[a,b,c] for a in (-1,1) for b in (-1,1) for c in (-1,1)])*size/2+center
    edges.extend([[vertices[i],vertices[j]] for i in range(8) for j in range(i+1,8) if np.count_nonzero(vertices[i]!=vertices[j])==1]);corners.extend(vertices)
all_points=np.vstack([*clouds.values(),np.asarray(corners)]);lo=all_points.min(axis=0);hi=all_points.max(axis=0);pad=(hi-lo)*.06+.05
for index,(mode,points) in enumerate(clouds.items(),4):
    ax=fig.add_subplot(2,3,index,projection='3d');colours,cmap,norm,meta=height_colours(points,limits=(-4.9,-1.5))
    ax.scatter(*points.T,s=64,c=colours,depthshade=False,edgecolors='none')
    ax.add_collection3d(Line3DCollection(edges,colors='#64748b',linewidths=.8,alpha=.5))
    ax.set(xlim=(lo[0]-pad[0],hi[0]+pad[0]),ylim=(lo[1]-pad[1],hi[1]+pad[1]),zlim=(lo[2]-pad[2],hi[2]+pad[2]),xlabel='World X (m)',ylabel='World Y (m)',zlabel='Height (m)')
    ax.set_box_aspect(hi-lo+2*pad);ax.view_init(elev=math.degrees(math.asin(1/math.sqrt(3))),azim=-135)
    ax.set_title(('DVL ranges only' if mode=='dvl-only' else 'DVL + assumed FLS/plane identity')+f'\nAll {len(points)} conditional hypotheses / unqualified')
    fig.colorbar(ScalarMappable(norm=norm,cmap=cmap),ax=ax,shrink=.55,pad=.08,label='Height (m; blue to red)')
ax=fig.add_subplot(2,3,6);names=['DVL 32.01 s','DVL 33.01 s','FLS 3.01 s'];x=np.arange(3)
before=np.abs(inspection['dvl-fls']['before_nonlinear_residual_m']);after=np.array([abs(v['residual_m']) for v in inspection['dvl-fls']['after_rows']])
ax.bar(x-.18,before,.36,label='Before update',color='#536d92');ax.bar(x+.18,after,.36,label='Nonlinear reevaluation',color='#d66e3e')
ax.set_xticks(x,names);ax.set_yscale('log');ax.set_ylabel('Absolute original range residual (m)');ax.legend(loc='upper left',fontsize=9);ax.set_title('Actual range equations after native optimization')
for bars,offset in ((before,-.18),(after,.18)):
    for i,value in enumerate(bars):ax.text(i+offset,value*1.2,f'{value:.4f}',ha='center',fontsize=8)
ax.grid(axis='y',alpha=.15)
fig.suptitle('Uninitialized negative control / all hypotheses retained / candidate rejected by the corrected preparation',fontsize=15)
fig.supxlabel('Corrected nonlinear preparation rejects this same-plane candidate before native installation. Plots preserve the uninitialized negative control; no accepted map.\n'
    'Native updates use IMU + pressure + DVL; eight cone readings remain support-only.\n'
    'Cloud rows were acquired at 3.01 / 32.01 / 33.01 s and processed at 33.01 s; context is 35.00 s. These are conditional direction hypotheses, not independent XYZ observations.\n'
    'RGB: 1% transmission at 2 m, particles, sunlight/caustics, wave water. Context disables haze/absorption only. Both clouds use 64 pt2 markers and common Turbo limits.\n'
    'Assessment-only initial yaw/translation, scale 1; grey boxes are scene geometry. Raw beam ranges, original 102 point hypotheses and all forty prediction errors remain preserved.',fontsize=8)
path=base/'comparison.png';fig.savefig(path,dpi=140,bbox_inches='tight',pad_inches=.18);plt.close(fig)
(base/'comparison.json').write_text(json.dumps(dict(scope='Read-only conditional range hypotheses; no accepted native SLAM cloud',
    point_counts={k:len(v) for k,v in clouds.items()},marker_area_pt2=64,height_limits_m=[-4.9,-1.5],
    rgb_stamp_ns=stamp,sonar_stamp_ns=stamp,context_stamp_ns=stamp,factor_delivery_stamp_ns=33010000000,
    original_factor_stamps_ns=[3010000000,32010000000,33010000000],map_qualified=False,
    source_sha256={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in (path,data/'inputs'/camera['file'],data/'inputs'/sonar_row['file'],old/'context-reference-35/isometric-rgb.png')}),indent=2)+'\n')
print('Rendered all measured-range hypotheses:',{k:len(v) for k,v in clouds.items()})
