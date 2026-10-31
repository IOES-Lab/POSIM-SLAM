# SPDX-License-Identifier: Apache-2.0
"""Plot all conditional observations and a separately labelled mean-model control."""
from pathlib import Path
import json
import gzip
import lzma
import importlib.util
import sys
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.colors import Normalize,ListedColormap
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from PIL import Image
import numpy as np

ROOT=Path(__file__).resolve().parent
SOURCE=ROOT.parent/'sonar-translated-view-20261031'
def matrix(q):
    x,y,z,w=np.asarray(q)/np.linalg.norm(q)
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                     [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                     [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])

spec=importlib.util.spec_from_file_location('public_geometry',ROOT.parent/'sonar-inspection-scene-20261031/verify.py')
geometry=importlib.util.module_from_spec(spec);spec.loader.exec_module(geometry)
surface_distances=geometry.surface_distances

def read(path):
    if not path.exists():
        path=next(p for p in [path.with_suffix('.json.gz'),path.with_suffix('.json.xz')] if p.exists())
    raw=path.read_bytes()
    if path.suffix=='.gz':raw=gzip.decompress(raw)
    elif path.suffix=='.xz':raw=lzma.decompress(raw)
    return json.loads(raw)
stats=read(ROOT/'assessment-summary.json')['modes']
clouds={}
for mode in ['angular_median','physical_plane']:
    with np.load(ROOT/(mode+'-assessed.npz'),allow_pickle=False) as f:clouds[mode]=f['world']
with np.load(ROOT.parent/'sonar-inspection-scene-20261031/scene-assessment-only.npz',allow_pickle=False) as f:scene=dict(f)
points=np.vstack(list(clouds.values()))
lower=np.minimum(points.min(axis=0),[7,-3.1,-4])-[.4,.4,.15]
upper=np.maximum(points.max(axis=0),[12.5,3.7,-1.8])+[.4,.4,.15]
colors=ListedColormap(plt.get_cmap('turbo')(np.linspace(.10,.95,256)),name='turbo_blue_red')
norm=Normalize(float(points[:,2].min()),float(points[:,2].max()))
fig=plt.figure(figsize=(23,8.5),layout='constrained')
grid=fig.add_gridspec(2,3,height_ratios=[1,.045],width_ratios=[1,1,1])
rgb=fig.add_subplot(grid[0,0]);image=np.asarray(Image.open(SOURCE/'context-isometric-rgb.png'))
rgb.imshow(image);rgb.axis('off')
proof=read(SOURCE/'context-receipt.json');cal=read(SOURCE/'calibration.json')
eye=np.fromstring(proof['camera']['pose'],sep=' ');roll,pitch,yaw=eye[3:]
cr,sr,cp,sp,cy,sy=np.cos(roll),np.sin(roll),np.cos(pitch),np.sin(pitch),np.cos(yaw),np.sin(yaw)
view=np.array([[cy*cp,cy*sp*sr-sy*cr,cy*sp*cr+sy*sr],
               [sy*cp,sy*sp*sr+cy*cr,sy*sp*cr-cy*sr],[-sp,cp*sr,cp*cr]])
focal=image.shape[1]/(2*np.tan(proof['camera']['horizontal_fov']/2))
def pixel(point):
    x,y,z=(np.asarray(point)-eye[:3])@view
    assert x>0
    return [image.shape[1]/2-focal*y/x,image.shape[0]/2-focal*z/x]
body,attitude=np.array(proof['body_position']),np.array(proof['body_rotation'])
sonar=cal['sonar']['sonar_mount'];origin=body+attitude@sonar['translation']
direction=attitude@matrix(sonar['rotation'])[:,0]
camera=cal['camera']['camera_mount'];camera_origin=body+attitude@camera['translation']
camera_direction=attitude@matrix(camera['rotation'])[:,2]
for start,end,color in [(origin,origin+1.5*direction,'#ffb34a'),
                        (camera_origin,camera_origin+1.5*camera_direction,'#59dfff')]:
    rgb.annotate('',xy=pixel(end),xytext=pixel(start),arrowprops=dict(arrowstyle='->',color=color,lw=2))
refs=np.vstack([t.reshape(-1,3) for name,t in scene.items() if name!='seabed']+
               [np.array([body,origin+1.5*direction,camera_origin+1.5*camera_direction])])
pixels=np.array([pixel(p) for p in refs])
rgb.set(xlim=(max(0,pixels[:,0].min()-60),min(image.shape[1],pixels[:,0].max()+60)),
        ylim=(min(image.shape[0],pixels[:,1].max()+60),max(0,pixels[:,1].min()-60)))
rgb.set_title('Native field RGB at recorded pose\nAssessment context: haze/absorption off',fontsize=12)
rgb.text(.025,.025,'Orange: sonar + same-mount RGB, 35° down\nCyan: vehicle RGB, forward / 2 m visibility',
         transform=rgb.transAxes,color='white',fontsize=9,bbox=dict(facecolor='black',alpha=.72,edgecolor='none'))
for column,(mode,title) in enumerate([('angular_median','Angular-median background'),
                                     ('physical_plane','Uniform physical-plane background')],1):
    ax=fig.add_subplot(grid[0,column],projection='3d')
    for name,triangles in scene.items():
        if name=='seabed':
            center=triangles.mean(axis=1)
            triangles=triangles[np.all((center[:,:2]>=lower[:2])&(center[:,:2]<=upper[:2]),axis=1)]
        ax.add_collection3d(Poly3DCollection(triangles,facecolor='#92a5ac',
            edgecolor='none' if name=='seabed' else '#627580',linewidth=.5,alpha=.12))
    data=clouds[mode];ax.scatter(*data.T,c=data[:,2],cmap=colors,norm=norm,s=64,
                               edgecolors='none',alpha=.68,depthshade=False)
    ax.set(xlim=tuple((lower[0],upper[0])),ylim=tuple((lower[1],upper[1])),zlim=tuple((lower[2],upper[2])),
           xlabel='East / x (m)',ylabel='North / y (m)',zlabel='Height / z (m)')
    ax.set_box_aspect(upper-lower);ax.view_init(elev=35.26438968,azim=-135)
    s=stats[mode]['all'];ax.set_title(f'{title}\nAll {len(data):,} correlated observations; surface RMSE {s["error_m"]["rmse"]:.3f} m\n'
        f'{s["beyond_10cm"]:,} beyond 10 cm; map unqualified',fontsize=11)
fig.colorbar(plt.cm.ScalarMappable(norm=norm,cmap=colors),cax=fig.add_subplot(grid[1,1:]),
             orientation='horizontal',label='Complete common height range (m): blue low → red high')
fig.suptitle('193 retained translated sonar frames / 62.87° actual target-bearing span\n'
             'Conditional image-region observations, not unique landmarks; no native pose update',fontsize=16)
fig.supxlabel('Every conditional output remains; no point/error trimming. Markers: 64 pt². Same isometric direction, separate field/cloud framing.\n'
              'Grey geometry is assessment reference. Original sonar + IMU/DVL/pressure navigation construct the observations; scene/truth/RGB enter assessment only.\n'
              'Native RGB context disables only haze/absorption; lighting, caustics, particles and waves remain at another phase. Actual sensor RGB retains 2 m visibility.',fontsize=10)
fig.savefig(ROOT/'full-cloud-comparison.png',dpi=140,bbox_inches='tight',pad_inches=.12);plt.close(fig)

oracle=read(ROOT/'oracle-mean-control.json')
fig=plt.figure(figsize=(16,9),layout='constrained');grid=fig.add_gridspec(2,3,height_ratios=[1,.65])
last=None
for column,case in enumerate(oracle['cases']):
    with np.load(ROOT/'field'/case['raw_file'],allow_pickle=False) as f:
        ranges=f['ranges'];a=np.arctan2(f['beam_directions'][:,1],f['beam_directions'][:,0])
    from posim_slam.sonar_surface_power import SurfacePowerModel,TriangleSurface
    model=SurfacePowerModel(a,np.linspace(-cal['sonar']['vertical_fov']/2,cal['sonar']['vertical_fov']/2,256),ranges,maximum_range_m=15.)
    supplied=TriangleSurface(np.concatenate((oracle['plane_triangles'],oracle['box_triangles'])))
    power=model.observation(supplied,case['source_position'],case['source_rotation']).forward(np.ones(len(supplied.triangles)))
    intensity=np.full(power.shape,-np.inf)
    np.log10(power,out=intensity,where=power>0);intensity*=10
    range_edges=np.r_[0.,(ranges[:-1]+ranges[1:])/2,ranges[-1]+(ranges[-1]-ranges[-2])/2]
    angle_edges=np.r_[a[0],(a[:-1]+a[1:])/2,a[-1]]
    rr,aa=np.meshgrid(range_edges,angle_edges,indexing='ij');x=-rr*np.sin(aa);y=rr*np.cos(aa)
    ax=fig.add_subplot(grid[0,column]);ax.set_facecolor('#182b33')
    last=ax.pcolormesh(x,y,intensity,cmap='gray',vmin=20,vmax=120,shading='flat',rasterized=True)
    ax.set(xlim=(-5.2,5.2),ylim=(0,6),xlabel='Starboard (m)',ylabel='Forward slant projection (m)')
    paired_world=np.array([p['point_world'] for p in case['points'] if p['paired']]).reshape(-1,3)
    error=surface_distances(paired_world,np.array(oracle['box_triangles'])) if len(paired_world) else np.array([])
    label=f'{error.max():.3f} m' if len(error) else 'unavailable'
    ax.set_title(f'Synthetic mean / {case["stamp_ns"]/1e9:.2f} s view\n'
                 f'Paired surface maximum: {label} ({case["paired_points"]} observations)',fontsize=11)
    ax.set_aspect('equal')
height=fig.add_subplot(grid[1,:])
for index,case in enumerate(oracle['cases']):
    for paired,color,label in [(False,'#a7b1ba','Other conditional outputs'),(True,'#1f77b4','Bright + recovery paired')]:
        selected=np.array([p['point_world'] for p in case['points'] if p['paired']==paired]).reshape(-1,3)
        values=surface_distances(selected,np.array(oracle['box_triangles'])) if len(selected) else np.array([])
        if len(values):height.scatter(np.full(len(values),index)+(np.arange(len(values))-(len(values)-1)/2)*.012,
                                 values,color=color,s=36,label=label if index==0 else None)
    height.scatter([index],[0.],marker='*',s=150,color='#c9362b',
                   label='Supplied edge/ground geometry: numerical zero' if index==0 else None)
height.axhline(.1,color='#9b673a',ls='--',lw=1,label='Diagnostic 10 cm reference')
height.set(xticks=range(3),xticklabels=[f'{c["stamp_ns"]/1e9:.2f} s view' for c in oracle['cases']],
           ylabel='Distance to known box surface (m)',ylim=(-.025,.71),xlim=(-.5,2.5))
height.grid(alpha=.2);height.legend(loc='upper right',ncol=2,fontsize=9)
fig.colorbar(last,ax=fig.axes[:3],shrink=.65,pad=.02,label='Conditional mean power (dB)')
fig.suptitle('Known 0.45 m box + ideal level ground + exact supplied poses; no speckle or pose noise\n'
             'Supplied geometric edges are exact; thresholded image points can miss the known box surface',fontsize=15)
fig.supxlabel('Assessment-only mathematical control, not a native sensor capture or estimator input. All conditional outputs are retained.\n'
              'Original source grid: 15 m / 599×256; display crop: 0–6 m. Same 5-sample median, 8 dB deficit and pairing rules.\n'
              'The controlled finite-object/ray/range-response model does not isolate the contribution of each image-formation stage.',fontsize=10)
fig.savefig(ROOT/'mean-boundary-control.png',dpi=140,bbox_inches='tight',pad_inches=.12);plt.close(fig)
print('Plotted complete 20,041 observations and separate ideal mean/edge controls')
