# SPDX-License-Identifier: Apache-2.0
"""Static assessment overview; reuses the retained native field camera only."""
import argparse
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.colors import Normalize, ListedColormap
from PIL import Image

ROOT = Path(__file__).resolve().parent


def matrix(q):
    x,y,z,w = np.asarray(q)/np.linalg.norm(q)
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
        [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
        [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])


def main(records):
    from fit import sha
    seal = json.loads((ROOT/'construction-seal.json').read_text())
    for name,digest in seal['files'].items():assert sha(ROOT/name)==digest
    context = records/'sonar-translated-view-20261031'
    image = np.asarray(Image.open(context/'context-isometric-rgb.png'))
    receipt = json.loads((context/'context-receipt.json').read_text())
    calibration = json.loads((context/'calibration.json').read_text())
    scene_path = records/'sonar-inspection-scene-20261031/scene-assessment-only.npz'
    with np.load(scene_path,allow_pickle=False) as z:scene = dict(z)
    clouds = np.load(ROOT/'assessed-clouds.npz',allow_pickle=False)
    assessment = json.loads((ROOT/'assessment.json').read_text())
    camera = np.fromstring(receipt['camera']['pose'],sep=' ')
    roll,pitch,yaw = camera[3:]
    cr,sr,cp,sp,cy,sy = np.cos(roll),np.sin(roll),np.cos(pitch),np.sin(pitch),np.cos(yaw),np.sin(yaw)
    rotation = np.array([[cy*cp,cy*sp*sr-sy*cr,cy*sp*cr+sy*sr],
        [sy*cp,sy*sp*sr+cy*cr,sy*sp*cr-cy*sr],[-sp,cp*sr,cp*cr]])
    focal = image.shape[1]/(2*np.tan(receipt['camera']['horizontal_fov']/2))
    def project(points):
        q = (np.asarray(points)-camera[:3])@rotation
        assert np.all(q[...,0]>0)
        return np.stack((image.shape[1]/2-focal*q[...,1]/q[...,0],
                         image.shape[0]/2-focal*q[...,2]/q[...,0]),axis=-1)
    body,attitude = np.array(receipt['body_position']),np.array(receipt['body_rotation'])
    sonar = calibration['sonar']['sonar_mount']
    origin = body+attitude@sonar['translation']
    direction = attitude@matrix(sonar['rotation'])[:,0]
    cam = calibration['camera']['camera_mount']
    camera_origin = body+attitude@cam['translation']
    camera_direction = attitude@matrix(cam['rotation'])[:,2]
    arrows = [(origin,origin+1.5*direction,'#ee9822'),
              (camera_origin,camera_origin+1.5*camera_direction,'#168bb7')]
    allpoints = np.vstack([clouds[k+'_world'] for k in ['initial_prism','fitted_prism']])
    references = np.vstack([v.reshape(-1,3) for k,v in scene.items() if k!='seabed']+
        [allpoints,np.array([body]+[end for _,end,_ in arrows])])
    bounds = project(references)
    xlim = max(0,bounds[:,0].min()-45),min(image.shape[1],bounds[:,0].max()+45)
    ylim = min(image.shape[0],bounds[:,1].max()+45),max(0,bounds[:,1].min()-45)
    colours = ListedColormap(plt.get_cmap('turbo')(np.linspace(.10,.95,256)),name='turbo_blue_red')
    norm = Normalize(allpoints[:,2].min(),allpoints[:,2].max())
    fig,axes = plt.subplots(1,3,figsize=(20,8),layout='constrained')
    axes[0].imshow(image)
    axes[0].set_title('Retained native field RGB\nExact recorded isometric camera',fontsize=13)
    for ax,label in zip(axes[1:],['initial_prism','fitted_prism']):
        ax.set_facecolor('#f5f7f8')
        for name,triangles in scene.items():
            if name=='seabed':continue
            for t in triangles:
                xy = project(np.vstack((t,t[0])))
                ax.plot(*xy.T,color='#b6c0c4',lw=.7,alpha=.55)
        points = clouds[label+'_world'];xy = project(points)
        ax.scatter(*xy.T,c=points[:,2],cmap=colours,norm=norm,s=64,
                   alpha=.75,edgecolors='none')
        s = assessment['families'][label]
        title = 'Initial finite shape' if label=='initial_prism' else 'Fitted finite shape'
        e = s['all_surface_error_m']
        ax.set_title(f'{title}: all {len(points)} model samples\nSurface RMSE {e["rmse"]*100:.2f} cm; '
                     f'{e["beyond_10cm"]} above diagnostic 10 cm',fontsize=12)
    for ax in axes:
        for start,end,color in arrows:
            a,b = project(np.array([start,end]))
            ax.annotate('',xy=b,xytext=a,arrowprops=dict(arrowstyle='->',color=color,lw=2))
        xy = project(body)
        ax.scatter([xy[0]],[xy[1]],color='#30383b',s=35,zorder=5)
        ax.set(xlim=xlim,ylim=ylim,aspect='equal');ax.axis('off')
    fig.colorbar(plt.cm.ScalarMappable(norm=norm,cmap=colours),ax=list(axes[1:]),
                 orientation='horizontal',shrink=.8,pad=.02,label='Shared complete height range (m): blue low → red high')
    fig.suptitle('Fit finite object + ground to original translated sonar; fixed sensor navigation\n'
                 'One restrictive shape hypothesis, no accepted SLAM map or pose update',fontsize=17)
    fig.supxlabel('All panels use the exact same retained camera projection and crop. All 548 surface samples per shape remain; markers 64 pt².\n'
        'Orange: sonar and same-mount RGB direction (35° down); blue: vehicle RGB direction; black: vehicle position. Grey wireframe is assessment-only scene geometry.\n'
        'RGB context disables haze/absorption for field inspection; lighting, caustics, particles and waves remain at another phase. Actual estimator camera retains 2 m visibility.\n'
        'The cloud contains synthetic samples of the fitted shape, not independently observed landmarks. Unmeasured rear faces, pose/plane uncertainty and extra objects remain unresolved.',fontsize=10)
    fig.savefig(ROOT/'full-cloud-comparison.png',dpi=140,bbox_inches='tight',pad_inches=.12)
    plt.close(fig)
    inputs = json.loads((ROOT/'inputs.json').read_text())
    fig,axes = plt.subplots(2,3,figsize=(15,10),layout='constrained')
    frames = inputs['frames']
    dbs = []
    for frame in frames:
        with np.load(ROOT/'raw'/frame['file'],allow_pickle=False) as z:dbs.append(z['intensity_db'].copy())
    finite = np.concatenate([a[np.isfinite(a)] for a in dbs])
    low,high = np.quantile(finite,[.02,.998])
    for ax,frame,db in zip(axes.ravel(),frames,dbs):
        with np.load(ROOT/'raw'/frame['file'],allow_pickle=False) as z:
            r = z['ranges'];a = np.arctan2(z['beam_directions'][:,1],z['beam_directions'][:,0])
        re = np.r_[0.,(r[:-1]+r[1:])/2,r[-1]+(r[-1]-r[-2])/2]
        ae = np.r_[a[0],(a[:-1]+a[1:])/2,a[-1]]
        rr,aa = np.meshgrid(re,ae,indexing='ij')
        ax.set_facecolor('black')
        last = ax.pcolormesh(-rr*np.sin(aa),rr*np.cos(aa),db,cmap='gray',vmin=low,vmax=high,
                            shading='flat',rasterized=True)
        q = np.array(frame['pixels']);x = -r[q[:,0]]*np.sin(a[q[:,1]]);y = r[q[:,0]]*np.cos(a[q[:,1]])
        ax.scatter(x,y,s=10,facecolors='none',edgecolors='#25bdd7',linewidths=.6)
        group = 'Fit' if frame['index'] in [49,88,151] else 'Unused temporal view'
        ax.set_title(f'{group}: frame {frame["index"]}, {frame["stamp_ns"]/1e9:.2f} s',fontsize=11)
        ax.set(xlim=(-13.1,13.1),ylim=(0,15),aspect='equal',xlabel='Starboard distance (m)',ylabel='Forward slant projection (m)')
    fig.colorbar(last,ax=list(axes.ravel()),shrink=.7,pad=.015,label='Raw power (dB), one shared display scale')
    fig.suptitle('Original 15 m sonar images and frozen 48 queries per view\n'
                 'Distance axes: near at bottom; far at top. No image-derived XYZ point factor.',fontsize=15)
    fig.supxlabel('Cyan circles: fixed queries chosen before fitting, including broad ground controls. All 599×256 raw values remain in each NPZ.\n'
                  'Display levels use shared finite-power 2% / 99.8% quantiles; absent returns remain black. The comparison is development evidence, not independent-scene validation.',fontsize=10)
    fig.savefig(ROOT/'raw-sonar-queries.png',dpi=130,bbox_inches='tight',pad_inches=.12)
    plt.close(fig)
    print('Retained exact-camera full-cloud overview and all six original sonar query views')


if __name__=='__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--records',required=True,type=Path)
    main(parser.parse_args().records)
