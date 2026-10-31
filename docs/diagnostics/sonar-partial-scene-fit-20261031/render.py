# SPDX-License-Identifier: Apache-2.0
"""Complete finite hypotheses and a separately labelled recorded-camera crop."""
import argparse
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.colors import Normalize, ListedColormap
from PIL import Image
from scipy.spatial.transform import Rotation
from run_fit import ROOT, FAMILIES, sha, write


def main(records):
    seal = json.loads((ROOT/'construction-seal.json').read_text())
    for name,digest in seal['files'].items():
        assert sha(ROOT/name)==digest
    context = records/'sonar-translated-view-20261031'
    image = np.asarray(Image.open(context/'context-isometric-rgb.png'))
    receipt = json.loads((context/'context-receipt.json').read_text())
    calibration = json.loads((context/'calibration.json').read_text())
    assessment = json.loads((ROOT/'assessment.json').read_text())
    clouds = np.load(ROOT/'assessed-clouds.npz',allow_pickle=False)
    camera = np.fromstring(receipt['camera']['pose'],sep=' ')
    camera_r = Rotation.from_euler('xyz',camera[3:]).as_matrix()
    body, attitude = np.array(receipt['body_position']), np.array(receipt['body_rotation'])
    arrows = []
    for kind,colour in [('sonar','#ee9822'),('camera','#168bb7')]:
        mount = calibration[kind][kind+'_mount']
        origin = body+attitude@mount['translation']
        direction = attitude@Rotation.from_quat(mount['rotation']).as_matrix()[:,0 if kind=='sonar' else 2]
        arrows.append((origin,origin+1.5*direction,colour))
    allpoints = np.vstack([clouds[n+'_world'] for n in FAMILIES])
    colours = ListedColormap(plt.get_cmap('turbo')(np.linspace(.10,.95,256)))
    norm = Normalize(allpoints[:,2].min(),allpoints[:,2].max())
    fig = plt.figure(figsize=(21,8),layout='constrained')
    ax = fig.add_subplot(131);ax.imshow(image);ax.axis('off')
    ax.set_title('Retained native field RGB\nRecorded isometric perspective; full capture')
    axes = []
    for k,name in enumerate(FAMILIES):
        ax = fig.add_subplot(1,3,k+2,projection='3d');axes.append(ax)
        points = clouds[name+'_world'];ground = clouds[name+'_ground_mask']
        for mask,alpha in [(ground,.25),(~ground,.9)]:
            ax.scatter(*points[mask].T,c=points[mask,2],cmap=colours,norm=norm,s=64,
                       alpha=alpha,edgecolors='none',depthshade=False)
        ax.scatter(*body,color='#202b33',marker='^',s=65)
        for start,end,colour in arrows:
            ax.quiver(*start,*(end-start),color=colour,arrow_length_ratio=.2,lw=2)
        ax.view_init(elev=np.rad2deg(camera[4]),azim=np.rad2deg(camera[5])+180)
        low,high = allpoints.min(axis=0),allpoints.max(axis=0)
        ax.set(xlim=(low[0],high[0]),ylim=(low[1],high[1]),zlim=(low[2]-.1,high[2]+.1),
               xlabel='World X (m)',ylabel='World Y (m)')
        ax.set_zticks([])  # Physical aspect makes sub-metre height ticks overlap.
        ax.set_box_aspect(np.maximum(high-low,.5))
        stats = assessment['families'][name]
        label = 'Initial two-reflector scene' if k==0 else 'Fitted partial scene'
        ax.set_title(f'{label}: all {len(points)} samples\n'
                     f'Object-hypothesis RMSE {stats["candidate_object_error_m"]["rmse"]*100:.2f} cm')
    fig.colorbar(plt.cm.ScalarMappable(norm=norm,cmap=colours),ax=axes,orientation='horizontal',
                 shrink=.8,pad=.02,label='Common complete-mesh height scale: blue low → red high (m)')
    fig.suptitle('Original sonar → finite multiple-reflector / ground hypotheses\n'
                 'No accepted SLAM cloud or acoustic pose update',fontsize=17)
    fig.supxlabel('All 3,939 samples per mesh remain, including unmeasured ground and hidden faces; markers 64 pt². Faint points: extrapolated ground prior.\n'
        'Whole-mesh clouds are orthographic, with the recorded camera viewing direction; they are not the RGB perspective projection.\n'
        'Orange: sonar / same-mount RGB direction (35° down); blue: vehicle RGB direction; black: vehicle position.\n'
        'RGB inspection context has haze/absorption off; lighting, caustics, particles and waves are from another phase. Actual estimator RGB retains 2 m visibility.',fontsize=10)
    fig.savefig(ROOT/'complete-scene-comparison.png',dpi=135,bbox_inches='tight',pad_inches=.12)
    plt.close(fig)
    focal = image.shape[1]/(2*np.tan(receipt['camera']['horizontal_fov']/2))
    def project(points):
        local = (np.asarray(points)-camera[:3])@camera_r
        valid = local[:,0]>0
        xy = np.column_stack((image.shape[1]/2-focal*local[:,1]/local[:,0],
                              image.shape[0]/2-focal*local[:,2]/local[:,0]))
        return xy,valid
    object_xy = []
    for name in FAMILIES:
        xy,valid = project(clouds[name+'_world'][~clouds[name+'_ground_mask']])
        object_xy.append(xy[valid])
    bounds = np.vstack(object_xy+[project(np.array([body]+[b for a,b,c in arrows]))[0]])
    xlim = max(0,bounds[:,0].min()-70),min(image.shape[1],bounds[:,0].max()+70)
    ylim = min(image.shape[0],bounds[:,1].max()+70),max(0,bounds[:,1].min()-70)
    fig,axes = plt.subplots(1,3,figsize=(18,8),layout='constrained')
    axes[0].imshow(image);axes[0].set_title('Native field RGB: same camera crop')
    visibility = {}
    for ax,name in zip(axes[1:],FAMILIES):
        points = clouds[name+'_world'][~clouds[name+'_ground_mask']]
        xy,valid = project(points)
        onscreen = valid & (xy[:,0]>=xlim[0]) & (xy[:,0]<=xlim[1]) & (xy[:,1]>=ylim[1]) & (xy[:,1]<=ylim[0])
        visibility[name] = dict(object_samples=len(points),inside_recorded_crop=int(sum(onscreen)),
                               behind_camera=int(sum(~valid)),complete_mesh_samples=len(clouds[name+'_world']))
        ax.scatter(*xy[valid].T,c=points[valid,2],cmap=colours,norm=norm,s=64,edgecolors='none',alpha=.7)
        ax.set_facecolor('#f5f7f8')
        ax.set_title(name.replace('_',' ')+f'\nAll {len(points)} object-hypothesis samples in crop')
    for ax in axes:
        for start,end,colour in arrows:
            xy,_ = project(np.array([start,end]))
            ax.annotate('',xy=xy[1],xytext=xy[0],arrowprops=dict(arrowstyle='->',color=colour,lw=2))
        xy,_ = project(np.array([body]));ax.scatter(*xy[0],color='#202b33',s=45)
        ax.set(xlim=xlim,ylim=ylim,aspect='equal');ax.axis('off')
    fig.colorbar(plt.cm.ScalarMappable(norm=norm,cmap=colours),ax=list(axes[1:]),orientation='horizontal',
                 shrink=.8,label='Same complete-mesh Turbo height scale (m)')
    fig.suptitle('Exact recorded camera projection and crop: object hypotheses only',fontsize=16)
    fig.supxlabel('Ground-prior samples are shown in the separate whole-mesh figure and remain in both complete PLYs; no error/visibility trimming.\n'
        'The native RGB and candidate objects use identical recorded intrinsics/extrinsics. These are synthetic shape samples, not verified sonar landmarks.\n'
        'Environment: haze/absorption off in inspection RGB; lighting/caustics/particles/waves remain at another phase. Estimator camera: 2 m visibility.',fontsize=10)
    fig.savefig(ROOT/'object-camera-comparison.png',dpi=135,bbox_inches='tight',pad_inches=.12)
    plt.close(fig)
    write(ROOT/'render-receipt.json',dict(visibility=visibility,projection='recorded camera and separate whole-mesh orthographic view',
          references={str(p.relative_to(records)):sha(p) for p in [context/'context-isometric-rgb.png',
              context/'context-receipt.json',context/'calibration.json']},markers_pt_squared=64))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--records',required=True,type=Path)
    main(parser.parse_args().records)
