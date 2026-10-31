"""Draw all emitted hypotheses beside their actual native scene overview."""
from pathlib import Path
import json

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.colors import Normalize
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import numpy as np
from PIL import Image

ROOT = Path(__file__).resolve().parent


def main():
    summary = json.loads((ROOT / 'assessment-summary.json').read_text())
    modes = [('default_point', 'Intensity features'),
             ('default_echo_boundary', 'Echo / shadow boundaries')]
    clouds = {}
    for mode, _ in modes:
        with np.load(ROOT / (mode + '-assessed.npz'), allow_pickle=False) as data:
            clouds[mode] = data['world'].copy()
    all_points = np.concatenate(list(clouds.values()))
    assert len(all_points) == 96
    lower = np.minimum(all_points.min(axis=0), [7., -3.1, -4.]) - [.3, .3, .2]
    upper = np.maximum(all_points.max(axis=0), [12.5, 3.7, -1.8]) + [.3, .3, .2]
    norm = Normalize(float(all_points[:, 2].min()), float(all_points[:, 2].max()))
    with np.load(ROOT / 'scene-assessment-only.npz', allow_pickle=False) as data:
        scene = {key: data[key].copy() for key in data.files}
    fig = plt.figure(figsize=(18, 8), dpi=140)
    grid = fig.add_gridspec(2, 3, height_ratios=[1, .08], width_ratios=[1.3, 1, 1], wspace=.05)
    rgb = fig.add_subplot(grid[0, 0])
    image = np.asarray(Image.open(ROOT / 'isometric-rgb.png'))
    rgb.imshow(image)
    rgb.set(xlim=(250, 980), ylim=(430, 30))
    rgb.set_title('Actual native RGB overview\nBlueROV2, three blocks and an L wall', fontsize=13)
    rgb.axis('off')
    proof = json.loads((ROOT / 'context-camera.json').read_text())
    calibration = json.loads((ROOT / 'calibration.json').read_text())
    eye = np.array(list(map(float, proof['camera']['pose'].split())))
    roll, pitch, yaw = eye[3:]
    cr, sr, cp, sp, cy, sy = np.cos(roll), np.sin(roll), np.cos(pitch), np.sin(pitch), np.cos(yaw), np.sin(yaw)
    view = np.array([[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                     [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr], [-sp, cp*sr, cp*cr]])
    focal = image.shape[1]/(2*np.tan(proof['camera']['horizontal_fov']/2))

    def pixel(point):
        x, y, z = (np.asarray(point)-eye[:3]) @ view
        assert x > 0
        return [image.shape[1]/2-focal*y/x, image.shape[0]/2-focal*z/x]

    body, attitude = np.array(proof['body_position']), np.array(proof['body_rotation'])
    sonar_origin = body + attitude @ np.array(calibration['sonar']['sonar_mount']['translation'])
    for origin, direction, color in [
        (sonar_origin, attitude @ np.array([np.cos(.35),0,-np.sin(.35)]), '#ffb34a'),
        (body+attitude@np.array([.4,0,-.1]), attitude[:,0], '#59dfff')]:
        rgb.annotate('', xy=pixel(origin+2*direction), xytext=pixel(origin),
                     arrowprops=dict(arrowstyle='->', color=color, lw=2))
    rgb.text(.02, .025, '20.1 deg down sonar + matched RGB\nVehicle RGB: horizontal',
             transform=rgb.transAxes, color='white', fontsize=10,
             bbox=dict(facecolor='black', alpha=.65, edgecolor='none'))
    axes = []
    for index, (mode, title) in enumerate(modes, 1):
        ax = fig.add_subplot(grid[0, index], projection='3d')
        axes.append(ax)
        for name, triangles in scene.items():
            if name == 'seabed':
                centres = triangles.mean(axis=1)
                local = triangles[(centres[:, 0] >= lower[0]) & (centres[:, 0] <= upper[0])
                                  & (centres[:, 1] >= lower[1]) & (centres[:, 1] <= upper[1])]
                ax.add_collection3d(Poly3DCollection(local, facecolor='#92a5ac',
                    edgecolor='none', alpha=.13))
            else:
                ax.add_collection3d(Poly3DCollection(triangles, facecolor='#92a5ac',
                    edgecolor='#657780', linewidth=.6, alpha=.09))
        points = clouds[mode]
        ax.scatter(*points.T, c=points[:, 2], norm=norm, cmap='turbo', s=64,
                   edgecolors='#182c37', linewidths=.45, depthshade=False)
        ax.set(xlim=(lower[0], upper[0]), ylim=(lower[1], upper[1]), zlim=(lower[2], upper[2]),
               xlabel='East / x, m', ylabel='North / y, m', zlabel='Height / z, m')
        ax.set_box_aspect(upper-lower)
        ax.view_init(elev=35.26438968, azim=-135)
        row = summary[mode]
        ax.set_title(f'{title}: all {len(points)} points\n'
                     f'Surface RMSE {row["surface_error_m"]["rmse"]:.3f} m; '
                     f'{row["diagnostic_counts"]["0.1"]["outside"]}/{len(points)} beyond 10 cm', fontsize=12)
        ax.tick_params(labelsize=8)
        for axis in (ax.xaxis, ax.yaxis, ax.zaxis):
            axis.pane.set_facecolor('#edf3f5')
    color_axis = fig.add_subplot(grid[1, 1:])
    fig.colorbar(plt.cm.ScalarMappable(norm=norm, cmap='turbo'), cax=color_axis,
                 orientation='horizontal', label='Common height scale, m (blue: low; red: high)')
    fig.suptitle('Same physical recording / IMU + DVL + pressure + forward sonar\n'
                 'Unqualified point hypotheses: no points removed', fontsize=17, y=.97)
    fig.text(.5, .035,
        'RGB: assessment-only native re-render at 28.01 s; haze/absorption off to expose geometry; waves, light, particles and caustics retained.\n'
        'Clouds: complete 2.5-48.5 s replay; initial yaw/translation alignment only, unit scale. Same isometric direction, independent framing.\n'
        'Recorded RGB keeps 2 m visibility; these replays use no camera. Geometry and simulator poses enter assessment only. Every marker is 64 pt².',
        ha='center', va='bottom', fontsize=10, linespacing=1.5)
    fig.subplots_adjust(left=.015, right=.98, bottom=.21, top=.83)
    fig.savefig(ROOT / 'full-cloud-comparison.png')
    plt.close(fig)


if __name__ == '__main__':
    main()
