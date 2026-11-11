"""Plot actual same-mount RGB/sonar and a labelled native field re-render."""
from pathlib import Path
import json
import math
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from PIL import Image

ROOT = Path(__file__).resolve().parent
project=ROOT.parents[2]
sys.path[:0]=[str(project/'tools'),str(project/'libraries/posim-slam/tools')]
from evaluate_vio import matrix


def read(name):
    return json.loads((ROOT / name).read_text())


def rpy(roll, pitch, yaw):
    cr, sr, cp, sp, cy, sy = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch), math.cos(yaw), math.sin(yaw)
    return np.array([[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                     [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr], [-sp, cp*sr, cp*cr]])


selected = read('field-audit.json')['figure_frame']
rgb_row = selected['reference_rgb']
with np.load(ROOT / 'field' / rgb_row['file'], allow_pickle=False) as data:
    rgb = data['data'].reshape(int(data['height']), int(data['step']))[:, :3*int(data['width'])].reshape(int(data['height']), int(data['width']), 3)
figure = plt.figure(figsize=(16, 13), dpi=120)
grid = figure.add_gridspec(2, 2, height_ratios=[1, 1.35], hspace=.25, wspace=.25)
rgb_axis, sonar_axis, context_axis = figure.add_subplot(grid[0, 0]), figure.add_subplot(grid[0, 1]), figure.add_subplot(grid[1, :])
rgb_axis.imshow(rgb)
focal=rgb.shape[1]/(2*math.tan(math.pi/3))
fan=focal*math.tan(math.radians(10))
for yy in (rgb.shape[0]/2-fan,rgb.shape[0]/2+fan):rgb_axis.axhline(yy,color='#ffba61',lw=1.4)
rgb_axis.text(.02,.03,'Orange: sonar vertical fan +/-10 deg',transform=rgb_axis.transAxes,color='#ffba61',fontsize=9,bbox=dict(facecolor='black',alpha=.65,edgecolor='none'))
rgb_axis.set_title(f'RGB at sonar origin/direction | {rgb_row["stamp_ns"]/1e9:.2f} s | 2 m visibility')
rgb_axis.axis('off')
with np.load(ROOT / 'field' / selected['file'], allow_pickle=False) as data:
    angles = np.arctan2(data['beam_directions'][:, 1], data['beam_directions'][:, 0])
    ranges = data['ranges']
    range_edges=np.r_[max(0.,ranges[0]-(ranges[1]-ranges[0])/2),(ranges[:-1]+ranges[1:])/2,min(15.,ranges[-1]+(ranges[-1]-ranges[-2])/2)]
    angle_edges=np.r_[-math.pi/3,(angles[:-1]+angles[1:])/2,math.pi/3]
    assert len(range_edges)==len(ranges)+1 and len(angle_edges)==len(angles)+1
    image = sonar_axis.pcolormesh(-range_edges[:,None]*np.sin(angle_edges),range_edges[:,None]*np.cos(angle_edges),
        data['intensity_db'], cmap='gray',vmin=20,vmax=120,shading='flat',rasterized=True)
sonar_axis.set(xlabel='Starboard (-sensor y), m', ylabel='Forward (+sensor x), m', xlim=(-15, 15), ylim=(0, 15), aspect='equal')
sonar_axis.set_title(f'Actual native sonar | {selected["stamp_ns"]/1e9:.2f} s | 15 m / 599 x 256')
sonar_axis.set_facecolor('#15252c')
figure.colorbar(image, ax=sonar_axis, fraction=.035, label='Raw intensity, dB')

context = np.asarray(Image.open(ROOT / 'isometric-rgb.png'))
context_axis.imshow(context)
context_axis.set_xlim(250,980)
context_axis.set_ylim(420,40)
context_axis.axis('off')
context_axis.set_title('Native isometric scene at the recorded body pose | assessment-only clear-water re-render')
proof = read('context-camera.json')
pose = np.array(list(map(float, proof['camera']['pose'].split())))
camera_rotation = rpy(*pose[3:])
focal = context.shape[1] / (2*math.tan(proof['camera']['horizontal_fov']/2))


def pixel(point):
    local = (np.asarray(point)-pose[:3]) @ camera_rotation
    assert local[0] > 0
    return np.array([context.shape[1]/2-focal*local[1]/local[0], context.shape[0]/2-focal*local[2]/local[0]])


body_position, body_rotation = np.array(proof['body_position']), np.array(proof['body_rotation'])
origin = pixel(body_position)
context_axis.annotate('BlueROV2', xy=origin, xytext=origin+[-110, -45], color='white',
    arrowprops=dict(arrowstyle='-', color='white'), bbox=dict(facecolor='black', alpha=.6, edgecolor='none'))
calibration = read('calibration.json')['sonar']
sonar_origin = body_position + body_rotation @ np.array(calibration['sonar_mount']['translation'])
sonar_rotation = body_rotation @ matrix(calibration['sonar_mount']['rotation'])
for name, start, direction, color, offset in [
    ('Sonar + matched RGB, 45 deg down', sonar_origin, sonar_rotation[:, 0], '#ffba61', [-210, 85]),
    ('Vehicle RGB, horizontal', body_position+body_rotation@np.array([.4, 0, -.1]), body_rotation[:, 0], '#68e8ff', [-120, 50])]:
    a, b = pixel(start), pixel(start+2*direction)
    context_axis.annotate('', xy=b, xytext=a, arrowprops=dict(arrowstyle='->', color=color, lw=2))
    context_axis.annotate(name, xy=b, xytext=b+offset, color=color, fontsize=9,
        bbox=dict(facecolor='black', alpha=.65, edgecolor='none'))
for index, item in enumerate(read('scene-metadata.json')['obstacles']):
    if item['shape'] != 'box' or 'block' not in item['name']:
        continue
    p = pixel([item['x'], item['y'], item['z']+item['height_m']/2])
    offset = [(70,50),(-10,70),(55,-20)][index]
    context_axis.annotate(f'Block {index+1}: {item["height_m"]:.2f} m', xy=p, xytext=p+offset,
        color='white', fontsize=10, arrowprops=dict(arrowstyle='-', color='white'),
        bbox=dict(facecolor='black', alpha=.6, edgecolor='none'))
figure.suptitle('BlueROV2 / Orca5 inspection field | 45-degree inspection mount, unchanged acoustic physics', fontsize=17)
figure.text(.5, .025,
    'Top: original RGB and sonar, matched within 10 ms; RGB keeps 2 m visibility, particles, lighting and caustics. Near is at the bottom of the sonar.\n'
    'Bottom: native RGB re-render at the recorded pose with haze/absorption off; lighting, caustics, particles and waves remain. Wave/particle phase differs.\n'
    'Scene geometry, comparison cameras and simulator poses are assessment-only. The acquisition contains no running SLAM estimator.',
    ha='center', va='bottom', fontsize=10, linespacing=1.6)
figure.subplots_adjust(left=.055, right=.955, top=.93, bottom=.12)
figure.savefig(ROOT / 'sensor-comparison.png')
plt.close(figure)
