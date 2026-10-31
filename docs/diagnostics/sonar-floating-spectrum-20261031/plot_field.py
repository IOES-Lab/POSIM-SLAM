"""Render retained source images in metric coordinates without synthesizing views."""
from pathlib import Path
import json

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

ROOT = Path(__file__).resolve().parent


def read(name):
    return json.loads((ROOT / name).read_text())


def rgb(row):
    with np.load(ROOT / 'field' / row['file'], allow_pickle=False) as data:
        key = 'source_data' if 'source_data' in data else 'data'
        width, height = int(data['width']), int(data['height'])
        step = int(data['source_step'] if 'source_step' in data else data['step'])
        return data[key].reshape(height, step)[:, :width * 3].reshape(height, width, 3)


def main():
    selected = read('field-audit.json')['figure_frame']
    camera = read('figure-camera.json')
    figure, axes = plt.subplots(2, 2, figsize=(16, 11), dpi=120)
    for axis, row, title in [
        (axes[0, 0], selected['reference_rgb'], 'RGB at the sonar mount'),
        (axes[1, 0], camera, 'Vehicle camera: horizontal mount'),
        (axes[1, 1], selected['isometric_rgb'], 'Recorded isometric field RGB')]:
        axis.imshow(rgb(row))
        axis.set_title(f'{title} | {row["stamp_ns"] / 1e9:.2f} s', fontsize=13)
        axis.axis('off')
    with np.load(ROOT / 'field' / selected['file'], allow_pickle=False) as data:
        angles = np.arctan2(data['beam_directions'][:, 1], data['beam_directions'][:, 0])
        ranges = data['ranges']
        lateral = -ranges[:, None] * np.sin(angles)
        forward = ranges[:, None] * np.cos(angles)
        image = axes[0, 1].pcolormesh(lateral, forward, data['intensity_db'],
            shading='nearest', cmap='gray', vmin=20, vmax=120, rasterized=True)
    axis = axes[0, 1]
    axis.set_title(f'Native sonar | {selected["stamp_ns"] / 1e9:.2f} s | 599 x 256', fontsize=13)
    axis.set(xlabel='Starboard distance (-sensor y), m', ylabel='Forward distance (sensor x), m',
             xlim=(-15, 15), ylim=(0, 15), aspect='equal')
    axis.set_facecolor('#18232b')
    figure.colorbar(image, ax=axis, label='Raw intensity, dB', fraction=.033, pad=.02)
    figure.suptitle('Orca5 / BlueROV2 | corrected floating sonar spectrum | 15 m', fontsize=18)
    figure.text(.5, .035,
        'Sonar/reference RGB share base_link and mount (x=0.24, z=-0.10 m; pitch 20.1 deg down). '
        'Vehicle camera: x=0.40, z=-0.10 m; pitch 0 deg.\n'
        'RGB uses the recorded 2 m underwater visibility, lighting, particles and seabed caustics. '
        'Acoustic data retain the beam-level source output. Near is at the bottom.\n'
        'Comparison cameras and simulator poses are assessment-only. This recording contains no running SLAM estimator.',
        ha='center', va='bottom', fontsize=10, linespacing=1.7)
    figure.tight_layout(rect=(.015, .13, .985, .95), h_pad=2, w_pad=2)
    figure.savefig(ROOT / 'field-comparison.png')
    plt.close(figure)


if __name__ == '__main__':
    main()
