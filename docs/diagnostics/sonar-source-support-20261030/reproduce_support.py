"""Separated-view source-support diagnostic, with training-only arc seeding.

No geometry/truth, future sonar frame or held-out mask enters construction.
Absent image support is unresolved, not evidence of free space. This script
does not make correspondences or emit an accepted map.
"""
from pathlib import Path
import json
import sys
import time
import numpy as np

import argparse
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--input', type=Path, required=True)
parser.add_argument('--output', type=Path, required=True)
args = parser.parse_args()
input_root = args.input.resolve()
base = args.output.resolve()
base.mkdir(parents=True, exist_ok=False)
with np.load(input_root / 'joint-poses.npz') as data:
    stamps, body_positions, body_rotations = data['stamps_ns'], data['positions'], data['rotations']
with np.load(input_root / 'beam-response.npz') as data:
    ranges, angles = data['ranges'], data['angles']
cal = json.loads((input_root / 'calibration.json').read_text())['sonar']
mount, lever = np.array(cal['sonar_to_imu']), np.array(cal['imu_to_sonar'])
origins = body_positions + np.einsum('nij,j->ni', body_rotations, lever)
rotations = body_rotations @ mount
frames = {row['stamp_ns']: row for row in json.loads((input_root / 'source-image-summary.json').read_text())['records']}
raw = {row['stamp_ns']: row for row in json.loads((input_root / 'beam-source-bounds.json').read_text())['rows']}
voxel = .05
settings = dict(voxel_m=voxel, minimum_positive_views=3, minimum_baseline_m=.5,
    minimum_depth_span_m=.25, maximum_candidates=300000, diagnostic_three_sigma_m=.1,
    range_sigma_m=.025083611719310283, azimuth_sigma_rad=.008213314126139592,
    split_policy=['first two thirds train, final third validate', 'alternating two-frame blocks'],
    scope='training-only candidates; source support consistency, not association, occupancy or map admission')
(base / 'heldout-settings.json').write_text(json.dumps(settings, indent=2) + '\n')
records = {}
for name in ('chronological', 'blocked'):
    started = time.perf_counter()
    train = np.arange(len(stamps)) < (2 * len(stamps)) // 3 if name == 'chronological' else (np.arange(len(stamps)) // 2) % 2 == 0
    seeds = []
    for i in np.flatnonzero(train):
        row = raw[int(stamps[i])]
        for beam, index, lower in zip(row['beams'], row['range_indices'], row['lower']):
            if lower <= 0:
                continue
            radius, theta = ranges[index], angles[beam]
            steps = max(3, int(np.ceil(radius * np.pi / 9 / (voxel / 2))) + 1)
            elevation = np.linspace(-np.pi / 18, np.pi / 18, steps)
            rays = np.column_stack([np.cos(theta)*np.cos(elevation), np.sin(theta)*np.cos(elevation), np.sin(elevation)])
            seeds.append(np.floor((origins[i] + radius * rays @ rotations[i].T) / voxel).astype(np.int32))
    cells = np.unique(np.vstack(seeds), axis=0)
    assert len(cells) <= settings['maximum_candidates']
    points = (cells.astype(float) + .5) * voxel
    n = len(points)
    positive = np.zeros((n, len(stamps)), bool)
    visible = np.zeros_like(positive)
    pixels = np.zeros(positive.shape, np.int32)
    hessian = np.zeros((n, 3, 3))
    first = np.full((n, 3), np.nan)
    baseline, zlow, zhigh = np.zeros(n), np.full(n, np.inf), np.full(n, -np.inf)
    # Construct and freeze all training decisions before opening held-out masks.
    for is_training in (True, False):
        for i in np.flatnonzero(train == is_training):
            with np.load(input_root / 'source-masks' / frames[int(stamps[i])]['file']) as data:
                mask = np.unpackbits(data['contrast'])[:int(np.prod(data['shape']))].reshape(tuple(data['shape'])).astype(bool)
            local = (points - origins[i]) @ rotations[i]
            radius = np.linalg.norm(local, axis=1)
            horizontal = np.hypot(local[:, 0], local[:, 1])
            azimuth, elevation = np.arctan2(local[:, 1], local[:, 0]), np.arctan2(local[:, 2], horizontal)
            visible[:, i] = (local[:, 0] > 0) & (radius >= .4) & (radius <= ranges[-1]) & (azimuth >= angles[0]) & (azimuth <= angles[-1]) & (abs(elevation) <= np.pi / 18)
            ri = np.clip(np.searchsorted((ranges[:-1] + ranges[1:]) / 2, radius), 0, len(ranges)-1)
            bi = np.clip(np.searchsorted((angles[:-1] + angles[1:]) / 2, azimuth), 0, len(angles)-1)
            pixels[:, i] = ri * len(angles) + bi
            positive[:, i] = visible[:, i] & mask[ri, bi]
            if not is_training:
                continue
            selected = np.flatnonzero(positive[:, i])
            new = selected[~np.isfinite(first[selected, 0])]
            first[new] = origins[i]
            baseline[selected] = np.maximum(baseline[selected], np.linalg.norm(first[selected] - origins[i], axis=1))
            zlow[selected] = np.minimum(zlow[selected], origins[i, 2])
            zhigh[selected] = np.maximum(zhigh[selected], origins[i, 2])
            q = local[selected]
            range_j = (q / radius[selected, None]) @ rotations[i].T
            angle_j = np.column_stack([-q[:, 1]/horizontal[selected]**2, q[:, 0]/horizontal[selected]**2, np.zeros(len(q))]) @ rotations[i].T
            hessian[selected] += np.einsum('ni,nj->nij', range_j, range_j) / settings['range_sigma_m']**2 + np.einsum('ni,nj->nij', angle_j, angle_j) / settings['azimuth_sigma_rad']**2
        if is_training:
            eigen = np.linalg.eigvalsh(hessian)
            sigma = np.where(eigen[:, 0] > 3*np.finfo(float).eps*np.maximum(eigen[:, -1], 1.), 3/np.sqrt(np.maximum(eigen[:, 0], np.finfo(float).tiny)), np.inf)
            selected = (positive[:, train].sum(axis=1) >= 3) & (baseline >= .5) & (zhigh - zlow >= .25)
            optimistic = selected & (sigma <= .1)
            selected_before_validation = selected.copy()
            optimistic_before_validation = optimistic.copy()
    assert np.array_equal(selected, selected_before_validation) and np.array_equal(optimistic, optimistic_before_validation)
    test_visible, test_positive = visible[:, ~train].sum(axis=1), positive[:, ~train].sum(axis=1)
    def summary(mask):
        relevant = mask & (test_visible > 0)
        fraction = test_positive[relevant] / test_visible[relevant]
        return dict(candidates=int(mask.sum()), visible_in_validation=int(relevant.sum()),
            validation_positive_views=int(test_positive[mask].sum()), validation_visible_views=int(test_visible[mask].sum()),
            validation_support_fraction_p10_p50_p90=np.percentile(fraction, [10,50,90]).tolist() if len(fraction) else None,
            zero_positive_validation_views=int((mask & (test_positive == 0)).sum()))
    np.savez_compressed(base / ('heldout-' + name + '.npz'), cells=cells, points=points,
        stamps_ns=stamps, train=train, positive=positive, visible=visible, pixels=pixels,
        selected=selected, optimistic=optimistic, optimistic_three_sigma_m=sigma)
    records[name] = dict(training_stamps_ns=stamps[train].tolist(), validation_stamps_ns=stamps[~train].tolist(),
        seeds=int(train.sum()), candidates=n, support_selected=summary(selected), optimistic_selected=summary(optimistic),
        smallest_optimistic_three_sigma_m=float(sigma[selected].min()) if selected.any() else None, seconds=time.perf_counter()-started)
report = dict(settings=settings, folds=records, truth_used=False, accepted_map=False, pose_uncertainty_in_selection=False,
    navigation='same 52.5-second native snapshot; navigation sensors from all times, no sonar factors; retrospective local mapping diagnostic')
(base / 'heldout-summary.json').write_text(json.dumps(report, indent=2) + '\n')
print(json.dumps({name: {k: v for k, v in row.items() if not k.endswith('stamps_ns')} for name, row in records.items()}, indent=2))
