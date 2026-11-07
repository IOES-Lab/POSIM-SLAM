"""Diagnostic point marginals from existing public geometry/Gaussian blocks.

Training support is not a verified physical correspondence. Evaluate every
training-supported candidate unchanged; never use assessment errors to select.
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
from posim_slam.inertial import exp_so3, left_jacobian_so3
from posim_slam.sonar_joint import linearize_track
from posim_slam.sonar_factor import point_return_factor
from posim_slam.sonar_plane_joint import marginal_plane_information

with np.load(input_root / 'joint-poses.npz') as data:
    stamps, positions, rotations = data['stamps_ns'], data['positions'], data['rotations']
    deltas, covariance = data['deltas'], data['covariance']
with np.load(input_root / 'beam-response.npz') as data:
    ranges, angles = data['ranges'], data['angles']
cal = json.loads((input_root / 'calibration.json').read_text())['sonar']
geometry = dict(imu_to_sonar=np.array(cal['imu_to_sonar']), sonar_to_imu=np.array(cal['sonar_to_imu']),
    range_min=.4, range_max=float(ranges[-1]), horizontal_fov=2*np.pi/3, vertical_fov=np.pi/9)
states = {int(stamp): dict(position=positions[i], attitude=rotations[i]) for i, stamp in enumerate(stamps)}
sigma = np.array([.025083611719310283, .008213314126139592])
def major(value):
    return float(3*np.sqrt(np.linalg.eigvalsh(value['plane_covariance'])[-1])) if value['full_rank'] else np.inf

summary = {}
checked_jacobian = False
for name in ('chronological', 'blocked'):
    started = time.perf_counter()
    with np.load(base / ('heldout-' + name + '.npz')) as data:
        points, training = data['points'], data['train']
        positive, pixels, selected = data['positive'], data['pixels'], data['selected']
        fixed_all_sigma = data['optimistic_three_sigma_m']
    indices = np.flatnonzero(selected)
    fixed_sigma, joint_sigma, independent_sigma = [], [], []
    for index in indices:
        support = np.flatnonzero(training & positive[index])
        if len(support) > 16:
            support = support[np.linspace(0, len(support)-1, 16).round().astype(int)]
        observations = [dict(stamp_ns=int(stamps[i]), range_m=float(ranges[pixels[index,i] // len(angles)]),
            azimuth_rad=float(angles[pixels[index,i] % len(angles)])) for i in support]
        block = linearize_track(observations=observations, states=states, landmark=points[index],
            measurement_covariance=np.diag(np.tile(sigma**2, len(support))), **geometry)
        j = block['jacobian'].copy()
        for k, i in enumerate(support):
            j[:, 6*k+3:6*k+6] = j[:, 6*k+3:6*k+6] @ rotations[i].T @ left_jacobian_so3(deltas[i,3:6])
        if not checked_jacobian:
            k, i = 0, support[0]
            pose = states[int(stamps[i])]
            reference = exp_so3(-deltas[i,3:6]) @ rotations[i]
            finite = []
            for axis in range(6):
                predictions = []
                for direction in (1., -1.):
                    delta = np.eye(6)[axis] * (direction * 1e-6)
                    factor = point_return_factor(position=pose['position']+delta[:3],
                        attitude=exp_so3(deltas[i,3:6]+delta[3:6]) @ reference,
                        landmark=points[index], measured_range=observations[k]['range_m'],
                        measured_azimuth=observations[k]['azimuth_rad'], **geometry)
                    predictions.append(factor['prediction'])
                finite.append((predictions[0]-predictions[1])/(2e-6)/sigma)
            assert np.allclose(j[:2,:6], np.array(finite).T, atol=2e-5, rtol=0)
            checked_jacobian = True
        columns = [6*i+d for i in support for d in range(6)]
        p = covariance[np.ix_(columns, columns)]
        independent = np.zeros_like(p)
        for k in range(len(support)):
            independent[6*k:6*k+6,6*k:6*k+6] = p[6*k:6*k+6,6*k:6*k+6]
        fixed = major(marginal_plane_information(j, np.zeros_like(p)))
        joint = major(marginal_plane_information(j, p))
        separate = major(marginal_plane_information(j, independent))
        assert joint >= fixed-1e-9
        fixed_sigma.append(fixed)
        joint_sigma.append(joint)
        independent_sigma.append(separate)
    fixed_sigma, joint_sigma, independent_sigma = map(np.array, (fixed_sigma, joint_sigma, independent_sigma))
    # A diagnostic independent-source subset: reject an entire candidate before
    # any map admission if it would reuse a claimed original timestamp/pixel.
    # A pixel may physically contain many reflectors; unselected branches remain
    # unresolved. This is not a statement that the raw image is single-scatterer.
    claimed = set()
    exclusive = []
    for local in np.lexsort((indices, fixed_sigma)):
        index = int(indices[local])
        keys = [(int(stamps[i]), int(pixels[index,i])) for i in np.flatnonzero(training & positive[index])]
        if any(key in claimed for key in keys):
            continue
        claimed.update(keys)
        exclusive.append(index)
    union = set()
    assignments = 0
    for index in indices:
        keys = [(int(stamps[i]), int(pixels[index,i])) for i in np.flatnonzero(training & positive[index])]
        union.update(keys)
        assignments += len(keys)
    np.savez_compressed(base / ('heldout-joint-' + name + '.npz'), indices=indices,
        fixed_pose_three_sigma_m=fixed_sigma, correlated_pose_three_sigma_m=joint_sigma,
        independent_pose_three_sigma_m=independent_sigma, exclusive_indices=np.array(exclusive, dtype=int))
    summary[name] = dict(candidates=len(indices), all_training_supported_candidates_evaluated=True,
        fixed_pose_three_sigma_m_min_p50_p90=np.percentile(fixed_sigma,[0,50,90]).tolist(),
        correlated_pose_three_sigma_m_min_p50_p90=np.percentile(joint_sigma,[0,50,90]).tolist(),
        independent_pose_three_sigma_m_min_p50_p90=np.percentile(independent_sigma,[0,50,90]).tolist(),
        fixed_pose_within_010_m=int((fixed_sigma<=.1).sum()), correlated_pose_within_010_m=int((joint_sigma<=.1).sum()),
        exclusive_candidates=len(exclusive), training_assignments=assignments, distinct_training_pixel_keys=len(union),
        shared_assignments=assignments-len(union), seconds=time.perf_counter()-started)
report = dict(folds=summary, native_coordinates_jacobian_finite_difference_passed=checked_jacobian,
    maximum_likelihood_frames=16, frame_selection='equispaced positive training support; other frames supply support only',
    covariance_method='existing public marginal_plane_information Gaussian three-parameter operation applied to point Jacobians; no plane constraints',
    noise='unchanged diagonal diagnostic scales; no branch/association/response-calibration uncertainty',
    map_qualified=False, scope='same-snapshot correlated pose prior excludes all sonar factors; conditional-on-association local diagnostic only')
(base / 'heldout-joint-summary.json').write_text(json.dumps(report, indent=2) + '\n')
print(json.dumps(report, indent=2))
