"""Audit a sparse archive against recorded pixels and fixed estimated poses.

No map is filtered or reconstructed. The information matrix conditions on the
historical published poses and independent pixel noise. It is not the joint VIO
posterior: pose, calibration and observation correlations remain unmodelled.
"""
import argparse
import hashlib
import inspect
import json
from pathlib import Path
import sys

import numpy as np

try:
    from posim_slam.map_snapshot import validate_snapshot
except ModuleNotFoundError as error:
    if error.name != 'posim_slam':
        raise
    sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
    from posim_slam.map_snapshot import validate_snapshot
from evaluate_vio import matrix, read


def track_information(point, origins, rotations, pixels, intrinsics, pixel_sigma):
    """Pinhole projection Jacobians in the map frame; no pose interpolation."""
    point, origins, rotations, pixels, intrinsics = (
        np.asarray(value, dtype=float)
        for value in (point, origins, rotations, pixels, intrinsics))
    n = len(origins)
    if (point.shape != (3,) or origins.shape != (n, 3)
            or rotations.shape != (n, 3, 3) or pixels.shape != (n, 2)
            or intrinsics.shape != (4,) or not np.all(intrinsics[:2] > 0)
            or not all(np.isfinite(v).all() for v in (point, origins, rotations, pixels, intrinsics))
            or not np.isfinite(pixel_sigma) or pixel_sigma <= 0):
        raise ValueError('invalid_track_information_input')
    if n and (not np.allclose(rotations.transpose(0, 2, 1) @ rotations, np.eye(3), atol=1e-8, rtol=0)
              or not np.allclose(np.linalg.det(rotations), 1., atol=1e-8, rtol=0)):
        raise ValueError('invalid_camera_rotation')
    result = dict(exposures=n, baseline_m=0., behind_camera=0,
                  sigma_position_m=None, maximum_reprojection_px=None,
                  median_reprojection_px=None, rank=0)
    if not n:
        return result
    # Bound temporary storage even for a long observed track.
    result['baseline_m'] = max(float(np.linalg.norm(
        origins[start:start+64, None] - origins[None], axis=2).max())
        for start in range(0, n, 64))
    local = np.einsum('nji,nj->ni', rotations, point - origins)
    result['behind_camera'] = int((local[:, 2] <= 0).sum())
    if result['behind_camera']:
        return result
    x, y, z = local.T
    fx, fy, cx, cy = intrinsics
    predicted = np.column_stack((fx*x/z + cx, fy*y/z + cy))
    error = np.linalg.norm(predicted - pixels, axis=1)
    result.update(maximum_reprojection_px=float(error.max()),
                  median_reprojection_px=float(np.median(error)))
    jacobian = np.zeros((n, 2, 3))
    jacobian[:, 0, 0], jacobian[:, 1, 1] = fx/z, fy/z
    jacobian[:, 0, 2], jacobian[:, 1, 2] = -fx*x/z**2, -fy*y/z**2
    whitened = np.einsum('nij,nkj->nik', jacobian, rotations).reshape(-1, 3) / pixel_sigma
    singular = np.linalg.svd(whitened, compute_uv=False)
    if singular[0] > 0:
        result['rank'] = int(np.sum(singular > 1e-10*singular[0]))
    if len(singular) == 3 and result['rank'] == 3:
        # Largest principal standard deviation of (J^T J)^-1. SVD avoids
        # squaring the small-parallax condition number.
        result['sigma_position_m'] = float(1/singular[-1])
    return result


def audit(args):
    archive = json.loads(args.archive.read_text())
    if archive.get('schema') != 'posim-visual-archive-trace-1':
        raise ValueError('visual_archive_trace_required')
    ids, points = archive['ids'], np.asarray(archive['points'], dtype='<f4')
    if (len(ids) != len(set(ids)) or any(type(i) is not int or i < 0 for i in ids)
            or points.shape != (len(ids), 3)):
        raise ValueError('invalid_archive_identity')
    raw = args.map.read_bytes()
    metadata = validate_snapshot(raw, dict(frame_id=args.frame,
        scope='accumulated_local_vio_landmarks', request_id=archive['request_id'],
        stamp_ns=archive['stamp_ns']))
    measured = np.frombuffer(raw.split(b'end_header\n', 1)[1], dtype='<f4').reshape(-1, 3)
    if metadata['points'] != len(ids) or not np.array_equal(measured, points):
        raise ValueError('archive_export_points_differ')
    cal = json.loads(args.calibration.read_text())
    poses = read(args.poses)
    origins, rotations = {}, {}
    for pose in poses:
        t = int(pose['stamp_ns'])
        if t > archive['stamp_ns']:
            continue
        rotation = matrix([pose[k] for k in ('qx', 'qy', 'qz', 'qw')])
        origins[t] = np.array([pose[k] for k in ('x', 'y', 'z')]) + rotation @ cal['T_i_c_translation_m']
        rotations[t] = rotation @ matrix(cal['T_i_c_q_xyzw'])
    observations = {i: [] for i in ids}
    shape, last, frames, missing = None, 0, 0, 0
    with args.trace.open() as stream:
        for line in stream:
            row = json.loads(line)
            t = row['stamp_ns']
            if type(t) is not int or t <= last:
                raise ValueError('visual_trace_clock')
            last = t
            if t > archive['stamp_ns']:
                break
            frames += 1
            if frames > 2048:
                raise ValueError('visual_trace_frame_capacity')
            current = row['width'], row['height']
            if (any(type(v) is not int or v <= 0 for v in current)
                    or shape is not None and shape != current):
                raise ValueError('visual_trace_shape')
            shape = current
            seen = set()
            for track in row['tracks']:
                i = track['id']
                if type(i) is not int or i < 0 or i in seen or type(track['full_weight']) is not bool:
                    raise ValueError('visual_trace_track_identity')
                seen.add(i)
                if i not in observations or not track['full_weight']:
                    continue
                if t not in origins:
                    missing += 1
                else:
                    observations[i].append((t, track['pixel']))
    if shape is None:
        raise ValueError('visual_trace_empty')
    sx, sy = shape[0]/cal['width'], shape[1]/cal['height']
    intrinsics = [cal['fx']*sx, cal['fy']*sy, (cal['cx']+.5)*sx-.5, (cal['cy']+.5)*sy-.5]
    rows = []
    for index, (i, point) in enumerate(zip(ids, points)):
        values = observations[i]
        times = [t for t, pixel in values]
        result = track_information(point,
            np.asarray([origins[t] for t in times]).reshape(-1, 3),
            np.asarray([rotations[t] for t in times]).reshape(-1, 3, 3),
            np.asarray([pixel for t, pixel in values]).reshape(-1, 2), intrinsics, args.pixel_sigma)
        rows.append(dict(id=i, export_vertex_index=index,
                         span_s=(max(times)-min(times))/1e9 if times else 0., **result))
    return dict(schema='posim-visual-archive-audit-1', scope=__doc__.strip(),
        frame=args.frame, stamp_ns=archive['stamp_ns'], points=len(rows),
        pixel_sigma_px=args.pixel_sigma, pose_uncertainty_included=False,
        inputs_are_historical_published_poses=True, missing_pose_observations=missing,
        trace_frames_through_export=frames, map_filtered=False, map_qualified=False,
        rows=rows, source_sha256={name: hashlib.sha256(path.read_bytes()).hexdigest()
            for name, path in vars(args).items() if isinstance(path, Path) and name != 'output'},
        implementation_sha256={path.name: hashlib.sha256(path.read_bytes()).hexdigest()
            for path in (Path(__file__), Path(inspect.getfile(matrix)),
                         Path(inspect.getfile(validate_snapshot)))})


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('map', 'archive', 'calibration', 'trace', 'poses', 'output'):
        parser.add_argument('--' + name, type=Path, required=True)
    parser.add_argument('--frame', required=True)
    parser.add_argument('--pixel-sigma', type=float, required=True,
                        help='Explicit pixel noise assumption, not hardware calibration')
    args = parser.parse_args()
    if args.output.exists():
        parser.error('Use a new report path')
    report = audit(args)
    args.output.write_text(json.dumps(report, indent=2, allow_nan=False) + '\n')
    print(json.dumps({k: report[k] for k in ('points', 'missing_pose_observations', 'map_filtered', 'map_qualified')}))


if __name__ == '__main__':
    main()
