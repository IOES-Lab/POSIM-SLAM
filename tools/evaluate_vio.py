"""Evaluate IMU-origin estimates against separate body-origin truth.

Initial yaw and translation fix the local gauge. Gravity makes roll/pitch
observable, so their errors must remain visible. No scale or whole-trajectory
fit is allowed. This evaluator is never imported by the native estimator.
"""
import argparse
import json
from pathlib import Path
import numpy as np


def matrix(q):
    q = np.asarray(q, dtype=float)
    if q.shape != (4,) or not np.isfinite(q).all() or abs(np.linalg.norm(q) - 1) > 1e-4:
        raise ValueError('invalid_evaluation_quaternion')
    x, y, z, w = q / np.linalg.norm(q)
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def slerp(a, b, u):
    a, b = np.asarray(a), np.asarray(b)
    matrix(a); matrix(b)
    dot = float(a @ b)
    if dot < 0:
        b, dot = -b, -dot
    if dot > .9995:
        q = (1-u)*a+u*b
        return q / np.linalg.norm(q)
    angle = np.arccos(np.clip(dot, -1, 1))
    return (np.sin((1-u)*angle)*a+np.sin(u*angle)*b) / np.sin(angle)


def read(path):
    with Path(path).open() as stream:columns=stream.readline().strip().split(',')
    value = np.genfromtxt(path, delimiter=',', names=True,
                         dtype=[(n,'i8' if n=='stamp_ns' else 'f8') for n in columns])
    if value.ndim != 1 or len(value) < 2:
        raise ValueError('insufficient_evaluation_rows')
    if not np.isfinite(np.column_stack([value[n] for n in value.dtype.names])).all():
        raise ValueError('invalid_evaluation_rows')
    if np.any(value['stamp_ns']<0) or np.any(np.diff(value['stamp_ns']) <= 0):
        raise ValueError('evaluation_time_reset')
    return value


def angle(r):
    return float(np.arccos(np.clip((np.trace(r)-1)*.5, -1, 1)))


def evaluate(states, truth, mount):
    offset=np.asarray(mount['translation'],dtype=float)
    if offset.shape!=(3,) or not np.isfinite(offset).all():
        raise ValueError('invalid_evaluation_mount')
    times = truth['stamp_ns']
    rotations, positions = [], []
    for state in states:
        t = state['stamp_ns']
        j = int(np.searchsorted(times, t))
        if j == 0 and t == times[0]:
            j = 1
        if j == len(times) or j == 0 or times[j]-times[j-1] > 50_000_000:
            raise ValueError('truth_not_bracketing_or_gap')
        u = float((t-times[j-1])/(times[j]-times[j-1]))
        p = (1-u)*np.array([truth[n][j-1] for n in ('x', 'y', 'z')])
        p += u*np.array([truth[n][j] for n in ('x', 'y', 'z')])
        q = slerp([truth[n][j-1] for n in ('qx', 'qy', 'qz', 'qw')],
                  [truth[n][j] for n in ('qx', 'qy', 'qz', 'qw')], u)
        r = matrix(q)
        positions.append(p+r @ offset)
        rotations.append(r @ matrix(mount['rotation']))
    positions = np.array(positions)
    estimate = np.column_stack([states[n] for n in ('x', 'y', 'z')])
    estimate_r = [matrix([row[n] for n in ('qx', 'qy', 'qz', 'qw')]) for row in states]
    initial = rotations[0] @ estimate_r[0].T
    yaw = np.arctan2(initial[1,0], initial[0,0])
    align_r = np.array([[np.cos(yaw),-np.sin(yaw),0],
                        [np.sin(yaw),np.cos(yaw),0],[0,0,1]])
    align_p = positions[0] - align_r @ estimate[0]
    aligned = estimate @ align_r.T + align_p
    aligned_r = [align_r @ r for r in estimate_r]
    errors = np.linalg.norm(aligned-positions, axis=1)
    angles = np.array([angle(r.T @ e) for r, e in zip(rotations, aligned_r)])
    relative, relative_r = [], []
    for i, t in enumerate(states['stamp_ns']):
        j = int(np.searchsorted(states['stamp_ns'], t+1_000_000_000))
        if j >= len(states) or abs(states['stamp_ns'][j]-t-1_000_000_000) > 100_000_000:
            continue
        true = rotations[i].T @ (positions[j]-positions[i])
        est = aligned_r[i].T @ (aligned[j]-aligned[i])
        relative.append(np.linalg.norm(true-est))
        relative_r.append(angle((rotations[i].T @ rotations[j]).T @
                                (aligned_r[i].T @ aligned_r[j])))
    result = dict(evaluated=True, states=len(states), duration_s=float(
        (states['stamp_ns'][-1]-states['stamp_ns'][0])*1e-9),
        evaluation_alignment=dict(rotation=align_r.tolist(),translation=align_p.tolist(),scale=1.),
        gauge='initial yaw and translation only; gravity tilt retained; scale fixed at 1; no trajectory fit',
        position_rmse_m=float(np.sqrt(np.mean(errors**2))), position_max_m=float(max(errors)),
        rotation_rmse_rad=float(np.sqrt(np.mean(angles**2))),
        rotation_max_rad=float(max(angles)),
        truth_distance_m=float(np.linalg.norm(np.diff(positions, axis=0), axis=1).sum()),
        estimated_distance_m=float(np.linalg.norm(np.diff(aligned, axis=0), axis=1).sum()),
        rpe_1s_pairs=len(relative),
        rpe_1s_position_rmse_m=float(np.sqrt(np.mean(np.square(relative)))) if relative else None,
        rpe_1s_rotation_rmse_rad=float(np.sqrt(np.mean(np.square(relative_r)))) if relative_r else None,
        scope='Recorded trajectory only; does not qualify product accuracy or initialization/recovery')
    return result, np.column_stack([states['stamp_ns'], aligned, positions, errors, angles])


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--states', type=Path, required=True)
    parser.add_argument('--truth', type=Path, required=True)
    parser.add_argument('--recording-receipt', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--trim-boundary-states',action='store_true',
                        help='Explicitly omit at most two edge states lacking truth brackets; never extrapolate')
    args = parser.parse_args()
    if args.output.exists():
        parser.error('Use a new output directory')
    states,truth=read(args.states),read(args.truth)
    total=len(states)
    if args.trim_boundary_states:
        selected=(states['stamp_ns']>=truth['stamp_ns'][0]) & (states['stamp_ns']<=truth['stamp_ns'][-1])
        if total-int(selected.sum())>2 or selected.sum()<5:
            raise ValueError('too_many_unbracketed_boundary_states')
        states=states[selected]
    result, rows = evaluate(states, truth,
                            json.loads(args.recording_receipt.read_text())['imu_mount'])
    result.update(states_total=total,states_without_truth=total-len(states),
                  evaluated_coverage_fraction=len(states)/total)
    args.output.mkdir(parents=True)
    (args.output / 'metrics.json').write_text(json.dumps(result, indent=2)+'\n')
    np.savetxt(args.output / 'aligned.csv', rows, delimiter=',',
               header='stamp_ns,x,y,z,truth_x,truth_y,truth_z,error_m,error_rad', comments='')
    print(json.dumps(result, indent=2))
