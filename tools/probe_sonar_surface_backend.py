# SPDX-License-Identifier: Apache-2.0
"""Compare finite-surface mean power with the matching POSIM WGPU C ABI.

Known analytic geometry and normals are supplied to the backend. Reflective and
zero-reflectivity foreground panels test opaque first-hit visibility. This is a
sensor-model experiment, not an unknown-scene reconstruction or hardware test.
"""
import argparse
import hashlib
import json
from pathlib import Path
import time

import numpy as np
import posim_slam.sonar_surface_power as surface_power
from probe_sonar_power_backend import load_wgpu_backend


def rectangle(center, normal, half):
    center, normal = np.asarray(center, float), np.asarray(normal, float)
    normal = normal / np.linalg.norm(normal)
    axis = np.eye(3)[np.argmin(abs(normal))]
    u = np.cross(normal, axis); u /= np.linalg.norm(u)
    v = np.cross(normal, u)
    corners = np.array([center + i * half[0] * u + j * half[1] * v
                        for i, j in [(-1, -1), (1, -1), (1, 1), (-1, 1)]])
    return corners[[[0, 1, 2], [0, 2, 3]]]


def probe(library, output, frames=64):
    if type(frames) is not int or not 16 <= frames <= 128:
        raise ValueError('bounded_surface_probe_frame_count_required')
    output.mkdir(parents=True, exist_ok=False)
    owner, compute, release, pointer = load_wgpu_backend(library)
    started = time.perf_counter()
    n, nb, nr = 399, 256, 256
    az, el = np.linspace(-np.pi / 3, np.pi / 3, nb), np.linspace(-np.pi / 18, np.pi / 18, nr)
    ranges = np.arange(n) * 1500 / (2 * 29900)
    model = surface_power.SurfacePowerModel(az, el, ranges, range_group=4, beam_group=3)
    native_model = surface_power.SurfacePowerModel(az, el, ranges)
    triangles = np.concatenate((rectangle([3.15, -.3, 0], [1., .25, .1], [.7, .32]),
                                rectangle([5.35, 0, 0], [1., 0, 0], [2., 1.2])))
    surface = surface_power.TriangleSurface(triangles)
    observation = model.observation(surface, np.zeros(3), np.eye(3))
    native_observation = native_model.observation(surface, np.zeros(3), np.eye(3))
    valid = observation.hit_triangles >= 0
    cosine = np.zeros((nr, nb))
    cosine[valid] = np.einsum('ij,ij->i', model.directions[valid.ravel()],
                             surface.normals[observation.hit_triangles[valid]])
    depth = np.ascontiguousarray(np.where(valid, observation.ranges, 0).T, dtype=np.float32)
    normals = np.zeros((nb, nr, 3), dtype=np.float32); normals[:, :, 2] = cosine.T
    window = .54 - .46 * np.cos(2 * np.pi * (np.arange(n) + 1) / n)
    window = np.array(window / np.linalg.norm(window), np.float32)
    beam = abs(np.sinc(.884 / (az[1] - az[0]) * np.sin(az[:, None] - az)))
    beamnorm = float(np.linalg.norm(beam)); beam = np.ascontiguousarray(beam, np.float32)
    area = 2 * (az[1] - az[0]) * (el[1] - el[0])
    rows, arrays = {}, {}
    for case, mu in [('reflective_occluder', np.full(4, .001)),
                     ('black_occluder', np.array([0., 0., .001, .001]))]:
        reflectivity = np.zeros((nr, nb), np.float32)
        reflectivity[valid] = mu[observation.hit_triangles[valid]]
        reflectivity = np.ascontiguousarray(reflectivity.T)
        predicted = observation.forward(mu)
        mean, m2 = np.zeros((n, nb)), np.zeros((n, nb))
        totals = []
        for frame in range(frames):
            result = compute(*(a.ctypes.data_as(pointer) for a in
                                (depth, normals, reflectivity, beam, window)),
                             nb, nr, n, 1, 1500., 10., 220., .0354 * np.log(10) / 20,
                             .02, 29900., beamnorm, area, az[-1] - az[0], el[-1] - el[0],
                             frame, 42, False)
            if not result:
                raise RuntimeError('matching_surface_wgpu_output_required')
            try:
                raw = np.ctypeslib.as_array(result, shape=(nb * n * 2,)).copy()
            finally:
                release(result, nb * n * 2)
            power = np.sum(raw.reshape(nb, n, 2).astype(float) ** 2, axis=2).T
            if not np.isfinite(power).all():
                raise ValueError('finite_surface_probe_output_required')
            delta = power - mean; mean += delta / (frame + 1); m2 += delta * (power - mean)
            totals.append(float(power.sum()))
        measured = np.array([[mean[r:r + 4, b:b + 3].mean() for b in range(0, nb, 3)]
                             for r in range(0, n, 4)])
        native_predicted = native_observation.forward(mu)
        expected_total = float(native_predicted.sum())
        rear = model.observation(surface_power.TriangleSurface(triangles[2:]), np.zeros(3), np.eye(3)).forward(np.full(2, .001))
        foreground = model.observation(surface_power.TriangleSurface(triangles[:2]), np.zeros(3), np.eye(3)).forward(mu[:2])
        standard_error = float(np.std(totals, ddof=1) / np.sqrt(frames))
        rows[case] = dict(frames=frames,
                         relative_mean_power_l2=float(np.linalg.norm(predicted - measured) / np.linalg.norm(measured)),
                         unoccluded_control_relative_l2=float(np.linalg.norm(rear + foreground - measured) / np.linalg.norm(measured)),
                         mean_total_power=float(np.mean(totals)), predicted_total_power=expected_total,
                         total_power_ratio=float(np.mean(totals) / expected_total),
                         total_mean_standard_error=standard_error,
                         total_mean_standard_error_units=float((np.mean(totals) - expected_total) / standard_error))
        arrays[case + '_mean'] = mean; arrays[case + '_variance'] = m2 / (frames - 1)
        arrays[case + '_predicted'] = native_predicted
    np.savez_compressed(output / 'power-controls.npz', **arrays, triangles=triangles,
                        azimuth=az, elevation=el, native_ranges=ranges)
    report = dict(scope=__doc__, map_qualified=False, hardware_calibrated=False,
                  library_sha256=hashlib.sha256(library.read_bytes()).hexdigest(),
                  source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                  model_sha256=hashlib.sha256(Path(surface_power.__file__).read_bytes()).hexdigest(),
                  rows=rows, valid_rays=int(valid.sum()),
                  foreground_rays=int(np.count_nonzero(valid & (observation.hit_triangles < 2))),
                  shared_seed_sequence_across_material_controls=True,
                  elapsed_s=time.perf_counter() - started)
    (output / 'report.json').write_text(json.dumps(report, indent=2) + '\n')
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--library', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--frames', type=int, default=64)
    args = parser.parse_args()
    print(json.dumps(probe(args.library, args.output, args.frames)))


if __name__ == '__main__':
    main()
