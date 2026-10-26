# SPDX-License-Identifier: Apache-2.0
"""Report power scaling and range shape of the selected POSIM WGPU C ABI.

Requires a matching sonar_wgpu_compute/sonar_wgpu_free library, NumPy and SciPy.
The same seed/frame sequence is reused across distances. No simulator, scene,
navigation estimate or hardware calibration enters this isolated return test.
"""
import argparse
import ctypes as ct
import hashlib
import json
from pathlib import Path

import numpy as np
from scipy.stats import gamma


def load_wgpu_backend(path):
    """Configure the matching, selected POSIM C ABI for diagnostic callers."""
    library = ct.CDLL(str(path.resolve()))
    pointer = ct.POINTER(ct.c_float)
    compute = library.sonar_wgpu_compute
    compute.argtypes = ([pointer] * 5 + [ct.c_uint32] * 4 + [ct.c_float] * 10
                        + [ct.c_uint64] * 2 + [ct.c_bool])
    compute.restype = pointer
    release = library.sonar_wgpu_free
    release.argtypes = [pointer, ct.c_size_t]
    release.restype = None
    return library, compute, release, pointer


def probe(path, frames=64):
    if type(frames) is not int or not 16 <= frames <= 128:
        raise ValueError('bounded_independent_frame_count_required')
    library, compute, release, pointer = load_wgpu_backend(path)
    n, beams, rays, bandwidth = 399, 1, 2, 29900.
    frequencies = np.arange(n)
    window = .54 - .46 * np.cos(2 * np.pi * (frequencies + 1) / n)
    window = (window / np.linalg.norm(window)).astype(np.float32)
    beam = np.ones(1, dtype=np.float32)
    normal = np.array([0, 0, 1, 0, 0, 0], dtype=np.float32)
    reflectivity = np.full(rays, .001, dtype=np.float32)
    area, alpha = 2.25e-5, .0354 * np.log(10) / 20
    rows = []
    for distance in (4., 5.35, 7.17):
        depth = np.array([distance, 0], dtype=np.float32)
        phase = np.exp(2j * np.pi * frequencies * distance / (1500 / (2 * bandwidth)) / n)
        reference = abs(np.fft.fft(window * phase)) ** 2
        reference /= reference.sum()
        powers, shape_errors = [], []
        for frame in range(frames):
            arrays = (depth, normal, reflectivity, beam, window)
            output = compute(*(a.ctypes.data_as(pointer) for a in arrays),
                             beams, rays, n, 1, 1500., 10., 220., alpha, .02,
                             bandwidth, 1., area, 2.0943951, .34906585,
                             frame, 42, False)
            if not output:
                raise RuntimeError('matching_wgpu_backend_output_required')
            try:
                raw = np.ctypeslib.as_array(output, shape=(beams * n * 2,)).copy()
            finally:
                release(output, beams * n * 2)
            power = np.sum(raw.reshape(n, 2).astype(float) ** 2, axis=1)
            if not np.isfinite(power).all() or power.sum() <= 0:
                raise ValueError('finite_nonzero_isolated_return_required')
            powers.append(float(power.sum()))
            shape_errors.append(float(np.linalg.norm(power / power.sum() - reference)))
        expected = (1e10 * .001 * area * distance ** -3 * np.exp(-4 * alpha * distance)
                    * n * (bandwidth / n) ** 2)
        rows.append(dict(range_m=distance, mean_integrated_power=float(np.mean(powers)),
                         expected_mean_integrated_power=expected,
                         mean_power_ratio=float(np.mean(powers) / expected),
                         normalized_shape_l2_median=float(np.median(shape_errors)),
                         normalized_shape_l2_max=max(shape_errors)))
    return dict(scope=__doc__, map_qualified=False, hardware_calibrated=False,
                library_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                frames_per_distance=frames, shared_seed_sequence_across_distances=True,
                ideal_gaussian_mean_ratio_interval_99=gamma.ppf([.005, .995], frames,
                                                               scale=1 / frames).tolist(),
                quantization='actual backend fixed-point and floating-point errors retained',
                post_dft_amplitude_scale='bandwidth / native_bins', rows=rows)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--library', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--frames', type=int, default=64)
    args = parser.parse_args()
    if not args.library.is_file() or args.output.exists():
        parser.error('existing matching library and new report path required')
    result = probe(args.library, args.frames)
    args.output.write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(result))


if __name__ == '__main__':
    main()
