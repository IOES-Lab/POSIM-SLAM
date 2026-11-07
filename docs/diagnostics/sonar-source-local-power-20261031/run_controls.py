"""Compare a NumPy adaptation with the unchanged selected POSIM C++ helper.

This is a mathematical range-grid control, without a Gazebo acquisition or a
navigation estimate. Supply the POSIM source and an existing OpenCV SDK; no
network, installed-source mutation or SDK copy is required.
"""
import argparse
import hashlib
import json
from pathlib import Path
import struct
import subprocess
import time
import numpy as np
from posim_slam.sonar_source_normals import ray_surface_normals


def cases():
    def grid(rows, cols):
        return np.linspace(-.8, .8, cols).astype('f4'), np.linspace(-.2, .2, rows).astype('f4')

    def rays(az, el):
        a, e = az.astype(float), el.astype(float)
        return np.stack(np.broadcast_arrays(np.cos(e[:, None])*np.cos(a),
                        np.cos(e[:, None])*np.sin(a), np.sin(e[:, None])), axis=-1)

    def emit(name, az, el, ranges):
        return name, az, el, np.asarray(ranges, dtype='f4')

    az, el = grid(17, 19)
    yield emit('front_plane_17x19', az, el, 3/rays(az, el)[..., 0])
    az, el = grid(23, 29)
    normal = np.array([1., .3, .5]); normal /= np.linalg.norm(normal)
    yield emit('slanted_plane_23x29', az, el, 3/(rays(az, el)@normal))
    az, el = grid(13, 17)
    yield emit('spherical_range_13x17', az, el, np.full((len(el), len(az)), 3.2))
    az = np.linspace(-np.pi/3, np.pi/3, 256).astype('f4')
    el = np.linspace(-np.pi/18, np.pi/18, 256).astype('f4')
    direction = rays(az, el)
    ground = np.array([-np.sin(np.deg2rad(35)), 0., np.cos(np.deg2rad(35))])
    den = direction@ground
    distance = np.divide(-1.5, den, out=np.full_like(den, np.inf), where=den < 0)
    # Independent slab intersections: a finite opaque cuboid in sensor frame.
    lo, hi = np.array([3., -.7, -.1]), np.array([4., .7, .45])
    near, far = lo/direction, hi/direction
    enter, leave = np.minimum(near, far).max(axis=-1), np.maximum(near, far).min(axis=-1)
    box = np.where((enter > 0) & (enter <= leave), enter, np.inf)
    distance = np.minimum(distance, box)
    distance[(distance > 15) | (distance <= 0)] = 0
    yield emit('finite_occluder_256x256', az, el, distance)
    az, el = grid(15, 17)
    distance = np.zeros((len(el), len(az)))
    distance[4:10, 3:8] = (3/rays(az, el)[..., 0])[4:10, 3:8]
    distance[12, 14] = 2.
    distance[0, :4] = [np.nan, np.inf, -1, 16]
    yield emit('missing_isolated_15x17', az, el, distance)
    rng = np.random.default_rng(20261031)
    az = np.sort(rng.uniform(-.8, .8, 19)).astype('f4')
    el = np.sort(rng.uniform(-.2, .2, 13)).astype('f4')
    yield emit('nonuniform_13x19', az, el, 3/rays(az, el)[..., 0])
    for name, rows, cols in (('two_by_two', 2, 2), ('single_row', 1, 7), ('single_column', 7, 1)):
        az, el = grid(rows, cols)
        yield emit(name, az, el, 3/rays(az, el)[..., 0])
    az, el = grid(7, 9)
    yield emit('empty_returns', az, el, np.zeros((len(el), len(az))))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', required=True, type=Path)
    parser.add_argument('--sdk', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    args.output.mkdir(exist_ok=False)
    header = args.source/'gazebo/posim_gz_multibeam_sonar/multibeam_sonar/RaySurfaceNormals.hh'
    source_hash = hashlib.sha256(header.read_bytes()).hexdigest()
    if source_hash != '121f9cb0d19a129c037f5f48ba64a7a476d48e7ba51944b88787f8e761003ee6':
        raise ValueError('source_header_identity_mismatch')
    here = Path(__file__).resolve().parent
    probe = args.output/'normal_reference'
    command = ['/usr/bin/c++', '-std=c++17', '-O2', '-I'+str(header.parent),
               '-I'+str(args.sdk/'include/opencv4'), str(here/'normal_reference.cc'),
               str(args.sdk/'lib/libopencv_core.4.13.0.dylib'),
               '-Wl,-rpath,'+str(args.sdk/'lib'), '-o', str(probe)]
    subprocess.run(command, check=True, capture_output=True)
    arrays, reports = {}, []
    for name, az, el, distance in cases():
        input_file, output_file = args.output/(name+'.bin'), args.output/(name+'-normal.bin')
        input_file.write_bytes(struct.pack('<IId', *distance.shape, 15.)
                               + az.astype('<f4').tobytes() + el.astype('<f4').tobytes()
                               + distance.astype('<f4').tobytes())
        before = time.perf_counter()
        run = subprocess.run([str(probe), str(input_file), str(output_file)],
                             check=True, capture_output=True, text=True)
        cpp_seconds = time.perf_counter()-before
        expected = np.fromfile(output_file, dtype='<f4').reshape(*distance.shape, 3)
        before = time.perf_counter()
        actual = ray_surface_normals(distance, az, el, 15.)
        python_seconds = time.perf_counter()-before
        error = float(np.max(abs(actual.astype(float)-expected)))
        same_mask = bool(np.array_equal(np.linalg.norm(actual, axis=-1) > 0,
                                        np.linalg.norm(expected, axis=-1) > 0))
        reports.append(dict(name=name, shape=list(distance.shape), max_absolute_error=error,
                            zero_mask_exact=same_mask, cpp_process_seconds=cpp_seconds,
                            python_seconds=python_seconds, opencv=run.stdout.strip()))
        for suffix, value in (('ranges', distance), ('azimuth', az), ('elevation', el),
                               ('normals', expected)):
            arrays[name+'_'+suffix] = value
    np.savez_compressed(args.output/'native-normals.npz', **arrays)
    report = dict(source_header_sha256=source_hash, cases=reports,
                  passed=all(r['zero_mask_exact'] and r['max_absolute_error'] <= 2e-5 for r in reports),
                  scope='Unchanged native range-normal function only; no native image or geometry estimation claim')
    (args.output/'normal-control-report.json').write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report, indent=2))
    if not report['passed']:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
