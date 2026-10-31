"""Bounded source-local moment control; supplied geometry is not a map.

Reuse the existing analytic selected-pixel and full-FFT implementations. This
benchmark introduces no new Fourier approximation and does not fit an image.
"""
import argparse
import json
from pathlib import Path
import time
import numpy as np
from posim_slam.sonar_range_response import PosimRangeResponse
from posim_slam.sonar_speckle import surface_pixel_moments
from posim_slam.sonar_source_surface import SourceSurfaceObservation
from posim_slam.sonar_surface_power import SurfacePowerModel, TriangleSurface


def surface():
    angle = np.deg2rad(35)
    n = np.array([-np.sin(angle), 0., np.cos(angle)])
    a, b = np.array([np.cos(angle), 0., np.sin(angle)]), np.array([0., 1., 0.])
    corners = -1.5*n + np.array([-30*a-30*b, 30*a-30*b, 30*a+30*b, -30*a+30*b])
    triangles = [corners[[0, 1, 2]], corners[[0, 2, 3]]]
    lo, hi = np.array([3., -.7, -.1]), np.array([4., .7, .45])
    for axis in range(3):
        other = [x for x in range(3) if x != axis]
        for value in (lo[axis], hi[axis]):
            c = np.tile(lo, (4, 1)); c[:, axis] = value
            c[:, other[0]] = [lo[other[0]], hi[other[0]], hi[other[0]], lo[other[0]]]
            c[:, other[1]] = [lo[other[1]], lo[other[1]], hi[other[1]], hi[other[1]]]
            triangles.extend((c[[0, 1, 2]], c[[0, 2, 3]]))
    return TriangleSurface(np.array(triangles))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    # Exact published 15 m development profile: not commercial calibration.
    bandwidth, speed, bins = 29900., 1500., 599
    az, el = np.linspace(-np.pi/3, np.pi/3, 256), np.linspace(-np.pi/18, np.pi/18, 256)
    model = SurfacePowerModel(az, el, np.arange(bins)*speed/(2*bandwidth),
                               maximum_range_m=15.)
    geometry = surface()
    start = time.perf_counter()
    source = SourceSurfaceObservation(model, geometry)
    setup_seconds = time.perf_counter()-start
    ideal = model.observation(geometry, np.zeros(3), np.eye(3))
    mu = np.full(len(geometry.triangles), .001)
    depths = source.ranges.T.astype(float)
    variance = np.zeros_like(depths)
    keep = ((source.hit_triangles >= 0) & (source.ranges > 0) & (source.ranges <= 15.))
    grid = np.zeros_like(source.ranges, dtype=float)
    grid[keep] = source.unit_power*mu[source.triangle_ids]/(bins*(bandwidth/bins)**2)
    variance[:] = grid.T
    mixing = abs(np.sinc(.884/(az[1]-az[0])*np.sin(az[:, None]-az)))
    mixing /= np.linalg.norm(mixing)
    fft = PosimRangeResponse(bins, bandwidth, speed)
    reports, arrays = [], {}
    for name, rr, bb in (
            ('pixels16', [100, 120, 160, 200], [64, 106, 150, 192]),
            ('pixels64', [100, 110, 120, 140, 160, 180, 220, 280], [16, 48, 80, 112, 144, 176, 208, 240])):
        pixels = np.array([(r, b) for r in rr for b in bb])
        start = time.perf_counter()
        actual = source.moments(mu, pixels)
        analytic_seconds = time.perf_counter()-start
        start = time.perf_counter()
        expected = fft.power_moments(depths, variance, mixing, pixels[:, ::-1], range_max_m=15.)
        fft_seconds = time.perf_counter()-start
        ideal_result = surface_pixel_moments(ideal, mu, pixels)
        error = {key: float(np.linalg.norm(actual[key]-expected[key])/np.linalg.norm(expected[key]))
                 for key in ('mean', 'covariance')}
        report = dict(name=name, pixels=len(pixels), independent_sources=len(source.selected_ranges),
                      analytic_seconds=analytic_seconds, full_fft_seconds=fft_seconds,
                      relative_l2_error=error,
                      ideal_normal_mean_difference=float(np.linalg.norm(actual['mean']-ideal_result['mean'])
                                                        /np.linalg.norm(actual['mean'])),
                      equivalent_looks_min_max=[float(actual['equivalent_looks'].min()),
                                                float(actual['equivalent_looks'].max())])
        reports.append(report)
        for key, value in actual.items():
            arrays[name+'_'+key] = value
        for key, value in expected.items():
            arrays[name+'_fft_'+key] = value
        arrays[name+'_pixels'] = pixels
    report = dict(profile=dict(beams=256, vertical_rays=256, range_bins=bins,
                               bandwidth_hz=bandwidth, maximum_range_m=15.),
                  observation_setup_seconds=setup_seconds, cases=reports,
                  passed=all(max(r['relative_l2_error'].values()) <= 2e-10 for r in reports),
                  scope='Conditional supplied geometry; one local process timing, not native throughput or a SLAM result')
    np.savez_compressed(args.output.with_suffix('.npz'), **arrays)
    args.output.write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report, indent=2))
    if not report['passed']:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
