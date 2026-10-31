"""Ground-only power baseline from recorded sonar and causal DVL plane means.

No truth, assessment mesh, RGB or native map points are read. These already
inspected frames are a development baseline, not independent validation.
"""
import argparse
import gzip
import json
from pathlib import Path
import numpy as np
from posim_slam.sonar_surface_power import SurfacePowerModel, TriangleSurface
from posim_slam.sonar_source_surface import SourceSurfaceObservation
from posim_slam.sonar_power_evidence import profile_power_evidence


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--records', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    source = args.records/'sonar-translated-shadow-20261031'
    provenance = json.loads(gzip.decompress((source/'input-provenance.json.gz').read_bytes()))
    protocol = json.loads((args.output/'protocol.json').read_text())
    pixels = np.asarray(protocol['queries'])
    arrays, results = {}, []
    for index in [48, 96, 144]:
        name = f'sonar-{index:04d}.npz'
        candidates = [f for f in provenance['frames'] if Path(f['file']).name == name]
        assert len(candidates) == 1
        row = candidates[0]
        with np.load(source/'field'/name, allow_pickle=False) as z:
            ranges, db = z['ranges'].copy(), z['intensity_db'].astype(float)
            azimuth = np.arctan2(z['beam_directions'][:, 1], z['beam_directions'][:, 0])
        n, d = np.array(row['normal_sonar']), row['plane_distance_m']
        first = np.cross(n, np.eye(3)[np.argmin(abs(n))]); first /= np.linalg.norm(first)
        second = np.cross(n, first)
        corners = np.array([d*n+100*(a*first+b*second) for a, b in [(-1,-1),(-1,1),(1,1),(1,-1)]])
        geometry = TriangleSurface(corners[[[0,1,2],[0,2,3]]])
        model = SurfacePowerModel(azimuth, np.linspace(-np.pi/18, np.pi/18, 256), ranges,
                                   maximum_range_m=15.)
        observation = SourceSurfaceObservation(model, geometry)
        prediction = observation.moments([.001, .001], pixels)
        power = np.power(10., db[pixels[:, 0], pixels[:, 1]]/10)
        score = profile_power_evidence(power, prediction['mean'], prediction['covariance'])
        results.append(dict(index=index, stamp_ns=row['stamp_ns'],
                            plane_stamp_ns=row['plane_stamp_ns'], gain=score['gain'],
                            normalized_residual_rms=float(np.linalg.norm(score['normalized_residual'])/8),
                            normalized_residual_max=float(np.max(abs(score['normalized_residual']))),
                            field_accuracy_accepted=False, ground_only=True))
        for key, value in [('observed',power),('mean',prediction['mean']),
                            ('covariance',prediction['covariance']),('residual',score['normalized_residual'])]:
            arrays[f'frame{index}_{key}'] = value
        arrays[f'frame{index}_normal'] = n
        arrays[f'frame{index}_distance'] = np.asarray(d)
    np.savez_compressed(args.output/'field-ground-baseline.npz', **arrays)
    report = dict(frames=results, assessment_geometry_used=False, native_fusion=False,
                  uncertainties_excluded=['DVL range/plane','navigation pose','reflectivity heterogeneity'],
                  scope=__doc__)
    (args.output/'field-ground-baseline.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
