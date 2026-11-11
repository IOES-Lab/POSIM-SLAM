"""Freeze earlier inferred scenes; change only source-calibrated sampling.

No scene oracle, renderer pose, new parameter fit or native update is used.
Training/unused frames and the earlier selected false birth stay unchanged.
"""
from pathlib import Path
import hashlib
import json
import argparse
import time
import numpy as np

ROOT = Path(__file__).resolve().parent
from posim_slam.sonar_surface_power import SurfacePowerModel, TriangleSurface
from posim_slam.sonar_source_surface import SourceSurfaceObservation
from posim_slam.sonar_speckle import periodic_hamming_amplitude


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def positive_mean(observation, reflectivity):
    model = observation.model
    q = reflectivity[observation.triangle_ids]*observation.unit_power
    bins, beams = len(model.native_ranges_m), model.nbeams
    step = model.native_ranges_m[1]-model.native_ranges_m[0]
    radial = np.zeros((bins, beams))
    for beam in range(beams):
        keep = observation.beam_ids == beam
        h = periodic_hamming_amplitude(observation.selected_ranges[None, keep]/step-
                                      np.arange(bins)[:, None], bins)
        radial[:, beam] = abs(h)**2@q[keep]
    az = model.azimuth_rad
    mixing = abs(np.sinc(.884/(az[1]-az[0])*np.sin(az[:, None]-az)))**2
    return radial@(mixing/mixing.sum()).T


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    out = parser.parse_args().output.resolve()
    out.mkdir()
    records = ROOT.parents[1]
    previous = records/'sonar-reflector-birth-20261031'
    seal = json.loads((previous/'construction-seal.json').read_text())
    for name, digest in seal['files'].items():
        assert sha(previous/name) == digest
    inputs = json.loads((previous/'inputs.json').read_text())
    selected = json.loads((previous/'fit-output.json').read_text())['selected_index']
    frames = inputs['frames']+inputs['unused_frames']
    assert len(frames) == 8 and selected == 3
    with np.load(previous/'scenes.npz', allow_pickle=False) as z:
        scenes = {i: dict(vertices=z[f'scene{i}_vertices'], faces=z[f'scene{i}_faces'],
                         reflectivity=z[f'scene{i}_reflectivity']) for i in [0, selected]}
    with np.load(ROOT.parent/'calibration.npz', allow_pickle=False) as z:
        sampled = z['sampled_ray_directions']
    protocol = dict(scope=__doc__, frames=[r['index'] for r in frames],
        training=[r['index'] for r in inputs['frames']], unused=[r['index'] for r in inputs['unused_frames']],
        scenes=[0, selected], arms=['ideal', 'sampled'], map_accepted=False,
        geometry_unchanged=True, navigation_unchanged=True, truth_scene_pose_rgb_used=False,
        selected_pixels=False, profiled_parameters='One scalar gain per complete image',
        source_sha256=sha(__file__), references={n:sha(previous/n) for n in
            ['construction-seal.json','inputs.json','fit-output.json','scenes.npz']},
        calibration_sha256=sha(ROOT.parent/'calibration.npz'))
    (out/'protocol.json').write_text(json.dumps(protocol, indent=2)+'\n')
    result, profiles = [], {}
    for row in frames:
        path = records/row['raw_record']/'raw'/row['file']
        assert sha(path) == row['raw_sha256']
        with np.load(path, allow_pickle=False) as z:
            raw = dict(z)
        az = np.arctan2(raw['beam_directions'][:, 1], raw['beam_directions'][:, 0])
        power = 10**(raw['intensity_db'].astype(float)/10)
        for arm, directions in [('ideal', None), ('sampled', sampled)]:
            model = SurfacePowerModel(az, np.linspace(-np.pi/18, np.pi/18, 256),
                raw['ranges'], maximum_range_m=15., sampled_ray_directions=directions)
            for index, scene in scenes.items():
                started = time.monotonic()
                triangles = (scene['vertices'][scene['faces']]-np.asarray(row['origin']))@np.asarray(row['rotation'])
                observation = SourceSurfaceObservation(model, TriangleSurface(triangles))
                mean = positive_mean(observation, scene['reflectivity'])
                assert np.isfinite(mean).all() and np.all(mean > 0)
                gain = float(np.mean(power/mean));residual = power/(gain*mean)-1
                value = dict(frame=row['index'], group='train' if row in inputs['frames'] else 'unused',
                    arm=arm, scene=index, pixels=power.size, gain=gain,
                    score=float(np.log(mean).sum()+mean.size*(np.log(gain)+1)),
                    residual_rms=float(np.sqrt(np.mean(residual**2))),
                    elapsed_s=time.monotonic()-started)
                result.append(value)
                for key, array in [('range_mean',gain*mean.mean(axis=1)),
                    ('beam_mean',gain*mean.mean(axis=0)),('range_residual_rms',np.sqrt(np.mean(residual**2,axis=1)))]:
                    profiles[f'{arm}_scene{index}_frame{row["index"]}_{key}'] = array
                print(json.dumps(value), flush=True)
    np.savez_compressed(out/'complete-image-profiles.npz', **profiles)
    (out/'assessment.json').write_text(json.dumps(dict(protocol=protocol, results=result), indent=2)+'\n')


if __name__ == '__main__':
    main()
