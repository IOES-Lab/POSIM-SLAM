# SPDX-License-Identifier: Apache-2.0
"""Invert a declared finite-plane fixture, with frozen poses and known support.

This benchmark estimates foreground plane offset/slopes and two reflectivities.
Foreground YZ support and the background are supplied. It does not discover
object support, estimate navigation or certify an unknown-scene map.
"""
import argparse
import hashlib
import json
from pathlib import Path
import time

import numpy as np
from scipy.optimize import minimize

import posim_slam.sonar_surface_power as surface_model
from posim_slam.sonar_surface_power import SurfacePowerModel, TriangleSurface


FACES = np.array([[0, 1, 2], [0, 2, 3]])


def surface_from_parameters(packet, parameters):
    h, a, b = parameters[:3]
    support = packet['known_yz_support']
    corners = np.column_stack((h + a * support[:, 0] + b * support[:, 1],
                               support + packet['yz_origin']))
    return np.concatenate((corners[FACES], packet['known_background']))


def validate_fixture(packet):
    required = {'observed', 'positions', 'rotations', 'azimuth', 'elevation',
                'native_ranges', 'train', 'test', 'known_yz_support', 'known_background',
                'yz_origin', 'initial_parameters', 'parameter_bounds', 'maximum_range_m'}
    if set(packet) != required:
        raise ValueError('declared_surface_fixture_fields_required')
    if any(not np.isfinite(value).all() for value in packet.values()):
        raise ValueError('finite_surface_fixture_required')
    count = len(packet['positions'])
    train, test = packet['train'], packet['test']
    if (not 4 <= count <= 48 or packet['positions'].shape != (count, 3)
            or packet['rotations'].shape != (count, 3, 3)
            or train.ndim != 1 or test.ndim != 1
            or train.dtype.kind not in 'iu' or test.dtype.kind not in 'iu'
            or min(len(train), len(test)) < 2
            or sorted(np.r_[train, test].tolist()) != list(range(count))
            or packet['known_yz_support'].shape != (4, 2)
            or packet['known_background'].shape != (2, 3, 3)
            or packet['yz_origin'].shape != (2,)
            or packet['initial_parameters'].shape != (5,)
            or packet['parameter_bounds'].shape != (5, 2)
            or packet['maximum_range_m'].shape != ()
            or np.any(packet['parameter_bounds'][:, 0] >= packet['parameter_bounds'][:, 1])
            or np.any(packet['initial_parameters'] < packet['parameter_bounds'][:, 0])
            or np.any(packet['initial_parameters'] > packet['parameter_bounds'][:, 1])
            or np.any(packet['parameter_bounds'][3:, 0] < np.log(1e-15))
            or np.any(packet['parameter_bounds'][3:, 1] > 0)
            or not np.allclose(packet['rotations'].transpose(0, 2, 1) @ packet['rotations'],
                               np.eye(3), atol=1e-8, rtol=0)
            or not np.allclose(np.linalg.det(packet['rotations']), 1., atol=1e-8, rtol=0)):
        raise ValueError('bounded_disjoint_surface_fixture_required')
    model = SurfacePowerModel(packet['azimuth'], packet['elevation'], packet['native_ranges'],
                              maximum_range_m=float(packet['maximum_range_m']),
                              range_group=4, beam_group=3)
    if (packet['observed'].shape != (count, *model.shape)
            or np.any(packet['observed'] < 0)
            or not np.linalg.norm(packet['observed'][train]) > 0
            or not np.linalg.norm(packet['observed'][test]) > 0):
        raise ValueError('nonnegative_surface_fixture_power_required')
    return model


def fit_fixture(packet, *, max_iterations=80):
    model = validate_fixture(packet)
    if type(max_iterations) is not int or not 1 <= max_iterations <= 100:
        raise ValueError('bounded_surface_fit_iterations_required')
    observed, train, test = packet['observed'], packet['train'], packet['test']
    normalizer = float(np.sum(observed[train] ** 2))

    def objective(parameters):
        surface = TriangleSurface(surface_from_parameters(packet, parameters))
        mu = np.repeat(np.exp(parameters[3:]), 2)
        loss, gradient = 0., np.zeros(5)
        for index in train:
            observation = model.observation(surface, packet['positions'][index], packet['rotations'][index])
            residual = observation.forward(mu) - observed[index]
            loss += .5 * np.sum(residual ** 2) / normalizer
            geometry = observation.geometry_adjoint(residual / normalizer, mu)[:2]
            yz = packet['known_yz_support'][FACES]
            gradient[0] += geometry[:, :, 0].sum()
            gradient[1] += np.sum(geometry[:, :, 0] * yz[:, :, 0])
            gradient[2] += np.sum(geometry[:, :, 0] * yz[:, :, 1])
            gradient[3:] += (observation.adjoint(residual / normalizer).reshape(2, 2).sum(axis=1)
                            * np.exp(parameters[3:]))
        return float(loss), gradient

    initial = packet['initial_parameters']
    initial_loss = objective(initial)[0]
    result = minimize(objective, initial, jac=True, method='L-BFGS-B',
                      bounds=packet['parameter_bounds'],
                      options=dict(maxiter=max_iterations, maxls=20, ftol=1e-11, gtol=1e-7))
    surface = TriangleSurface(surface_from_parameters(packet, result.x))
    mu = np.repeat(np.exp(result.x[3:]), 2)
    predicted = np.array([model.observation(surface, packet['positions'][i], packet['rotations'][i]).forward(mu)
                          for i in test])
    report = dict(scope=__doc__, map_qualified=False, navigation_ready=False,
                  optimizer_success=bool(result.success), optimizer_message=str(result.message),
                  iterations=int(result.nit), evaluations=int(result.nfev), initial_loss=initial_loss,
                  final_loss=float(result.fun), gradient=result.jac.tolist(),
                  estimated_parameters=result.x.tolist(),
                  heldout_relative_power_l2=float(np.linalg.norm(predicted - observed[test])
                                                 / np.linalg.norm(observed[test])),
                  assumptions=['known foreground YZ support and background geometry',
                               'fixed exact fixture poses, without pose covariance',
                               'ideal face normals; single opaque first hits',
                               'conditional ensemble-mean least squares',
                               'branch derivatives exclude silhouette/visibility-cut derivatives'])
    return report, predicted, surface.triangles


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--inputs', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    with np.load(args.inputs, allow_pickle=False) as data:
        packet = dict(data)
    args.output.mkdir(parents=True, exist_ok=False)
    started = time.perf_counter()
    report, predicted, triangles = fit_fixture(packet)
    np.savez_compressed(args.output / 'result.npz', predicted=predicted, triangles=triangles)
    report.update(elapsed_s=time.perf_counter() - started,
                  inputs_sha256=hashlib.sha256(args.inputs.read_bytes()).hexdigest(),
                  source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                  model_sha256=hashlib.sha256(Path(surface_model.__file__).read_bytes()).hexdigest(),
                  versions=dict(numpy=np.__version__, scipy=__import__('scipy').__version__))
    (args.output / 'report.json').write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(report))


if __name__ == '__main__':
    main()
