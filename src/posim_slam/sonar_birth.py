# SPDX-License-Identifier: Apache-2.0
"""Sensor-only finite reflector proposals for the experimental scene inverse.

Regions of unexplained power retain range/azimuth/elevation-volume support.
Distinct views, not pixels/track age, vote for shared candidate cells. These
are hypotheses to re-evaluate with the existing source scene observation;
neither a bright region nor an overlap cell is a measured XYZ landmark.
"""
from collections import deque
import numpy as np


def _mean_image(observation, reflectivity):
    """Full native mean for proposal discovery, using finite Fourier lags.

    The linear autocorrelation of the *same* periodic Hamming samples gives
    |A(d)|². Sum source phases at positive/negative lags before the original-N
    FFT; fold both signs, not a padded range axis. Source ray normals, ranges,
    reflectivity and beam mixing come from the existing observation/model.
    This private proposal calculation provides no power covariance or geometry
    derivative. Cancellation can leave tiny signed means: retain them and a
    conservative diagnostic arithmetic envelope, never clip original returns
    or interpret this envelope as physical noise/calibrated confidence.
    Exact selected-pixel moments remain the inverse validation operator.
    """
    model = observation.model
    mu = np.asarray(reflectivity)
    if (mu.shape != (observation.size,) or not np.isfinite(mu).all()
            or np.any(mu < 0)):
        raise ValueError('nonnegative_finite_face_reflectivity_required')
    bins, beams = len(model.native_ranges_m), model.nbeams
    if len(observation.selected_ranges) > 65536:
        raise MemoryError('bounded_birth_phase_workspace')
    q = mu[observation.triangle_ids]*observation.unit_power
    f = np.arange(bins)
    window = .54-.46*np.cos(2*np.pi*(f+1)/bins)
    window /= np.linalg.norm(window)
    lag = np.correlate(window, window, mode='full')[bins-1:]
    period = bins*(model.native_ranges_m[1]-model.native_ranges_m[0])
    distance = observation.selected_ranges/period
    spectrum = np.zeros((bins, beams), dtype=complex)
    total = np.bincount(observation.beam_ids, weights=q, minlength=beams)
    spectrum[0] = lag[0]*total/bins
    for start in range(1, bins, 8):
        indices = np.arange(start, min(start+8, bins))
        phase = np.exp(2j*np.pi*indices[:,None]*distance[None,:])*q
        for index, values in zip(indices, phase):
            summed = (np.bincount(observation.beam_ids, weights=values.real, minlength=beams)
                      + 1j*np.bincount(observation.beam_ids, weights=values.imag, minlength=beams))
            coefficient = lag[index]*summed/bins
            spectrum[index] += coefficient
            spectrum[(-index)%bins] += coefficient.conjugate()
    local = np.fft.fft(spectrum, axis=0).real
    az = model.azimuth_rad
    mixing = abs(np.sinc(.884/(az[1]-az[0])*np.sin(az[:,None]-az)))**2
    mixing /= mixing.sum()
    image = local@mixing.T
    envelope = 512*np.finfo(float).eps*bins*(total@mixing.T)
    return dict(mean=image, arithmetic_envelope=np.broadcast_to(envelope,image.shape).copy(),
                signed_mean_retained=True, confidence_calibrated=False)


def _average(image, radius):
    if radius == 0:
        return image.copy()
    pad = np.pad(image, radius, mode='edge')
    width = 2*radius+1
    # Local positive sums preserve weak distant returns. Subtracting large
    # integral-image prefixes can erase a weak patch next to a strong echo.
    return np.lib.stride_tricks.sliding_window_view(pad,(width,width)).mean(axis=(-1,-2))


def _components(mask):
    labels = np.zeros(mask.shape, dtype=np.int32)
    groups = []
    for a,b in np.argwhere(mask):
        if labels[a,b]:
            continue
        label = len(groups)+1
        queue, rows = deque([(int(a),int(b))]), []
        labels[a,b] = label
        while queue:
            i,j = queue.popleft(); rows.append((i,j))
            for u,v in [(i-1,j),(i+1,j),(i,j-1),(i,j+1)]:
                if (0<=u<mask.shape[0] and 0<=v<mask.shape[1]
                        and mask[u,v] and labels[u,v]==0):
                    labels[u,v] = label;queue.append((u,v))
        groups.append(np.array(rows,dtype=int))
    return labels, groups


def unexplained_regions(observed, prediction, *, radius=2, contrast=4.,
                        minimum_pixels=8, minimum_beams=3):
    """Find candidate excess-power regions without a per-pixel gain/noise floor.

    One common gain matches full-image integrated powers. Box means are applied
    identically to data, prediction and its arithmetic envelope. Correlations
    remain unresolved: contrast/size settings are proposal parameters, not a
    constant-false-alarm guarantee or physical-association acceptance test.
    Every component, including rejected small components, remains in labels.
    """
    power = np.asarray(observed)
    mean, envelope = np.asarray(prediction['mean']), np.asarray(prediction['arithmetic_envelope'])
    if (power.ndim != 2 or power.size>2_000_000 or min(power.shape)<3
            or mean.shape!=power.shape or envelope.shape!=power.shape
            or not all(np.isfinite(x).all() for x in [power,mean,envelope])
            or np.any(power<0) or np.any(envelope<0) or power.sum()<=0 or mean.sum()<=0
            or type(radius) is not int or not 0<=radius<=8
            or not np.isfinite(contrast) or contrast<=1
            or type(minimum_pixels) is not int or not 1<=minimum_pixels<=4096
            or type(minimum_beams) is not int or not 1<=minimum_beams<=64):
        raise ValueError('bounded_power_region_inputs_required')
    gain = float(power.sum()/mean.sum())
    measured, expected = _average(power,radius), _average(mean,radius)
    uncertainty = _average(envelope,radius)
    threshold = gain*np.maximum(expected+uncertainty,0.)
    mask = measured>contrast*threshold
    labels, components = _components(mask)
    regions = []
    for label,pixels in enumerate(components,1):
        beams = len(np.unique(pixels[:,1]))
        score = float(np.sum(measured[tuple(pixels.T)]-contrast*threshold[tuple(pixels.T)]))
        regions.append(dict(label=label,pixels=len(pixels),beams=beams,
                            first=pixels.min(axis=0).tolist(),last=pixels.max(axis=0).tolist(),
                            excess_power=score,selected=bool(len(pixels)>=minimum_pixels and beams>=minimum_beams),
                            physical_identity_verified=False))
    return dict(gain=gain,labels=labels,regions=regions,observed=power.copy(),
                smoothed_observed=measured,smoothed_expected=expected,
                smoothed_arithmetic_envelope=uncertainty,map_accepted=False)


def support_cells(region, *, ranges, azimuth, origin, rotation, vertical_fov,
                  cell_size=.2, samples=65):
    """Voxelize a finite range/azimuth region's whole nominal elevation volume.

    Cells express computational search support at caller-supplied fixed pose,
    not calibrated bounds. Half-cell expansion retains the sampled locus, but
    is not a proof of continuous coverage or pose/measurement uncertainty.
    No ground plane, shadow height, truth or intensity-derived XYZ is supplied.
    """
    pixels = np.asarray(region)
    rr, az, p, r = (np.asarray(v) for v in [ranges,azimuth,origin,rotation])
    if (pixels.ndim!=2 or pixels.shape[1:]!=(2,) or not 1<=len(pixels)<=153344
            or pixels.dtype.kind not in 'iu' or rr.ndim!=1 or az.ndim!=1
            or np.any(pixels<0) or np.any(pixels[:,0]>=len(rr)) or np.any(pixels[:,1]>=len(az))
            or p.shape!=(3,) or r.shape!=(3,3) or not all(np.isfinite(v).all() for v in [rr,az,p,r])
            or not np.allclose(r.T@r,np.eye(3),atol=1e-8,rtol=0)
            or not np.isclose(np.linalg.det(r),1.,atol=1e-8,rtol=0)
            or not np.isfinite([vertical_fov,cell_size]).all() or not 0<vertical_fov<np.pi
            or not .02<=cell_size<=1 or type(samples) is not int or not 3<=samples<=257):
        raise ValueError('bounded_finite_echo_region_support_required')
    # All member pixels contribute. Chunk only the workspace, never select a
    # small collection of convenient peaks or collapse elevation to zero.
    elevation = np.linspace(-vertical_fov/2,vertical_fov/2,samples)
    cells = set()
    for start in range(0,len(pixels),256):
        selected = pixels[start:start+256]
        distance, angle = rr[selected[:,0]], az[selected[:,1]]
        directions = np.stack(np.broadcast_arrays(
            np.cos(elevation[:,None])*np.cos(angle),
            np.cos(elevation[:,None])*np.sin(angle),np.sin(elevation[:,None])),axis=-1)
        world = (distance[None,:,None]*directions).reshape(-1,3)@r.T+p
        cells.update(map(tuple,np.floor(world/cell_size).astype(int)))
        if len(cells)>262144:
            raise MemoryError('echo_support_cell_budget')
    return np.array(sorted(cells),dtype=np.int64).reshape(-1,3)


def shared_support(view_cells, *, minimum_views=2):
    """Count distinct views per cell, retaining single-view/competing support."""
    if (not 1<=len(view_cells)<=32 or type(minimum_views) is not int
            or not 2<=minimum_views<=32):
        raise ValueError('bounded_distinct_view_support_required')
    votes = {}
    for view,regions in enumerate(view_cells):
        for region,cells in enumerate(regions):
            cells = np.asarray(cells)
            if cells.ndim!=2 or cells.shape[1:]!=(3,) or cells.dtype.kind not in 'iu' or len(cells)>262144:
                raise ValueError('integer_region_cells_required')
            for cell in set(map(tuple,cells)):
                votes.setdefault(cell,{}).setdefault(view,[]).append(region)
        if len(votes)>524288:
            raise MemoryError('shared_echo_cell_budget')
    keys = sorted(votes)
    counts = np.array([len(votes[cell]) for cell in keys],dtype=np.int16)
    return dict(cells=np.array(keys,dtype=np.int64).reshape(-1,3),distinct_views=counts,
                selected=counts>=minimum_views,
                memberships=[votes[cell] for cell in keys],
                map_accepted=False,physical_identity_verified=False)


def choose_surface_birth(vertices, faces, reflectivity, views, alternatives):
    """Discrete inverse over caller-proposed finite scenes and original power.

    Each alternative explicitly supplies its complete vertices/faces/materials.
    Include the unchanged scene as alternative zero. All alternatives use the
    *same* original queries/poses and the existing source re-tracing operator.
    Selection uses training score only. A lower score proposes a scene; it
    never admits a physical landmark or injects a native state update. Caller
    must retain alternatives, test unused views and resolve joint uncertainty.
    """
    from .sonar_scene_fit import SceneHeightFit
    if not 0<=len(alternatives)<=32:
        raise ValueError('bounded_scene_birth_alternatives_required')
    scenes = [dict(name='unchanged_scene',vertices=vertices,faces=faces,reflectivity=reflectivity)]
    scenes += list(alternatives)
    results = []
    for scene in scenes:
        fit = SceneHeightFit(scene['vertices'],scene['faces'],scene['reflectivity'],views)
        value = fit.evaluate(fit.vertices[:,2])
        results.append(dict(name=scene['name'],score=value['score'],result=value))
    best = min(range(len(results)),key=lambda i:(results[i]['score'],i))
    return dict(proposed_index=best,alternatives=results,map_accepted=False,
                physical_identity_verified=False,native_update=False)
