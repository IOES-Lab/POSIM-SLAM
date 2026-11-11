# SPDX-License-Identifier: Apache-2.0
"""Post-selection full-image mean check; no geometry fit or reference input."""
import argparse
import json
import time
from pathlib import Path
import numpy as np
from posim_slam.sonar_speckle import periodic_hamming_amplitude
from posim_slam.sonar_source_surface import SourceSurfaceObservation
from posim_slam.sonar_surface_power import TriangleSurface
from run_birth import ROOT,TRAIN,UNUSED,sha,write,raw_view,load_scenes


def positive_image(observation,reflectivity):
    """Reorder the existing mean's positive sums; retain every native pixel."""
    model=observation.model
    q=np.asarray(reflectivity)[observation.triangle_ids]*observation.unit_power
    bins,beams=len(model.native_ranges_m),model.nbeams
    step=model.native_ranges_m[1]-model.native_ranges_m[0]
    radial=np.zeros((bins,beams))
    for beam in range(beams):
        keep=observation.beam_ids==beam
        if not np.any(keep):continue
        delay=observation.selected_ranges[keep][None,:]/step-np.arange(bins)[:,None]
        h=periodic_hamming_amplitude(delay,bins)
        radial[:,beam]=abs(h)**2@q[keep]
    az=model.azimuth_rad
    mixing=abs(np.sinc(.884/(az[1]-az[0])*np.sin(az[:,None]-az)))**2
    mixing/=mixing.sum()
    return radial@mixing.T


def run(records,save=True):
    if save and (ROOT/'full-image-audit.json').exists():raise FileExistsError('preserve original full-image control')
    seal=json.loads((ROOT/'construction-seal.json').read_text())
    for name,digest in seal['files'].items():assert sha(ROOT/name)==digest
    data=json.loads((ROOT/'inputs.json').read_text())
    selected=json.loads((ROOT/'fit-output.json').read_text())['selected_index']
    scenes=load_scenes();report=[];arrays={};started=time.monotonic()
    with np.load(ROOT/'proposal-images.npz',allow_pickle=False) as discovery:
        for frame in data['frames']+data['unused_frames']:
            view,power,_=raw_view(records,frame)
            for index in sorted({0,selected}):
                candidate=scenes[index]
                observation=SourceSurfaceObservation(view['model'],TriangleSurface(
                    (candidate['vertices'][candidate['faces']]-view['origin'])@view['rotation']))
                mean=positive_image(observation,candidate['reflectivity'])
                active=mean>0
                assert not np.any((power>0)&~active)
                gain=float(np.mean(power[active]/mean[active]))
                residual=np.zeros_like(mean);residual[active]=power[active]/(gain*mean[active])-1
                score=float(np.log(mean[active]).sum()+active.sum()*(np.log(gain)+1))
                case=dict(index=frame['index'],scene=index,group='train' if frame['index'] in TRAIN else 'unused',
                    pixels=mean.size,zero_support_pixels=int(sum(~active.ravel())),score=score,gain=gain,
                    residual_rms=float(np.sqrt(np.mean(residual**2))))
                if index==0 and frame['index'] in TRAIN:
                    estimate=discovery[f'frame{frame["index"]}_mean']
                    envelope=discovery[f'frame{frame["index"]}_arithmetic_envelope']
                    error=abs(mean-estimate)
                    assert np.all(error<=envelope)
                    case['all_pixel_discovery_error_envelope_ratio']=float(np.max(error/envelope))
                prefix=f'scene{index}_frame{frame["index"]}_'
                for name,value in [('range_observed_mean',power.mean(1)),('range_predicted_mean',gain*mean.mean(1)),
                    ('range_residual_rms',np.sqrt(np.mean(residual**2,axis=1))),
                    ('beam_observed_mean',power.mean(0)),('beam_predicted_mean',gain*mean.mean(0)),
                    ('beam_residual_rms',np.sqrt(np.mean(residual**2,axis=0)))]:arrays[prefix+name]=value
                report.append(case);print(json.dumps(case),flush=True)
    result=dict(adaptive_post_selection=True,selected_index_unchanged=selected,
        map_accepted=False,native_update=False,truth_scene_rgb_used=False,
        elapsed_s=time.monotonic()-started,results=report,
        references={n:sha(ROOT/n) for n in ['construction-seal.json','full-image-protocol.json','full_image_audit.py']})
    if save:write(ROOT/'full-image-audit.json',result);np.savez_compressed(ROOT/'full-image-profiles.npz',**arrays)
    return result,arrays


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--records',required=True,type=Path)
    run(parser.parse_args().records)
