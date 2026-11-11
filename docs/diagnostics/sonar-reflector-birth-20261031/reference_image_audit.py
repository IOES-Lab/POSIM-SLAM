# SPDX-License-Identifier: Apache-2.0
"""Reuse frozen assessment grids only after inference; no new field/oracle fit."""
import argparse
import json
from pathlib import Path
from types import SimpleNamespace
import numpy as np
from full_image_audit import positive_image
from run_birth import ROOT,sha,write,raw_view


def run(records,save=True):
    if save and (ROOT/'reference-image-audit.json').exists():raise FileExistsError('retain original reference control')
    seal=json.loads((ROOT/'construction-seal.json').read_text())
    for name,digest in seal['files'].items():assert sha(ROOT/name)==digest
    reference=records/'sonar-height-diversity-fit-20261031'
    path=reference/'field-forward-audit.npz'
    assert sha(path)==json.loads((reference/'sha256.json').read_text())['field-forward-audit.npz']
    inputs=json.loads((ROOT/'inputs.json').read_text());report=[];arrays={}
    with np.load(path,allow_pickle=False) as source:
        for frame in inputs['frames']:
            if frame['index'] not in [27,81,88]:continue
            view,power,_=raw_view(records,frame);model=view['model']
            for arm in ['full_scene_navigation_pose','full_scene_true_pose']:
                prefix=f'{arm}_frame{frame["index"]}_'
                ranges=source[prefix+'ranges'];ids=source[prefix+'hit_triangles'];normal=source[prefix+'normals']
                keep=(ids>=0)&(ranges>0)&(ranges<=model.maximum_range_m)
                r=ranges[keep].astype(float)
                observation=SimpleNamespace(model=model,beam_ids=np.nonzero(keep)[1],triangle_ids=ids[keep],selected_ranges=r,
                    unit_power=model.power_constant*r**-3*np.exp(-4*model.alpha*r)*normal[...,2][keep].astype(float)**2)
                mu=np.full(int(ids.max())+1,.001)
                mean=positive_image(observation,mu)
                pixels=source[prefix+'pixels']
                np.testing.assert_allclose(mean[tuple(pixels.T)],source[prefix+'mean'],rtol=2e-12,atol=0.)
                active=mean>0;assert not np.any((power>0)&~active)
                gain=float(np.mean(power[active]/mean[active]));residual=np.zeros_like(mean)
                residual[active]=power[active]/(gain*mean[active])-1
                value=float(np.log(mean[active]).sum()+active.sum()*(np.log(gain)+1))
                row=dict(index=frame['index'],arm=arm,gain=gain,score=value,pixels=mean.size,
                         residual_rms=float(np.sqrt(np.mean(residual**2))),residual_max=float(np.max(abs(residual))))
                for name,array in [('range_observed_mean',power.mean(1)),('range_predicted_mean',gain*mean.mean(1)),
                    ('range_residual_rms',np.sqrt(np.mean(residual**2,axis=1))),
                    ('beam_observed_mean',power.mean(0)),('beam_predicted_mean',gain*mean.mean(0)),
                    ('beam_residual_rms',np.sqrt(np.mean(residual**2,axis=0)))]:arrays[prefix+name]=array
                report.append(row);print(json.dumps(row),flush=True)
    result=dict(assessment_only=True,inference_unchanged=True,reference_is_not_renderer_grid=True,
        map_accepted=False,native_update=False,results=report,
        references={str(path.relative_to(records)):sha(path)},
        protocol_sha256=sha(ROOT/'reference-image-protocol.json'))
    if save:write(ROOT/'reference-image-audit.json',result);np.savez_compressed(ROOT/'reference-image-profiles.npz',**arrays)
    return result,arrays


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--records',required=True,type=Path)
    run(parser.parse_args().records)
