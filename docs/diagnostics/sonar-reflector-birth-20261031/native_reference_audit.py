# SPDX-License-Identifier: Apache-2.0
"""Public-ABI control with an assessment-only supplied grid, no renderer."""
import argparse
import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace
import numpy as np
from scipy.stats import gamma
from full_image_audit import positive_image
from run_birth import ROOT,sha,write,raw_view


def acquire(records,binary):
    output=ROOT/'native-reference'
    output.mkdir(exist_ok=True)
    if (output/'native-output.npz').exists():raise FileExistsError('preserve original native reference control')
    reference=records/'sonar-height-diversity-fit-20261031/field-forward-audit.npz'
    row=next(r for r in json.loads((ROOT/'inputs.json').read_text())['frames'] if r['index']==88)
    view,power,_=raw_view(records,row);model=view['model']
    with np.load(reference,allow_pickle=False) as z:
        prefix='full_scene_true_pose_frame88_';ranges=z[prefix+'ranges'];ids=z[prefix+'hit_triangles'];normal=z[prefix+'normals']
    keep=(ids>=0)&(ranges>0)&(ranges<=15.);r=ranges[keep].astype(float)
    obs=SimpleNamespace(model=model,beam_ids=np.nonzero(keep)[1],triangle_ids=ids[keep],selected_ranges=r,
        unit_power=model.power_constant*r**-3*np.exp(-4*model.alpha*r)*normal[...,2][keep].astype(float)**2)
    mean=positive_image(obs,np.full(int(ids.max())+1,.001));gain=float(np.mean(power/mean))
    ratio=power/(gain*mean)
    query=np.array(np.unravel_index(np.argsort(ratio.ravel())[-64:][::-1],ratio.shape)).T
    xyz=(ranges[...,None]*model.directions.reshape(*model.ray_shape,3)).astype('f4')
    directions=np.column_stack((np.cos(model.azimuth_rad),np.sin(model.azimuth_rad),np.zeros(model.nbeams)))
    operand=output/'reference-operand.npz';np.savez_compressed(operand,xyz=xyz,beam_directions=directions)
    helper=records/'sonar-native-power-moments-20261031/control.py'
    spec=importlib.util.spec_from_file_location('source_native_control',helper)
    control=importlib.util.module_from_spec(spec);spec.loader.exec_module(control)
    protocol=dict(frame_ids=list(range(60000,60064)),seed=73,queries=query.tolist(),
                  source='ffecee9be9976f68c9e883d6cf3bbd6d40eb7729',operator_min_interval_s=.05)
    write(output/'execution-protocol.json',protocol)
    control.acquire(SimpleNamespace(binary=binary,input=operand,output=output),protocol)
    return verify(records,save=True)


def verify(records,save=False):
    source=ROOT/'native-reference'
    with np.load(source/'inputs.npz',allow_pickle=False) as z:
        depth,normal,angles,area,alpha=z['depth'],z['normal'],z['angles'],float(z['area']),float(z['alpha'])
    with np.load(source/'native-output.npz',allow_pickle=False) as z:observed=z['power_sum'].T/64
    model=SimpleNamespace(native_ranges_m=np.arange(599)*1500/(2*29900),nbeams=256,azimuth_rad=angles)
    mask=(depth.T>0)&(depth.T<=15.);r=depth.T[mask].astype(float)
    obs=SimpleNamespace(model=model,beam_ids=np.nonzero(mask)[1],triangle_ids=np.zeros(len(r),dtype=int),selected_ranges=r,
        unit_power=1e10*area*(29900.**2/599)*r**-3*np.exp(-4*alpha*r)*normal.transpose(1,0,2)[...,2][mask].astype(float)**2)
    expected=positive_image(obs,[.001]);ratio=observed/expected
    lower=float(gamma.ppf(.01/(2*ratio.size),a=64)/64);upper=float(gamma.isf(.01/(2*ratio.size),a=64)/64)
    failed=(ratio<lower)|(ratio>upper)
    result=dict(conditional_native_only=True,reference_grid_is_not_renderer_grid=True,
        new_renderer_or_sdk_mutation=False,frames=64,lower_ratio=lower,upper_ratio=upper,
        failed_pixels=int(failed.sum()),minimum_ratio=float(ratio.min()),maximum_ratio=float(ratio.max()),
        full_image_agreement_passed=not bool(failed.any()),map_accepted=False)
    if save:
        write(ROOT/'native-reference-result.json',result)
        np.savez_compressed(ROOT/'native-reference-ratios.npz',expected=expected,mean_ratio=ratio,failed=failed)
    print(json.dumps(result),flush=True);return result


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--records',required=True,type=Path)
    parser.add_argument('--binary',type=Path);parser.add_argument('--verify',action='store_true');args=parser.parse_args()
    verify(args.records) if args.verify else acquire(args.records,args.binary)
