# SPDX-License-Identifier: Apache-2.0
"""All-pixel check of the existing native Monte Carlo record, no new execution."""
import argparse
import json
from pathlib import Path
from types import SimpleNamespace
import numpy as np
from scipy.stats import gamma
from full_image_audit import positive_image
from run_birth import ROOT,sha,write


def run(records,save=True):
    if save and (ROOT/'native-full-audit.json').exists():raise FileExistsError('preserve original native control')
    source=records/'sonar-native-power-moments-20261031'
    manifest=json.loads((source/'sha256.json').read_text())
    for name in ['inputs.npz','native-output.npz','protocol.json']:assert sha(source/name)==manifest[name],name
    with np.load(source/'inputs.npz',allow_pickle=False) as z:
        depth,normal,angles,area,alpha=z['depth'],z['normal'],z['angles'],float(z['area']),float(z['alpha'])
        supplied_mixing=z['mixing']
    with np.load(source/'native-output.npz',allow_pickle=False) as z:observed=z['power_sum'].T/512
    model=SimpleNamespace(native_ranges_m=np.arange(599)*1500/(2*29900),nbeams=256,azimuth_rad=angles)
    valid=(depth.T>0)&(depth.T<=15.);rr=depth.T[valid].astype(float)
    beam_ids=np.nonzero(valid)[1]
    obs=SimpleNamespace(model=model,selected_ranges=rr,beam_ids=beam_ids,triangle_ids=np.zeros(len(rr),dtype=int),
        unit_power=1e10*area*(29900.**2/599)*rr**-3*np.exp(-4*alpha*rr)*normal.transpose(1,0,2)[...,2][valid].astype(float)**2)
    expected=positive_image(obs,[.001])
    assert expected.shape==observed.shape and np.all(expected>0)
    ratio=observed/expected
    lower=float(gamma.ppf(.01/(2*ratio.size),a=512)/512)
    upper=float(gamma.isf(.01/(2*ratio.size),a=512)/512)
    failed=(ratio<lower)|(ratio>upper)
    az=angles
    recomputed=abs(np.sinc(.884/(az[1]-az[0])*np.sin(az[:,None]-az)));recomputed/=np.linalg.norm(recomputed)
    worst=np.unravel_index(np.argmax(ratio),ratio.shape)
    result=dict(conditional_native_only=True,new_kernel_or_renderer_execution=False,frames=512,pixels=ratio.size,
        lower_ratio=lower,upper_ratio=upper,failed_pixels=int(failed.sum()),
        minimum_ratio=float(ratio.min()),maximum_ratio=float(ratio.max()),
        maximum_standardized_mean_error=float(np.max(abs(ratio-1))*np.sqrt(512)),
        worst_pixel=list(map(int,worst)),worst_expected=float(expected[worst]),worst_observed=float(observed[worst]),
        worst_observed_fraction_of_image_peak=float(observed[worst]/observed.max()),
        mixing_max_absolute_difference=float(abs(recomputed-supplied_mixing).max()),
        full_image_agreement_passed=not bool(failed.any()),map_accepted=False,
        references={str((source/n).relative_to(records)):sha(source/n) for n in ['inputs.npz','native-output.npz','protocol.json']},
        protocol_sha256=sha(ROOT/'native-full-protocol.json'))
    if save:write(ROOT/'native-full-audit.json',result);np.savez_compressed(ROOT/'native-full-audit.npz',
        expected=expected,mean_ratio=ratio,failed=failed)
    print(json.dumps(result),flush=True);return result,dict(expected=expected,mean_ratio=ratio,failed=failed)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--records',required=True,type=Path)
    run(parser.parse_args().records)
