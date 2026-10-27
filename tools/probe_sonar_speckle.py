# SPDX-License-Identifier: Apache-2.0
"""Measure source speckle moments/correlation with the actual WGPU library.

Geometry and normals are supplied analytic controls. Pixel selection uses only
the conditional prediction before any backend frames. No simulation navigation,
scene inference, map accuracy or hardware noise calibration is claimed.
"""
import argparse
import hashlib
import json
from pathlib import Path
import time
import numpy as np
import posim_slam.sonar_speckle as speckle_model
import posim_slam.sonar_surface_power as surface_model
from posim_slam.sonar_surface_power import SurfacePowerModel, TriangleSurface
from posim_slam.sonar_speckle import surface_pixel_moments
from probe_sonar_power_backend import load_wgpu_backend
from probe_sonar_surface_backend import rectangle


def probe(library, output, frames=128, *, seed=73, first_frame=20000):
    if type(frames) is not int or not 64 <= frames <= 256:
        raise ValueError('bounded_independent_speckle_frames_required')
    if (type(seed) is not int or not 0 <= seed < 2**64 or type(first_frame) is not int
            or not 0 <= first_frame <= 2**64-frames):
        raise ValueError('uint64_seed_and_frame_interval_required')
    output.mkdir(parents=True, exist_ok=False)
    started=time.perf_counter();owner,compute,release,pointer=load_wgpu_backend(library)
    n,nb,nr=399,256,256
    az=np.linspace(-np.pi/3,np.pi/3,nb);el=np.linspace(-np.pi/18,np.pi/18,nr)
    ranges=np.arange(n)*1500/(2*29900)
    model=SurfacePowerModel(az,el,ranges)
    triangles=np.concatenate((rectangle([3.15,-.3,0],[1.,.25,.1],[.7,.32]),
                              rectangle([5.35,0,0],[1.,0,0],[2.,1.2])))
    observation=model.observation(TriangleSurface(triangles),np.zeros(3),np.eye(3))
    mu=np.full(4,.001);prediction=observation.forward(mu)
    rb,bb=np.unravel_index(prediction.argmax(),prediction.shape);rb=rb//4*4;bb=bb//3*3
    far=prediction.copy();far[ranges<4.8]=0
    fr,fb=np.unravel_index(far.argmax(),far.shape);fr=fr//4*4;fb=fb//3*3
    starts=[(rb,bb),(rb,min(bb+3,nb-3)),(min(rb+4,n-4),bb),(fr,fb)]
    groups=[[(r,b) for r in range(r0,min(r0+4,n)) for b in range(b0,min(b0+3,nb))] for r0,b0 in starts]
    pixels=np.array(sorted(set(pixel for group in groups for pixel in group)),dtype=int)
    weights=np.zeros((len(groups),len(pixels)))
    lookup={tuple(pixel):i for i,pixel in enumerate(pixels)}
    for row,group in enumerate(groups):
        for pixel in group:weights[row,lookup[pixel]]=1/len(group)
    moments=surface_pixel_moments(observation,mu,pixels,weights)
    valid=observation.hit_triangles>=0
    cosine=np.zeros((nr,nb));cosine[valid]=np.einsum('ij,ij->i',model.directions[valid.ravel()],observation.surface.normals[observation.hit_triangles[valid]])
    depth=np.ascontiguousarray(np.where(valid,observation.ranges,0).T,dtype=np.float32)
    normals=np.zeros((nb,nr,3),np.float32);normals[:,:,2]=cosine.T
    reflectivity=np.ascontiguousarray(np.where(valid,.001,0).T,dtype=np.float32)
    window=.54-.46*np.cos(2*np.pi*(np.arange(n)+1)/n);window=np.array(window/np.linalg.norm(window),np.float32)
    beam=abs(np.sinc(.884/(az[1]-az[0])*np.sin(az[:,None]-az)));beamnorm=float(np.linalg.norm(beam));beam=np.ascontiguousarray(beam,np.float32)
    area=2*(az[1]-az[0])*(el[1]-el[0]);samples=[];native_samples=[]
    for frame in range(frames):
        result=compute(*(a.ctypes.data_as(pointer) for a in (depth,normals,reflectivity,beam,window)),
                       nb,nr,n,1,1500.,10.,220.,.0354*np.log(10)/20,.02,29900.,beamnorm,area,
                       az[-1]-az[0],el[-1]-el[0],first_frame+frame,seed,False)
        if not result:raise RuntimeError('actual_speckle_backend_output_required')
        try:raw=np.ctypeslib.as_array(result,shape=(nb*n*2,)).copy()
        finally:release(result,nb*n*2)
        power=np.sum(raw.reshape(nb,n,2).astype(float)**2,axis=2).T
        chosen=power[pixels[:,0],pixels[:,1]]
        if not np.isfinite(chosen).all():raise ValueError('finite_native_speckle_required')
        native_samples.append(chosen);samples.append(weights@chosen)
    samples=np.array(samples);native_samples=np.array(native_samples)
    covariance=np.cov(samples.T);mean=samples.mean(0);variance=np.diag(moments['covariance'])
    k=moments['complex_covariance'];fourth=[]
    for w in weights:
        take=w>0;root=np.sqrt(w[take]);b=k[np.ix_(take,take)]*root[:,None]*root[None,:]
        eigenvalues=np.linalg.eigvalsh(b);fourth.append(float(np.sum(eigenvalues**4)))
    variance_se=np.sqrt(2*variance**2/(frames-1)+6*np.array(fourth)/frames)
    def correlation(c):
        sd=np.sqrt(np.diag(c));return c/np.outer(sd,sd)
    report=dict(scope=__doc__,map_qualified=False,hardware_calibrated=False,frames=frames,
                frame_interval=[first_frame,first_frame+frames-1],seed=seed,pixel_groups=[[int(v) for v in s] for s in starts],
                predicted_mean=moments['mean'].tolist(),measured_mean=mean.tolist(),
                mean_standard_error_units=((mean-moments['mean'])/np.sqrt(variance/frames)).tolist(),
                predicted_variance=variance.tolist(),measured_variance=np.diag(covariance).tolist(),
                variance_standard_error_units=((np.diag(covariance)-variance)/variance_se).tolist(),
                predicted_equivalent_looks=moments['equivalent_looks'].tolist(),
                empirical_equivalent_looks=(mean**2/np.diag(covariance)).tolist(),
                predicted_correlation=correlation(moments['covariance']).tolist(),
                empirical_correlation=correlation(covariance).tolist(),
                naive_independent_12_pixel_variance=(moments['mean']**2/12).tolist(),
                library_sha256=hashlib.sha256(library.read_bytes()).hexdigest(),
                source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                moment_source_sha256=hashlib.sha256(Path(speckle_model.__file__).read_bytes()).hexdigest(),
                surface_source_sha256=hashlib.sha256(Path(surface_model.__file__).read_bytes()).hexdigest(),
                numpy_version=np.__version__,
                elapsed_s=time.perf_counter()-started)
    np.savez_compressed(output/'speckle-controls.npz',samples=samples,native_samples=native_samples,
                        pixels=pixels,weights=weights,mean=moments['mean'],covariance=moments['covariance'],
                        triangles=triangles,azimuth=az,elevation=el,native_ranges=ranges)
    (output/'report.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    return report


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--library',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True);parser.add_argument('--frames',type=int,default=128)
    parser.add_argument('--seed',type=int,default=73);parser.add_argument('--first-frame',type=int,default=20000)
    args=parser.parse_args();print(json.dumps(probe(args.library,args.output,args.frames,seed=args.seed,first_frame=args.first_frame),indent=2))


if __name__=='__main__':main()
