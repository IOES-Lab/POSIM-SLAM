# SPDX-License-Identifier: Apache-2.0
"""Fixed-navigation finite geometry development diagnostic, not SLAM admission.

Construction reads only raw sonar, causal DVL plane/navigation means and the
existing sensor-only angular-shadow hypotheses. Scene, pose truth and RGB are
excluded. The one-prism family is an explicit restrictive shape prior.
"""
import argparse
import gzip
import hashlib
import json
import lzma
import time
from pathlib import Path
import numpy as np
from scipy.cluster.hierarchy import linkage, fcluster
from posim_slam.sonar_surface_power import SurfacePowerModel, TriangleSurface
from posim_slam.sonar_source_surface import SourceSurfaceObservation
from posim_slam.sonar_speckle import periodic_hamming_amplitude
from posim_slam.sonar_power_evidence import profile_power_evidence


ROOT = Path(__file__).resolve().parent
FACES = np.array([[0,1,2],[0,2,3],[4,6,5],[4,7,6],
                  [0,4,5],[0,5,1],[1,5,6],[1,6,2],
                  [2,6,7],[2,7,3],[3,7,4],[3,4,0]])


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def write(path, value):
    path.write_text(json.dumps(value, indent=2, allow_nan=False)+'\n')


def geometry(parameters, seed, prism):
    sx, sy, z = seed['ground']+parameters[:3]
    center = np.array(seed['reference_xy'])
    def height(xy):
        return z+(xy-center)@np.array([sx, sy])
    xy = center+100*np.array([[-1,-1],[-1,1],[1,1],[1,-1]])
    ground = np.column_stack((xy, height(xy)))[[[0,1,2],[0,2,3]]]
    if not prism:
        return ground, np.empty((0,3))
    x, y = np.array(seed['center_xy'])+parameters[3:5]
    lx, ly, h, yaw = parameters[5:9]
    c, s = np.cos(yaw), np.sin(yaw)
    xy = .5*np.array([[-lx,-ly],[lx,-ly],[lx,ly],[-lx,ly]])@np.array([[c,s],[-s,c]])+[x,y]
    bottom = np.column_stack((xy, height(xy)))
    vertices = np.vstack((bottom, bottom+[0,0,h]))
    return np.concatenate((ground, vertices[FACES])), vertices


def make_model(frame):
    az = np.arctan2(frame['beam_directions'][:,1],frame['beam_directions'][:,0])
    return SurfacePowerModel(az,np.linspace(-np.pi/18,np.pi/18,256),
                             frame['ranges'],maximum_range_m=15.)


def mean_basis(observation, pixels):
    """Exact source Fourier power means; covariance is computed at final states.

    Same response as public surface_pixel_moments, evaluated in <=8-row chunks.
    The two columns group existing triangle IDs into ground and prism material.
    No ideal triangle normal, sub-bin interpolation or ray subsampling is used.
    """
    m = observation.model
    az = m.azimuth_rad
    beam = abs(np.sinc(.884/(az[1]-az[0])*np.sin(az[:,None]-az)))
    beam /= np.linalg.norm(beam)
    result = np.zeros((len(pixels),2))
    if not len(observation.selected_ranges):
        return result
    q = np.zeros((len(observation.selected_ranges),2))
    q[np.arange(len(q)),(observation.triangle_ids>=2).astype(int)] = observation.unit_power*.001
    step = m.native_ranges_m[1]-m.native_ranges_m[0]
    for lo in range(0,len(pixels),8):
        selected = pixels[lo:lo+8]
        h = beam[selected[:,1,None],observation.beam_ids]*periodic_hamming_amplitude(
            observation.selected_ranges[None,:]/step-selected[:,0,None],len(m.native_ranges_m))
        result[lo:lo+8] = abs(h)**2@q
    return result


def prepare(records, raw):
    protocol = json.loads((ROOT/'protocol.json').read_text())
    source = records/'sonar-translated-shadow-20261031'
    provenance = json.loads(gzip.decompress((source/'input-provenance.json.gz').read_bytes()))
    shadows = json.loads(lzma.decompress((source/'angular_median-complete.json.xz').read_bytes()))
    indices = protocol['training_indices']+protocol['unused_temporal_indices']
    frames = []
    for index in indices:
        name = f'sonar-{index:04d}.npz'
        row = next(r for r in provenance['frames'] if r['file']==name)
        input_path = raw/name
        assert sha(input_path)==row['raw_sha256']
        frames.append(dict(index=index,**row))
    points = np.array([o['geometry']['point_map'] for f in shadows['frames']
        if f['file'] in [f'sonar-{i:04d}.npz' for i in protocol['training_indices']]
        for c in f['components'] for o in c['observations']
        if o['bright_pair_observed'] and o['recovery_observed'] and o['geometry']['available']])
    labels = fcluster(linkage(points[:,:2],method='single'),.75,criterion='distance')
    cluster = max(np.unique(labels),key=lambda i:(sum(labels==i),-i))
    selected = points[labels==cluster]
    reference = np.median(selected[:,:2],axis=0)
    ground = []
    for f in frames[:3]:
        n = np.array(f['rotation'])@f['normal_sonar']
        d = f['plane_distance_m']+n@f['origin']
        ground.append([-n[0]/n[2],-n[1]/n[2],(d-n[:2]@reference)/n[2]])
    ground = np.median(ground,axis=0)
    centered = selected[:,:2]-reference
    _,vectors = np.linalg.eigh(centered.T@centered)
    v = vectors[:,-1]
    yaw = float((np.arctan2(v[1],v[0])+np.pi/2)%np.pi-np.pi/2)
    c,s = np.cos(yaw),np.sin(yaw)
    rotation = np.array([[c,s],[-s,c]])
    uv = centered@rotation.T
    lo,hi = uv.min(axis=0),uv.max(axis=0)
    center = reference+(lo+hi)/2@rotation
    size = np.maximum(hi-lo,.2)
    heights = selected[:,2]-(ground[2]+centered@ground[:2])
    seed = dict(ground=ground.tolist(),reference_xy=reference.tolist(),center_xy=center.tolist(),
                side_lengths=size.tolist(),height=float(np.clip(np.median(heights),.05,1)),yaw=yaw,
                selected_cluster=int(cluster),cluster_counts={str(i):int(sum(labels==i)) for i in np.unique(labels)})
    p = np.r_[np.zeros(5),size,seed['height'],yaw,0.]
    _,vertices = geometry(p,seed,True)
    (ROOT/'raw').mkdir()
    for f in frames:
        data = (raw/f['file']).read_bytes()
        (ROOT/'raw'/f['file']).write_bytes(data)
        with np.load(raw/f['file'],allow_pickle=False) as z:
            ranges,az = z['ranges'],np.arctan2(z['beam_directions'][:,1],z['beam_directions'][:,0])
        local = (vertices-np.array(f['origin']))@np.array(f['rotation'])
        r = np.linalg.norm(local,axis=1)
        a = np.arctan2(local[:,1],local[:,0])
        ri = np.rint(np.interp(np.linspace(r.min()-.6,r.max()+.6,8),ranges,np.arange(len(ranges)))).astype(int)
        bi = np.rint(np.interp(np.linspace(a.min()-np.deg2rad(8),a.max()+np.deg2rad(8),4),az,np.arange(len(az)))).astype(int)
        f['pixels'] = np.unique(np.array([(i,j) for i in ri for j in bi]+[(i,j) for i in [80,140,200,300] for j in [32,96,160,224]]),axis=0).tolist()
    np.savez_compressed(ROOT/'seed-candidates.npz',points=points,labels=labels,parameters=p)
    write(ROOT/'inputs.json',dict(seed=seed,frames=frames,source_hashes={
        name:sha(source/name) for name in ['input-provenance.json.gz','angular_median-complete.json.xz']},
        protocol_sha256=sha(ROOT/'protocol.json'),construction_truth_scene_rgb_used=False))
    print(json.dumps(dict(prepared_frames=indices,seed=seed,protocol_sha256=sha(ROOT/'protocol.json'))),flush=True)


class Fit:
    def __init__(self):
        self.inputs = json.loads((ROOT/'inputs.json').read_text())
        self.protocol = json.loads((ROOT/'protocol.json').read_text())
        self.seed = self.inputs['seed']
        self.frames = self.inputs['frames']
        self.models, self.powers = {},{}
        for f in self.frames:
            with np.load(ROOT/'raw'/f['file'],allow_pickle=False) as z:
                self.models[f['index']] = make_model(z)
                pixels = np.array(f['pixels'])
                self.powers[f['index']] = 10**(z['intensity_db'][pixels[:,0],pixels[:,1]].astype(float)/10)
        self.start = time.monotonic()
        self.calls = 0

    def evaluate(self, parameters, prism, frames=None, final=False, causal=False):
        triangles, vertices = geometry(parameters,self.seed,prism)
        details, arrays = [],{}
        score = 0.
        for f in self.frames[:3] if frames is None else frames:
            r,o = np.array(f['rotation']),np.array(f['origin'])
            local = (triangles-o)@r
            if causal:
                n,d = np.array(f['normal_sonar']),f['plane_distance_m']
                u = np.cross(n,np.eye(3)[np.argmin(abs(n))]);u/=np.linalg.norm(u)
                v = np.cross(n,u)
                corners = np.array([d*n+100*(a*u+b*v) for a,b in [(-1,-1),(-1,1),(1,1),(1,-1)]])
                local = corners[[[0,1,2],[0,2,3]]]
            obs = SourceSurfaceObservation(self.models[f['index']],TriangleSurface(local))
            pixels = np.array(f['pixels'])
            basis = mean_basis(obs,pixels)
            ratio = np.exp(parameters[9]) if prism else 1.
            mean = basis@np.array([1.,ratio])
            power = self.powers[f['index']]
            active = mean>0
            if np.any((power>0)&~active) or not np.any(active) or not np.any(power[active]>0):
                raise ValueError('unsupported_return_or_gain')
            gain = np.mean(power[active]/mean[active])
            value = float(np.log(mean[active]).sum()+active.sum()*(np.log(gain)+1))
            residual = power[active]/(gain*mean[active])-1
            score += value
            detail = dict(index=f['index'],score=value,gain=float(gain),pixels=len(pixels),
                          normalized_residual_rms=float(np.sqrt(np.mean(residual**2))))
            if final:
                mu = np.r_[np.full(2,.001),np.full(len(local)-2,.001*ratio)]
                moments = obs.moments(mu,pixels)
                np.testing.assert_allclose(mean,moments['mean'],rtol=2e-12,atol=0)
                evidence = profile_power_evidence(power,moments['mean'],moments['covariance'])
                np.testing.assert_allclose(gain,evidence['gain'],rtol=2e-12)
                prefix = f'frame{f["index"]}_'
                for k,v in [('mean',mean),('basis',basis),('observed',power),('covariance',moments['covariance']),
                            ('pixels',pixels),('residual',evidence['normalized_residual'])]:
                    arrays[prefix+k] = v
            details.append(detail)
        self.calls += 1
        return score,details,arrays,triangles,vertices

    def search(self, p, prism, stream):
        steps = np.array([.01,.01,.05,.1,.1,.1,.1,.05,.15,.5])
        lower = np.r_[[-.06,-.06,-.3,-.6,-.6],[.1,.1,.05,-np.pi,-np.log(20)]]
        upper = np.r_[[.06,.06,.3,.6,.6],[1.5,1.5,1.,np.pi,np.log(20)]]
        dimensions = 10 if prism else 3
        p = p.copy()
        best,_,_,_,_ = self.evaluate(p,prism)
        def log(kind,accepted,score,params,error=None):
            row = dict(call=self.calls,family='prism' if prism else 'plane',kind=kind,
                       accepted=accepted,score=score,parameters=params.tolist(),
                       elapsed_s=time.monotonic()-self.start,error=error)
            stream.write(json.dumps(row,allow_nan=False)+'\n');stream.flush()
        log('initial',True,best,p)
        for scale in [1.,.5]:
            for sweep in range(2):
                for j in range(dimensions):
                    for sign in [1,-1]:
                        trial = p.copy();trial[j] = np.clip(p[j]+sign*scale*steps[j],lower[j],upper[j])
                        if np.array_equal(p,trial):continue
                        try:
                            score,_,_,_,_ = self.evaluate(trial,prism)
                            accepted = score<best
                            log(f'{scale}/{sweep}/{j}/{sign}',accepted,score,trial)
                            if accepted:p,best = trial,score
                        except ValueError as e:
                            log(f'{scale}/{sweep}/{j}/{sign}',False,None,trial,str(e))
                print(json.dumps(dict(family='prism' if prism else 'plane',scale=scale,sweep=sweep,
                                      calls=self.calls,best=best,elapsed_s=time.monotonic()-self.start)),flush=True)
        return p


def run():
    f = Fit()
    with np.load(ROOT/'seed-candidates.npz',allow_pickle=False) as z:p = z['parameters'].copy()
    if (ROOT/'trace.jsonl').exists():raise FileExistsError('fit already attempted; preserve trace')
    with (ROOT/'trace.jsonl').open('x') as stream:
        plane = f.search(p,False,stream)
        start = p.copy();start[:3] = plane[:3]
        prism = f.search(start,True,stream)
    report = dict(native_fusion=False,map_accepted=False,conditional_fixed_navigation=True,
                  field_accuracy_accepted=False,training_indices=f.protocol['training_indices'],
                  unused_temporal_indices=f.protocol['unused_temporal_indices'],families={})
    arrays = {}
    for label,params,with_prism,causal in [('causal_plane',p,False,True),('shared_plane',plane,False,False),
                                          ('initial_prism',start,True,False),('fitted_prism',prism,True,False)]:
        value,details,data,triangles,vertices = f.evaluate(params,with_prism,f.frames,True,causal)
        report['families'][label] = dict(parameters=params.tolist(),frames=details,
                                         train_score=sum(d['score'] for d in details[:3]),
                                         unused_temporal_score=sum(d['score'] for d in details[3:]))
        for k,v in data.items():arrays[label+'_'+k] = v
        arrays[label+'_triangles'] = triangles
        arrays[label+'_vertices'] = vertices
    np.savez_compressed(ROOT/'fit-output.npz',**arrays)
    write(ROOT/'fit-output.json',report)
    write(ROOT/'construction-seal.json',dict(truth_scene_rgb_used=False,
        files={name:sha(ROOT/name) for name in ['protocol.json','fit.py','inputs.json','seed-candidates.npz',
                                               'trace.jsonl','fit-output.npz','fit-output.json']},
        elapsed_s=time.monotonic()-f.start,calls=f.calls))
    print(json.dumps(report),flush=True)


if __name__=='__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('mode',choices=['prepare','fit'])
    parser.add_argument('--records',type=Path)
    parser.add_argument('--raw',type=Path)
    args = parser.parse_args()
    if args.mode=='prepare':prepare(args.records,args.raw)
    else:run()
