# SPDX-License-Identifier: Apache-2.0
"""Experimental fixed-pose reconstruction using the generic POSIM sonar profile.

Known estimated poses and intensity are inputs. This proxy omits occlusion,
surface normals, coherent cross terms and pose uncertainty. Its output is a
nonnegative scattering field, never certified obstacles or navigation factors.
"""
import numpy as np


class Projector:
    def __init__(self, points, poses, azimuth, ranges, vertical_fov, *, range_power=3., absorption_db_m=.0354, maximum_range=None, beam_pixel_width=None, native_ranges=None):
        self.points=np.asarray(points,float);self.poses=poses;self.azimuth=np.asarray(azimuth,float)
        self.ranges=np.asarray(ranges,float);self.size=len(points);self.nr=len(ranges)
        azimuth=self.azimuth;ranges=self.ranges
        if (self.points.ndim!=2 or self.points.shape[1:]!=(3,) or not 0<len(self.points)<=800_000
                or not 2<=len(poses)<=48 or azimuth.ndim!=1 or not 3<=len(azimuth)<=512
                or ranges.ndim!=1 or not 2<=len(ranges)<=1024
                or not all(np.isfinite(v).all() for v in (self.points,azimuth,ranges))
                or not np.all(np.diff(ranges)>0) or ranges[0]<0
                or not np.isfinite(vertical_fov) or not 0<vertical_fov<np.pi
                or not np.isfinite(range_power) or not 0<=range_power<=8
                or not np.isfinite(absorption_db_m) or absorption_db_m<0):
            raise ValueError('bounded_finite_projection_inputs_required')
        for pose in poses:
            position=np.asarray(pose['position']);rotation=np.asarray(pose['rotation'])
            if (position.shape!=(3,) or rotation.shape!=(3,3)
                    or not np.isfinite(position).all() or not np.isfinite(rotation).all()
                    or not np.allclose(rotation.T@rotation,np.eye(3),atol=1e-8,rtol=0)
                    or not np.isclose(np.linalg.det(rotation),1.,atol=1e-8,rtol=0)):
                raise ValueError('rigid_sonar_poses_required')
        self.groups=[np.arange(i,min(i+3,len(azimuth))) for i in range(0,len(azimuth),3)]
        self.nb=len(self.groups);self.shape=(len(poses),self.nr,self.nb)
        maximum_range=ranges[-1]+(ranges[-1]-ranges[-2])/2 if maximum_range is None else maximum_range
        if not np.isfinite(maximum_range) or maximum_range<=.2:raise ValueError('positive_range_required')
        from scipy.sparse import coo_matrix,eye
        self.fine_ranges=np.asarray(ranges if native_ranges is None else native_ranges)
        self.fine_nr=len(self.fine_ranges)
        if (self.fine_ranges.ndim!=1 or not 2<=self.fine_nr<=1024
                or not np.isfinite(self.fine_ranges).all() or not np.all(np.diff(self.fine_ranges)>0)):
            raise ValueError('ordered_native_ranges_required')
        if native_ranges is None:self.range_mix=eye(self.nr,format='csr')
        else:
            if maximum_range>self.fine_ranges[-1]+self.fine_ranges[1]-self.fine_ranges[0]:raise ValueError('native_range_alias_period_exceeded')
            starts=np.arange(0,self.fine_nr,4);counts=np.minimum(4,self.fine_nr-starts)
            np.testing.assert_allclose(np.add.reduceat(self.fine_ranges,starts)/counts,ranges,rtol=1e-12,atol=1e-12)
            # Source window is periodic Hamming, normalized in frequency.
            # Its integer-bin DFT power has exactly the center and +/-1 taps.
            # Sub-bin delays interpolate adjacent native bins before averaging.
            taps=np.array([(.46/2)**2,.54**2,(.46/2)**2]);taps/=taps.sum()
            source=np.arange(self.fine_nr);rows=[];cols=[];weights=[]
            for offset,weight in zip((-1,0,1),taps):
                row=((source+offset)%self.fine_nr)//4;rows.extend(row);cols.extend(source);weights.extend(weight/counts[row])
            self.range_mix=coo_matrix((weights,(rows,cols)),shape=(self.nr,self.fine_nr)).tocsr()
        range_gram=(self.range_mix.T@self.range_mix).toarray()
        if not np.all(np.diff(azimuth)>0):raise ValueError('ordered_azimuth_required')
        beam_pixel_width=(azimuth[-1]-azimuth[0])/(len(azimuth)-1) if beam_pixel_width is None else beam_pixel_width
        if not np.isfinite(beam_pixel_width) or beam_pixel_width<=0:raise ValueError('positive_beam_width_required')
        amplitude=np.abs(np.sinc(.884/beam_pixel_width*np.sin(azimuth[:,None]-azimuth[None,:])))
        power=amplitude**2/np.sum(amplitude**2)
        self.kernel=np.array([power[group].mean(axis=0) for group in self.groups])
        gram=self.kernel.T@self.kernel;self.entries=[];diagonal=np.zeros(self.size)
        alpha=absorption_db_m*np.log(10)/20
        entry_bytes=0
        for pose in poses:
            local=(self.points-pose['position'])@pose['rotation'];distance=np.linalg.norm(local,axis=1)
            az=np.arctan2(local[:,1],local[:,0]);el=np.arctan2(local[:,2],np.hypot(local[:,0],local[:,1]))
            keep=(local[:,0]>0)&(abs(el)<=vertical_fov/2)&(az>=azimuth[0])&(az<=azimuth[-1])&(distance>=.2)&(distance<=maximum_range)
            ids=np.flatnonzero(keep).astype(np.int32);d=distance[ids]
            a1=np.clip(np.searchsorted(azimuth,az[ids]),1,len(azimuth)-1).astype(np.int32);a0=a1-1
            wa=(az[ids]-azimuth[a0])/(azimuth[a1]-azimuth[a0])
            fine=self.fine_ranges;r1=np.clip(np.searchsorted(fine,d),1,len(fine)-1);r0=r1-1
            wr=np.clip((d-fine[r0])/(fine[r1]-fine[r0]),0,1)
            if native_ranges is not None:
                period=fine[-1]+fine[1]-fine[0];wrap=d>fine[-1];r0[wrap]=len(fine)-1;r1[wrap]=0;wr[wrap]=(d[wrap]-fine[-1])/(period-fine[-1])
            gain=d**(-range_power)*np.exp(-4*alpha*d)
            columns=[]
            for ri,ar,weight in [(r0,a0,(1-wr)*(1-wa)),(r0,a1,(1-wr)*wa),(r1,a0,wr*(1-wa)),(r1,a1,wr*wa)]:
                columns.append(((ri*len(azimuth)+ar).astype(np.int32),(weight*gain).astype(np.float64)))
            k2=(1-wa)**2*gram[a0,a0]+2*wa*(1-wa)*gram[a0,a1]+wa**2*gram[a1,a1]
            range2=(1-wr)**2*range_gram[r0,r0]+2*wr*(1-wr)*range_gram[r0,r1]+wr**2*range_gram[r1,r1]
            diagonal[ids]+=gain**2*range2*k2
            self.entries.append((ids,columns))
            entry_bytes+=ids.nbytes+sum(i.nbytes+w.nbytes for i,w in columns)
            if entry_bytes>460_000_000:raise MemoryError('projection_entry_budget')
        self.sensitivity=np.sqrt(diagonal);self.scale=np.divide(1.,self.sensitivity,out=np.zeros_like(diagonal),where=diagonal>0)
        self.bytes=sum(ids.nbytes+sum(i.nbytes+w.nbytes for i,w in cols) for ids,cols in self.entries)+self.points.nbytes+self.scale.nbytes+self.sensitivity.nbytes+self.kernel.nbytes+self.range_mix.data.nbytes+self.range_mix.indices.nbytes+self.range_mix.indptr.nbytes
        if self.bytes>480_000_000:raise MemoryError('projection_memory_budget')

    def forward(self,values):
        values=np.asarray(values)
        if values.shape!=(self.size,) or not np.isfinite(values).all():raise ValueError('finite_coefficient_vector_required')
        values=values*self.scale;result=[]
        for ids,columns in self.entries:
            image=np.zeros(self.fine_nr*len(self.azimuth))
            for indices,weights in columns:image+=np.bincount(indices,weights=weights*values[ids],minlength=len(image))
            result.append((self.range_mix@image.reshape(self.fine_nr,-1))@self.kernel.T)
        return np.asarray(result)

    def adjoint(self,images):
        images=np.asarray(images)
        if images.shape!=self.shape or not np.isfinite(images).all():raise ValueError('finite_observation_tensor_required')
        result=np.zeros(self.size)
        for image,(ids,columns) in zip(images,self.entries):
            back=(self.range_mix.T@(image@self.kernel)).ravel();value=np.zeros(len(ids))
            for indices,weights in columns:value+=back[indices]*weights
            result[ids]+=value
        return result*self.scale


def solve(projector,observed,*,fraction=.02,iterations=80):
    """Nonnegative L1 least squares with checked quadratic majorization."""
    b=np.asarray(observed)
    if (b.shape!=projector.shape or not np.isfinite(b).all() or np.any(b<0)
            or not np.isfinite(fraction) or not 0<=fraction<=.2
            or type(iterations) is not int or not 1<=iterations<=200):raise ValueError('bounded_nonnegative_fit_inputs_required')
    atb=projector.adjoint(b);lam=fraction*float(atb.max())
    if not np.linalg.norm(b)>0:return np.zeros(projector.size),dict(lambda_value=lam,lambda_fraction=fraction,iterations=0,history=[],empty_signal=True)
    if not np.any(atb>0):return np.zeros(projector.size),dict(lambda_value=lam,lambda_fraction=fraction,iterations=0,history=[],unsupported_signal=True)
    rng=np.random.default_rng(71);direction=rng.normal(size=projector.size);direction[projector.scale==0]=0;direction/=np.linalg.norm(direction)
    for _ in range(10):
        value=projector.adjoint(projector.forward(direction));length=np.linalg.norm(value)
        if not length>0:raise ValueError('empty_operator')
        direction=value/length
    lipschitz=float(direction@projector.adjoint(projector.forward(direction)))*1.1
    x=np.zeros(projector.size);y=x.copy();momentum=1.;history=[]
    for iteration in range(iterations):
        residual=projector.forward(y)-b;gradient=projector.adjoint(residual);f=.5*float(np.sum(residual**2))
        for trial in range(30):
            candidate=np.maximum(0,y-(gradient+lam)/lipschitz);delta=candidate-y
            candidate_residual=projector.forward(candidate)-b;newf=.5*float(np.sum(candidate_residual**2))
            bound=f+float(gradient@delta)+.5*lipschitz*float(delta@delta)
            if newf<=bound+1e-10*max(1.,f):break
            lipschitz*=2
        else:raise RuntimeError('majorization_failed')
        new_momentum=(1+np.sqrt(1+4*momentum**2))/2
        y=candidate+(momentum-1)/new_momentum*(candidate-x);x=candidate;momentum=new_momentum
        if iteration%10==0 or iteration==iterations-1:
            row=dict(iteration=iteration,objective=newf+lam*float(x.sum()),relative_power_l2=float(np.linalg.norm(candidate_residual)/np.linalg.norm(b)),positive_coefficients=int(np.count_nonzero(x)),lipschitz=lipschitz)
            history.append(row);print(row,flush=True)
    return x,dict(lambda_value=lam,lambda_fraction=fraction,iterations=iterations,history=history)


def analytic_probe():
    rng=np.random.default_rng(12);az=np.linspace(-np.pi/3,np.pi/3,31);ranges=np.linspace(.1,7,71)
    points=np.array([[4.,.2,-.5],[4.,.2,-1.],[4.,.2,-1.5],[5.,.2,-1.],[3.,.2,-1.]])
    poses=[dict(position=np.array([x,0,z]),rotation=np.eye(3)) for x,z in [(0,0),(.5,-.2),(1.,.2)]]
    az[13]+=.001
    p=Projector(points,poses,az,ranges,np.deg2rad(60))
    x=rng.normal(size=len(points));y=rng.normal(size=p.shape)
    np.testing.assert_allclose(np.sum(p.forward(x)*y),x@p.adjoint(y),rtol=1e-12,atol=1e-12)
    matrix=np.column_stack([p.forward(v).ravel() for v in np.eye(len(points))])
    np.testing.assert_allclose(np.sum(matrix**2,axis=0),1.,rtol=1e-12)
    # A known point on x=4 has analytic range 4 and azimuth zero, not a pixel projection.
    exact=Projector(np.array([[4.,0.,0.]]),[dict(position=np.zeros(3),rotation=np.eye(3))]*2,az,np.arange(.1,7.1,.1),np.deg2rad(20))
    image=exact.forward(np.ones(1))[0];assert np.argmax(image.sum(axis=1))==39
    from scipy.optimize import nnls
    reference=np.array([0.,2.,0.,0.,0.]);answer,_=nnls(matrix,matrix@reference)
    np.testing.assert_allclose(answer,reference,atol=1e-10)
    native=np.arange(80)*.1;coarse=native.reshape(-1,4).mean(1)
    h=Projector(points,poses,az,coarse,np.deg2rad(60),native_ranges=native)
    v=rng.normal(size=h.shape);np.testing.assert_allclose(np.sum(h.forward(x)*v),x@h.adjoint(v),rtol=1e-12,atol=1e-12)
    hm=np.column_stack([h.forward(v).ravel() for v in np.eye(len(points))]);np.testing.assert_allclose(np.sum(hm**2,axis=0),1.,rtol=1e-12)
    # Match the DFT of the actual periodic source window, not a fitted PSF.
    window=.54-.46*np.cos(2*np.pi*(np.arange(80)+1)/80);power=abs(np.fft.fft(window))**2;power/=power.sum()
    expected=np.zeros(80);expected[[79,0,1]]=np.array([(.46/2)**2,.54**2,(.46/2)**2])/((.54**2)+2*(.46/2)**2)
    np.testing.assert_allclose(power,expected,atol=1e-14)
    return dict(nonuniform_grid_adjoint=True,column_normalization=True,analytic_range_peak=True,multiview_single_point=True,native_range_adjoint=True,native_range_column_normalization=True,source_window_dft=True)


def load_packet(path):
    """Only numeric measured observations, fixed estimated poses and calibration."""
    with np.load(path,allow_pickle=False) as values:packet={k:values[k] for k in values.files}
    validate_packet(packet)
    return packet


def validate_packet(packet):
    """Validate the same contracts before either CLI or direct API allocation."""
    required={'positions','rotations','stamps','observed','train','test','azimuth','ranges','vertical_fov','maximum_range','beam_pixel_width','range_power','frame_id'}
    if not required.issubset(packet):raise ValueError('incomplete_measured_packet')
    count=len(packet['positions']);train=packet['train'];test=packet['test']
    if (not 4<=count<=48 or packet['positions'].shape!=(count,3)
            or packet['rotations'].shape!=(count,3,3) or packet['stamps'].shape!=(count,)
            or not np.isfinite(packet['positions']).all() or not np.isfinite(packet['rotations']).all()
            or not np.allclose(packet['rotations'].transpose(0,2,1)@packet['rotations'],np.eye(3),atol=1e-8,rtol=0)
            or not np.allclose(np.linalg.det(packet['rotations']),1.,atol=1e-8,rtol=0)
            or packet['stamps'].dtype.kind not in 'iu' or np.any(packet['stamps']<0) or not np.all(packet['stamps'][1:]>packet['stamps'][:-1])
            or packet['observed'].ndim!=3 or len(packet['observed'])!=count
            or not np.isfinite(packet['observed']).all() or np.any(packet['observed']<0)
            or train.ndim!=1 or test.ndim!=1 or train.dtype.kind not in 'iu' or test.dtype.kind not in 'iu'
            or min(len(train),len(test))<2 or sorted(np.r_[train,test].tolist())!=list(range(count))
            or packet['frame_id'].shape!=() or not str(packet['frame_id']) or any(c.isspace() for c in str(packet['frame_id']))):
        raise ValueError('disjoint_ordered_measured_packet_required')
    az=packet['azimuth'];ranges=packet['ranges']
    scalar_names=('vertical_fov','maximum_range','beam_pixel_width','range_power')
    if (any(packet[k].shape!=() or not np.isfinite(packet[k]) for k in scalar_names)
            or not 0<float(packet['vertical_fov'])<np.pi
            or not .2<float(packet['maximum_range'])<=200
            or float(packet['beam_pixel_width'])<=0 or not 0<=float(packet['range_power'])<=8
            or az.ndim!=1 or not 3<=len(az)<=512 or not np.isfinite(az).all()
            or not np.all(np.diff(az)>0) or np.max(np.abs(az))>=np.pi/2
            or ranges.ndim!=1 or not 2<=len(ranges)<=1024 or not np.isfinite(ranges).all()
            or ranges[0]<0 or not np.all(np.diff(ranges)>0)
            or packet['observed'].shape[1:]!=(len(ranges),(len(az)+2)//3)):
        raise ValueError('bounded_calibrated_observations_required')


def reconstruction(packet, *, cell=.2, refine=2, native_range=True, iterations=80, fraction=.02):
    """Fit training observations; test intensities affect only reported scores."""
    if not np.isfinite(cell) or not .1<=cell<=.3 or type(refine) is not int or not 0<=refine<=2:raise ValueError('bounded_reconstruction_grid_required')
    validate_packet(packet)
    if native_range and 'native_ranges' not in packet:raise ValueError('native_range_bins_required')
    from scipy.spatial import cKDTree
    poses=[dict(position=p,rotation=q) for p,q in zip(packet['positions'],packet['rotations'])]
    azimuth=packet['azimuth'];vertical=float(packet['vertical_fov']);maximum=float(packet['maximum_range'])
    az=np.linspace(azimuth[0],azimuth[-1],257);el=np.linspace(-vertical/2,vertical/2,65)
    rays=np.stack(np.broadcast_arrays(np.cos(az[:,None])*np.cos(el),np.sin(az[:,None])*np.cos(el),np.sin(el)),axis=-1).reshape(-1,3)
    bound=np.concatenate([np.vstack([p['position'],p['position']+maximum*rays@p['rotation'].T]) for p in poses])
    lower=np.floor(bound.min(0)/cell)*cell-cell;upper=np.ceil(bound.max(0)/cell)*cell+cell;shape=np.ceil((upper-lower)/cell)
    if not np.isfinite(shape).all() or np.prod(shape)>800_000:raise MemoryError('voxel_budget_exceeded')
    shape=shape.astype(int)
    axes=[lower[i]+(np.arange(shape[i])+.5)*cell for i in range(3)];points=np.stack(np.meshgrid(*axes,indexing='ij'),axis=-1).reshape(-1,3);cells=np.full(len(points),cell)
    train=packet['train'];test=packet['test'];observed=packet['observed'];kwargs=dict(vertical_fov=vertical,maximum_range=maximum,beam_pixel_width=float(packet['beam_pixel_width']),range_power=float(packet['range_power']))
    if native_range:kwargs['native_ranges']=packet['native_ranges']
    stages=[]
    for level in range(refine+1):
        model=Projector(points,[poses[i] for i in train],azimuth,packet['ranges'],**kwargs)
        heldout=Projector(points,[poses[i] for i in test],azimuth,packet['ranges'],**kwargs)
        if model.bytes+heldout.bytes>650_000_000:raise MemoryError('combined_projection_budget')
        coefficient,fit=solve(model,observed[train],fraction=fraction,iterations=iterations if level==0 else min(200,iterations+20))
        predicted=heldout.forward(coefficient);denominator=float(np.linalg.norm(observed[test]));error=float(np.linalg.norm(predicted-observed[test])/denominator) if denominator else None
        stages.append(dict(level=level,total_cells=len(points),positive_cells=int(np.count_nonzero(coefficient)),minimum_cell_m=float(cells.min()),projection_bytes=model.bytes+heldout.bytes,heldout_relative_power_l2=error,fit=fit))
        if level==refine or not np.any(coefficient>0):break
        tree=cKDTree(points);chosen=set()
        for i in np.flatnonzero(coefficient>0):chosen.update(tree.query_ball_point(points[i],np.sqrt(3)*cells[i]*1.01))
        chosen=np.array(sorted(chosen),dtype=int);mask=np.ones(len(points),bool);mask[chosen]=False
        offsets=np.array([[x,y,z] for x in (-1,1) for y in (-1,1) for z in (-1,1)])
        children=points[chosen,None,:]+offsets[None,:,:]*cells[chosen,None,None]/4
        points=np.concatenate([points[mask],children.reshape(-1,3)]);cells=np.r_[cells[mask],np.repeat(cells[chosen]/2,8)]
        if len(points)>800_000:raise MemoryError('adaptive_voxel_budget')
    return dict(points=points,coefficient=coefficient,albedo=coefficient*model.scale,cell_m=cells,predicted=predicted,observed=observed[test],stages=stages)


def write_cells(path,result,frame,stamp):
    """Standard binary PLY for inspection; every positive cell is preserved."""
    keep=result['coefficient']>0;points=result['points'][keep]
    values=np.empty(len(points),dtype=[(k,'<f4') for k in ('x','y','z','cell_size_m','coefficient','relative_albedo')])
    for i,k in enumerate(('x','y','z')):values[k]=points[:,i]
    values['cell_size_m']=result['cell_m'][keep];values['coefficient']=result['coefficient'][keep];values['relative_albedo']=result['albedo'][keep]
    header=['ply','format binary_little_endian 1.0','comment scope experimental_sonar_scattering_cells','comment map_qualified false','comment navigation_ready false','comment frame_id '+frame,'comment stamp_ns '+str(stamp),'element vertex '+str(len(points))]
    header.extend('property float '+name for name in values.dtype.names);header.append('end_header')
    path.write_bytes(('\n'.join(header)+'\n').encode()+values.tobytes())


def main():
    import argparse,hashlib,json,time
    from pathlib import Path
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--inputs',type=Path);p.add_argument('--output',type=Path);p.add_argument('--cell',type=float,default=.2);p.add_argument('--refine',type=int,choices=(0,1,2),default=2);p.add_argument('--iterations',type=int,default=80);p.add_argument('--fraction',type=float,default=.02);p.add_argument('--range-response',choices=('native-hamming','coarse-linear'),default='native-hamming')
    a=p.parse_args()
    if a.inputs is None:print(analytic_probe());return
    if a.output is None:p.error('--output is required with --inputs')
    packet=load_packet(a.inputs);a.output.mkdir(parents=True,exist_ok=False);start=time.perf_counter()
    result=reconstruction(packet,cell=a.cell,refine=a.refine,native_range=a.range_response=='native-hamming',iterations=a.iterations,fraction=a.fraction)
    np.savez_compressed(a.output/'field.npz',**{k:result[k] for k in ('points','coefficient','albedo','cell_m')})
    np.savez_compressed(a.output/'predictions.npz',observed=result['observed'],predicted=result['predicted'])
    write_cells(a.output/'albedo-cells.ply',result,str(packet['frame_id']),int(packet['stamps'][packet['train']].max()))
    report=dict(schema='posim-sonar-volume-diagnostic-1',map_qualified=False,navigation_ready=False,field_type='relative scattering coefficients; not occupancy probability or certified landmarks',frame=str(packet['frame_id']),cell_m=a.cell,refine=a.refine,range_response=a.range_response,stages=result['stages'],inputs_sha256=hashlib.sha256(a.inputs.read_bytes()).hexdigest(),source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),versions=dict(numpy=np.__version__,scipy=__import__('scipy').__version__),elapsed_s=time.perf_counter()-start,assumptions=['fixed estimated poses without pose covariance','generic POSIM mean-power beam response','per-ray range-power proxy','surface normals and occlusion omitted','approximate sub-bin range response','L1-selected field support is not an uncertainty certificate'])
    (a.output/'report.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report))


if __name__=='__main__':main()
