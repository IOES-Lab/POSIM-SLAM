"""Recompute every prepared two-view fit and third-view compatibility result.

Uses public POSIM-SLAM and NumPy only. Raw-image extraction, native replay,
physical identity, pose covariance and hardware calibration are outside scope.
"""
from pathlib import Path
import argparse
import gzip
import hashlib
import json
import sys
import numpy as np

HERE=Path(__file__).resolve().parent
for ancestor in HERE.parents:
    candidates=(ancestor,ancestor/'libraries/posim-slam')
    found=next((p for p in candidates if (p/'src/posim_slam').is_dir()),None)
    if found is not None:break
else:raise RuntimeError('public_posim_slam_source_required')
sys.path[:0]=[str(found/'src'),str(found/'tools'),str(HERE)]
from evaluate_vio import matrix
from pair_geometry import fit_pair,SIGMA

def load(path):
    return json.loads(gzip.decompress(path.read_bytes())) if path.suffix=='.gz' else json.loads(path.read_text())

def distinct(points,mask,scale):
    retained=[]
    for point in points[mask]:
        if not retained or min(np.linalg.norm(point-p) for p in retained)>1e-4*max(scale,1.):retained.append(point)
    return len(retained)

def fit_inputs(pairs,poses,geometry):
    """This function receives no third-view observations or validation mask."""
    parts={name:[] for name in pairs[0]['variants']}
    for pair in pairs:
        a,b=pair['previous_stamp_ns'],pair['current_stamp_ns']
        if a not in poses or b not in poses:continue
        p0,r0=poses[a];p1,r1=poses[b]
        rotation,translation=r0.T@r1,r0.T@(p1-p0)
        for name,observations in pair['variants'].items():
            n=len(observations['previous_range_azimuth'])
            if not n:continue
            fitted=fit_pair(observations['previous_range_azimuth'],observations['current_range_azimuth'],rotation,translation,geometry=geometry)
            parts[name].append(dict(first_stamp=np.full(n,a,np.int64),second_stamp=np.full(n,b,np.int64),
                first_measurements=np.array(observations['previous_range_azimuth']),
                points_in_first=fitted['points'],points_in_map=p0+fitted['points']@r0.T,
                cost=fitted['cost'],valid=fitted['valid'],converged=fitted['converged']))
    return {name:{key:np.concatenate([p[key] for p in rows]) for key in rows[0]} for name,rows in parts.items()}

def summarize(rows):
    predicted=[r for r in rows if r['fit'] and r['found'] and r['visible']]
    return dict(candidates=len(rows),training_fit=sum(r['fit'] for r in rows),
        validation_found=sum(r['found'] for r in rows),validation_source_supported=sum(r['supported'] for r in rows),
        visible_predictions=len(predicted),family_compatible=sum(r['compatible'] for r in rows),
        chosen_range_abs_error_m_p50_p90=np.percentile([abs(r['error'][0]) for r in predicted],[50,90]).tolist() if predicted else None,
        chosen_azimuth_abs_error_deg_p50_p90=np.degrees(np.percentile([abs(r['error'][1]) for r in predicted],[50,90])).tolist() if predicted else None,
        validation_success_ratio_all_candidates=sum(r['compatible'] for r in rows)/len(rows) if rows else None)

def assess(fits,triplets,poses,geometry):
    result={}
    for name,data in fits.items():
        cost=np.where(data['valid']&data['converged'],data['cost'],np.inf)
        best=cost.argmin(axis=1);index=np.arange(len(cost));available=np.isfinite(cost[index,best])
        near=(data['valid']&data['converged'])&(cost<=cost[index,best,None]+1.)
        modes=np.array([distinct(p,m,z[0]) for p,m,z in zip(data['points_in_first'],near,data['first_measurements'])])
        pair=dict(candidates=len(cost),converged_candidates=int(available.sum()),
            converged_cost_p50_p90=np.percentile(cost[index,best][available],[50,90]).tolist(),
            diagnostic_cost_within_9=int((available&(cost[index,best]<=9.)).sum()),
            multiple_distinct_converged_near_modes=int((modes>1).sum()),
            multiple_near_seed_fits=int((near.sum(axis=1)>1).sum()),
            mode_search='nine initial elevations; distinct converged fits do not prove global uniqueness')
        rows=[];no_pose=0
        for block in triplets:
            a,b,c=block['first_stamp_ns'],block['second_stamp_ns'],block['validation_stamp_ns']
            training=block['variants'][name]
            if a not in poses or b not in poses or c not in poses:no_pose+=1;continue
            ids=np.flatnonzero((data['first_stamp']==a)&(data['second_stamp']==b));n=len(ids)
            assert n==training['training']['matches']
            p,r=poses[c];q=(data['points_in_map'][ids]-p)@r
            radius=np.linalg.norm(q,axis=-1);bearing=np.arctan2(q[...,1],q[...,0]);elevation=np.arctan2(q[...,2],np.hypot(q[...,0],q[...,1]))
            visible=(q[...,0]>0)&(radius>=geometry['range_min'])&(radius<=geometry['range_max'])&(abs(bearing)<=geometry['horizontal_fov']/2)&(abs(elevation)<=geometry['vertical_fov']/2)
            measured=np.array(training['validation_range_azimuth']).reshape(-1,2)
            error=np.stack([radius,bearing],axis=-1)-measured[:,None,:]
            error[...,1]=np.arctan2(np.sin(error[...,1]),np.cos(error[...,1]))
            found=np.array(training['validation_found'],bool);supported=np.array(training['validation_source_supported'],bool)
            minimum=np.min(np.where(near[ids]&visible,np.sum((error/SIGMA)**2,axis=-1),np.inf),axis=1)
            fit=available[ids];chosen=best[ids];training_cost=cost[ids,chosen]
            for i in range(n):
                rows.append(dict(fit=bool(fit[i]),found=bool(found[i]),supported=bool(supported[i]),
                    visible=bool(visible[i,chosen[i]]),compatible=bool(found[i] and fit[i] and minimum[i]<=9.),
                    error=error[i,chosen[i]],selected=bool(fit[i] and training_cost[i]<=9.)))
        result[name]=dict(pair=pair,triplet_all=summarize(rows),
            triplet_training_selected=summarize([r for r in rows if r['selected']]),no_pose_blocks=no_pose)
    return result

checks=0
def compare(actual,expected,path=''):
    global checks
    if isinstance(expected,dict):
        assert actual.keys()==expected.keys(),path
        for k in expected:compare(actual[k],expected[k],path+'/'+k)
    elif isinstance(expected,list):
        assert len(actual)==len(expected),path
        for i,(a,b) in enumerate(zip(actual,expected)):compare(a,b,path+'/'+str(i))
    elif isinstance(expected,float):
        np.testing.assert_allclose(actual,expected,rtol=1e-8,atol=1e-9,err_msg=path);checks+=1
    else:assert actual==expected,(path,actual,expected);checks+=1

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--case',choices=['wall','finite'],action='append')
    parser.add_argument('--output',type=Path,help='Retain every recomputed mode in a new directory')
    args=parser.parse_args()
    for name,digest in load(HERE/'sha256.json').items():
        assert hashlib.sha256((HERE/name).read_bytes()).hexdigest()==digest,name
    if args.output:args.output.mkdir(parents=True,exist_ok=False)
    reports={}
    for name in args.case or ['wall','finite']:
        folder=HERE/'data'/name;metadata=load(folder/'geometry.json')
        pairs=load(folder/'pairs.json.gz');triplets=load(folder/'triplets.json.gz')
        trace=np.genfromtxt(folder/'navigation.csv',delimiter=',',names=True)
        mount=np.array(metadata['sonar_to_imu']);lever=np.array(metadata['imu_to_sonar']);poses={}
        for row in trace:
            r=matrix([row[k] for k in ('qx','qy','qz','qw')]);poses[int(row['stamp_ns'])]=(np.array([row[k] for k in ('x','y','z')])+r@lever,r@mount)
        fitted=fit_inputs(pairs,poses,metadata['geometry'])
        actual=assess(fitted,triplets,poses,metadata['geometry'])
        compare(actual,load(folder/'expected-summary.json')['variants'],name)
        if args.output:
            destination=args.output/name;destination.mkdir()
            for variant,values in fitted.items():np.savez_compressed(destination/(variant+'.npz'),**values)
            (destination/'summary.json').write_text(json.dumps(actual,indent=2)+'\n')
        reports[name]={variant:values['triplet_training_selected']['family_compatible'] for variant,values in actual.items()}
    print(json.dumps(dict(passed=True,summary_checks=checks,cases=reports,
        scope='Prepared range/bearing fits and unused-view predictions; no raw extraction, covariance or SLAM qualification')))

if __name__=='__main__':main()
