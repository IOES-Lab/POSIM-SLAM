"""Render saved calibration inputs/results; no estimator or simulator required."""
from pathlib import Path
import argparse,json
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from posim_slam.sonar_range_response import PosimRangeResponse
from verify import expected_panel,relative_residual

ROOT=Path(__file__).resolve().parent
parser=argparse.ArgumentParser();parser.add_argument('--operator-record',type=Path,default=ROOT.parent/'sonar-pr31-36-20261030');args=parser.parse_args()
fig,axes=plt.subplots(2,2,figsize=(12,8),layout='constrained')
with np.load(args.operator_record/'endpoint-ffi-all.npz',allow_pickle=False) as data:
    saved=data['n599_s73_d5.213'][-1];values=saved[:,0]+1j*saved[:,1]
    response=PosimRangeResponse(599,29900.,1500.).complex_response([5.213])[0]
    gain=np.vdot(response,values)/np.vdot(response,response)
    scale=np.max(abs(values)**2)
    ranges=np.arange(599)*1500/(2*29900)
    axes[0,0].semilogy(ranges,abs(values)**2/scale,label='Saved native GPU',color='#0072b2')
    axes[0,0].semilogy(ranges,abs(gain*response)**2/scale,label='Aperture model',color='#e69f00',linestyle='--')
    axes[0,0].set(xlabel='Range (m)',ylabel='Normalized power',title='Known single return: full 599-bin range axis')
    axes[0,0].legend()
with np.load(ROOT/'captures/sonoptix/summary.npz',allow_pickle=False) as data:
    error=(data['xyz'][...,0].astype(float)-4.)*100
    raster=axes[0,1].imshow(error,origin='lower',aspect='auto',cmap='turbo',vmin=-2,vmax=2)
    axes[0,1].set(xlabel='Horizontal ray index',ylabel='Vertical ray index',title='256 x 256 renderer: wall-normal error')
    fig.colorbar(raster,ax=axes[0,1],label='Error (cm)')
    dirs=data['beam_directions'];angles=np.arctan2(dirs[:,1],dirs[:,0])
    expected,correlation=expected_panel(data['xyz'],data['normal_rgb8'],angles,599,15.,1)
    residual,prediction=relative_residual(data['mean_power'],expected)
    measured=data['mean_power'];scale=measured[128].max()
    axes[1,0].semilogy(data['ranges'],measured[128]/scale,label='25-ping measured mean',color='#0072b2')
    axes[1,0].semilogy(data['ranges'],prediction[128]/scale,label='Rendered-input expectation',color='#e69f00',linestyle='--')
    axes[1,0].set(xlabel='Range (m)',ylabel='Normalized mean power',title='Default intrinsics: whole profile at beam 128')
    axes[1,0].legend(fontsize=8)
    heat=axes[1,1].imshow(correlation,vmin=0,vmax=1,cmap='viridis')
    for i in range(3):
        for j in range(3):axes[1,1].text(j,i,f'{correlation[i,j]:.2f}',ha='center',va='center',color='black' if correlation[i,j]>.65 else 'white')
    labels=['b128/r160','b128/r161','b129/r160']
    axes[1,1].set(xticks=range(3),yticks=range(3),xticklabels=labels,yticklabels=labels,title='Conditional power correlation; not pose covariance')
    fig.colorbar(heat,ax=axes[1,1],label='Correlation')
for axis in (axes[0,0],axes[1,0]):axis.grid(alpha=.2)
fig.suptitle('POSIM source-response calibration: static panel, no SLAM cloud',fontsize=14)
fig.savefig(ROOT/'range-response-calibration.png',dpi=150)
plt.close(fig)
