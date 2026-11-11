"""Optional Matplotlib figure from the complete retained diagnostic outputs."""
from pathlib import Path
import json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
plt.rcParams['svg.hashsalt']='posim-slam-sampled-dvl-20261101'

root=Path(__file__).resolve().parent
result=json.loads((root/'assessment.json').read_text())
with np.load(root/'expected.npz',allow_pickle=False) as record:
    p=record['seed_projection'];q=record['ray_projection'];outside=record['outside_hull'];hull=record['hull']
fig,axes=plt.subplots(1,2,figsize=(13.8,5.4))
fig.subplots_adjust(left=.065,right=.98,bottom=.19,top=.68,wspace=.24)
fig.suptitle('Sampled DVL: conditional plane and spatial support',fontsize=16,y=.98)
left,right=axes
closed=np.vstack((hull,hull[0]))
left.plot(closed[:,0],closed[:,1],color='#354658',lw=1.5,ls='--',label='Nominal seed hull')
left.scatter(q[~outside,0],q[~outside,1],s=64,color='#6fa900',alpha=.6,label='Ray hit inside hull')
left.scatter(q[outside,0],q[outside,1],s=64,color='#ed785e',alpha=.8,label='Ray hit outside hull')
left.scatter(p[:,0],p[:,1],s=64,color='#164d80',marker='x',lw=1.6,label='Nominal axis seed')
left.set(xlabel='Plane coordinate u (m)',ylabel='Plane coordinate v (m)')
left.set_aspect('equal',adjustable='datalim')
left.set_title('38 / 176 hits outside; maximum extension 89.64 mm',loc='left',fontsize=10,y=1.29)
left.legend(loc='lower left',bbox_to_anchor=(-.02,1.015),ncol=2,fontsize=8.5,frameon=False)
rows=result['range_cases'];colors=plt.get_cmap('turbo')([.15,.4,.7,.95])
for beam,color in zip(sorted(set(r['beam_id'] for r in rows)),colors):
    values=[r for r in rows if r['beam_id']==beam]
    right.scatter([r['stamp_ns']/1e9 for r in values],[1000*r['residual_m'] for r in values],
        s=64,color=color,alpha=.75,label='Beam '+str(beam))
training=[r['stamp_ns']/1e9 for r in rows if r['training']]
right.axvspan(min(training),max(training),color='#728fa6',alpha=.12)
right.axhline(0,color='#6f7780',lw=.8)
right.set(xlabel='Original sensor time (s)',ylabel='Range residual (mm)',ylim=(-75,75))
right.set_title('32 fitting ranges: 7.64 mm RMS; 300 unused: 30.11 mm RMS',loc='left',fontsize=10,y=1.29)
right.legend(loc='lower left',bbox_to_anchor=(-.02,1.015),ncol=4,fontsize=8.5,frameon=False)
for ax in axes:
    ax.grid(alpha=.18);ax.spines[['top','right']].set_visible(False)
fig.text(.065,.065,'Every original range is retained. Shading marks fitting times. Markers: 64 pt².\n'
    'Plane hits and axis seeds are conditional geometry, not measured XYZ. Uncalibrated error model; no native update or map admission.',fontsize=9,color='#49515b')
fig.savefig(root/'comparison.png',dpi=180,facecolor='white')
fig.savefig(root/'comparison.svg',facecolor='white',metadata={'Date':None})
svg=root/'comparison.svg'
svg.write_text('\n'.join(line.rstrip() for line in svg.read_text().splitlines())+'\n')
plt.close(fig)
