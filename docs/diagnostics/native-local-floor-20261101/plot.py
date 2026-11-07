"""Scientific comparison of every retained support diagnostic, not a SLAM map."""
from pathlib import Path
import json
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

matplotlib.rcParams['svg.hashsalt']='posim-slam-native-local-floor-20261101'

ROOT=Path(__file__).resolve().parent
record=json.loads((ROOT/'assessment.json').read_text())
geometry=dict(flat_controls=record['flat_controls'],contexts=dict(initialized_reference=record['initial_support_comparison']))
local=dict(local_windows=record['local_windows'])
fig,ax=plt.subplots(2,2,figsize=(13,8.6),layout='constrained')
blue='#2463c5';orange='#d67000';green='#228447';red='#b83232'
for tilt,style in [(0.,'-'),(.15,'--')]:
    rows=[r for r in geometry['flat_controls'] if r['tilt_rad']==tilt]
    ax[0,0].plot([r['height_m'] for r in rows],[100*r['nominal_maximum_plane_error_m'] for r in rows],
        style,color=blue,marker='o',label=f'Centre axis, tilt {tilt:g} rad')
    ax[0,0].plot([r['height_m'] for r in rows],[100*r['conditional_maximum_plane_error_m'] for r in rows],
        style,color=orange,marker='x',label=f'Minimum ray, tilt {tilt:g} rad')
ax[0,0].axhline(5,color=red,lw=1,label='Unchanged 5 cm planarity guard')
ax[0,0].set(title='A  Exact flat-floor controls',xlabel='Sensor height (m)',ylabel='Maximum endpoint plane error (cm)')
ax[0,0].legend(fontsize=8)
rows=geometry['contexts']['initialized_reference']['rows'];x=np.arange(len(rows))
for name,color,label in [('nominal',blue,'Centre axis'),('conditional',orange,'Conditional minimum ray')]:
    ax[0,1].scatter(x,[100*r[name+'_plane_error_m'] for r in rows],s=64,color=color,label=label,alpha=.7)
for v in [-5,5]:ax[0,1].axhline(v,color=red,lw=1)
ax[0,1].axvline(31.5,color='0.5',lw=1,ls=':')
ax[0,1].text(15.5,-5.5,'32 factor ranges',ha='center',fontsize=8)
ax[0,1].text(47.5,-5.5,'32 unfused guards',ha='center',fontsize=8)
ax[0,1].set(title='B  Same recorded ranges and fitted plane',xlabel='Original range index in frozen selection',ylabel='Signed endpoint plane error (cm)')
ax[0,1].legend(fontsize=8,loc='upper left')
windows=local['local_windows'];indices=np.arange(len(windows))
labels=[f"{r['start_packet_index']+3.01:.0f}" for r in windows]
for name,color,offset,label in [('nominal',blue,-.18,'Centre axis'),('conditional',orange,.18,'Conditional minimum ray')]:
    ax[1,0].bar(indices+offset,[100*r[name]['maximum_plane_deviation_m'] for r in windows],width=.34,color=color,label=label)
    ax[1,1].bar(indices+offset,[r[name]['outside_hull'] for r in windows],width=.34,color=color,label=label)
ax[1,0].axhline(5,color=red,lw=1,label='Unchanged 5 cm guard')
for a in ax[1]:
    a.set_xticks(indices,labels)
    a.set_xlabel('Local factor-window start (original seconds)')
    a.axvspan(2.55,3.45,color=green,alpha=.1)
    a.legend(fontsize=8)
ax[1,0].set(title='C  All nine predeclared local windows',ylabel='Maximum guard-plane error (cm)')
ax[1,1].set(title='D  Every possible factor-ray hit must be covered',ylabel='Ray hits outside the guard hull (of 176)')
ax[1,1].annotate('Only 31–38 s passes both guards',xy=(3,0),xytext=(3.4,12),
    fontsize=8,color=green,arrowprops=dict(arrowstyle='->',color=green))
for a in ax.flat:
    a.grid(axis='y',alpha=.18);a.set_axisbelow(True)
fig.suptitle('Minimum range is not a centre-axis point observation',fontsize=16)
fig.savefig(ROOT/'comparison.png',dpi=160)
svg=ROOT/'comparison.svg'
fig.savefig(svg,metadata={'Date':None})
svg.write_text('\n'.join(line.rstrip() for line in svg.read_text().splitlines())+'\n')
