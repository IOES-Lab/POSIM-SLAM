from pathlib import Path
import hashlib,json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
base=Path.cwd()/'.local/sonar-low-altitude-15m-20261029';data=json.loads((base/'dvl-sonar-prediction.json').read_text())
rows=[r for r in data['rows'] if r['residual_m'] is not None];errors=np.array([r['residual_m'] for r in rows]);times=np.array([r['sonar_stamp_ns']/1e9 for r in rows])
fig,axes=plt.subplots(1,2,figsize=(13,5.6),layout='constrained')
axes[0].scatter(times,errors,s=12,alpha=.6,c=np.where(errors<-.1,'#c2410c',np.where(errors>.1,'#7c3aed','#2563eb')))
axes[0].axhline(0,c='#334155',lw=1);axes[0].axhspan(-.1,.1,color='#93c5fd',alpha=.25)
axes[0].set(xlabel='Original sonar exposure (s)',ylabel='Observed onset − predicted geometric range (m)',title='All 664 preselected predictions; no error-based removal')
absolute=np.sort(abs(errors));axes[1].plot(absolute,np.arange(1,len(errors)+1)/len(errors),color='#1d4ed8',lw=2)
axes[1].set_xscale('symlog',linthresh=.005);axes[1].set_xlim(0,absolute.max()*1.05);axes[1].set_ylim(0,1.01)
axes[1].axvline(.1,c='#64748b',ls='--');axes[1].set(xlabel='Absolute range error (m; symlog)',ylabel='Cumulative fraction',title='Complete error distribution, including the 2.68 m tail')
for ax in axes:ax.grid(alpha=.2)
fig.suptitle('DVL-supported surface predicts FLS onsets • independent IMU/DVL/pressure poses',fontsize=14)
fig.supxlabel('Median 1.59 cm; p90 5.52 cm; maximum 2.676 m. All 624 errors ≤0.10 m and all 40 larger errors remain.\n'
 'Patch/support selection uses predicted geometry without using onset range. Pose replay omits every sonar factor.\n'
 'Conditional geometric first returns; reflectivity, nearer occluders and intensity-threshold effects remain unresolved. No map acceptance.',fontsize=9)
path=base/'prediction-range-errors.png';fig.savefig(path,dpi=140);plt.close(fig)
(base/'prediction-range-errors.json').write_text(json.dumps(dict(points=len(rows),all_predictions_retained=True,within_010_m=int((abs(errors)<=.1).sum()),
 negative_beyond_010_m=int((errors<-.1).sum()),positive_beyond_010_m=int((errors>.1).sum()),map_qualified=False,
 figure_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),source_sha256=hashlib.sha256((base/'dvl-sonar-prediction.json').read_bytes()).hexdigest()),indent=2)+'\n')
print('Rendered complete prediction residuals, including all large errors')
