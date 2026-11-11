"""Plot complete selected-pixel moments from the controlled native kernel."""
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

root = Path(__file__).resolve().parent
with np.load(root/'moments.npz', allow_pickle=False) as z:
    mean, covariance = z['mean'], z['covariance']
with np.load(root/'native-output.npz', allow_pickle=False) as z:
    samples = abs(z['selected'].astype(np.complex128))**2
observed = samples.mean(axis=0)
ratio = observed/mean
correlation = covariance/np.sqrt(np.outer(np.diag(covariance), np.diag(covariance)))
empirical = np.cov(samples, rowvar=False)
empirical /= np.sqrt(np.outer(np.diag(empirical), np.diag(empirical)))
fig, axes = plt.subplots(1, 3, figsize=(14, 5.2))
fig.subplots_adjust(left=.06, right=.94, bottom=.24, top=.80, wspace=.35)
axes[0].axhline(1, color='black', linewidth=1)
axes[0].errorbar(np.arange(64), ratio, yerr=1.96/np.sqrt(512), fmt='.', color='#1764b5', capsize=1)
axes[0].set(xlabel='Predeclared pixel index', ylabel='Native sample mean / predicted mean',
            title='All 64 selected pixels; 512 native frames')
for ax, values, title in zip(axes[1:], [correlation, empirical], ['Source conditional correlation', 'Native empirical correlation']):
    im = ax.imshow(values, vmin=0, vmax=1, cmap='viridis', origin='lower')
    ax.set(title=title, xlabel='Pixel index', ylabel='Pixel index')
fig.colorbar(im, ax=axes[1:], label='Power correlation', shrink=.75)
fig.suptitle('Fixed retained range grid → current WGPU kernel: conditional stochastic control', fontsize=13)
fig.text(.01, .025, 'Bars: marginal mean standard errors (±1.96 / √512), not a simultaneous interval or map/pose uncertainty.\n'
         'No new Gazebo acquisition or RGB rendering; all selected samples and full-image aggregates are retained.', fontsize=9)
fig.savefig(root/'native-moments.png', dpi=160, bbox_inches='tight', pad_inches=.15)
