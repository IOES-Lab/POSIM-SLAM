# SPDX-License-Identifier: Apache-2.0
"""Plot paired post-seal scene-support comparisons, not estimator output."""
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parent


def main():
    full = json.loads((ROOT/'field-forward-audit.json').read_text())['frames']
    subset = json.loads((ROOT/'scene-support-ablation.json').read_text())['frames']
    rows = {(r['index'],r['arm']):r for r in full+subset}
    indices = [27,81,88]
    labels = [('reference_ground','Reference ground only'),
        ('reference_ground_and_target','Reference ground + target'),
        ('fitted_plane_and_reference_target','Fitted plane + same reference target'),
        ('sealed_finite_fit_navigation_pose','Sealed one-prism/plane fit'),
        ('full_scene_true_pose','Full reference scene / interpolated true pose')]
    fig = plt.figure(figsize=(17,8.5),layout='constrained')
    grid = fig.add_gridspec(3,2,height_ratios=[1,.14,.14])
    axes = [fig.add_subplot(grid[0,j]) for j in range(2)]
    x = np.arange(len(indices));width = .14
    colors = ['#7a9cbd','#7fbb8e','#dc985f','#b46c80','#8c7fb5']
    for j,((arm,label),color) in enumerate(zip(labels,colors)):
        delta = [rows[i,arm]['composite_score']-rows[i,'full_scene_navigation_pose']['composite_score'] for i in indices]
        rms = [rows[i,arm]['normalized_residual_rms'] for i in indices]
        offset = (j-2)*width
        axes[0].bar(x+offset,delta,width,color=color,label=label)
        axes[1].bar(x+offset,rms,width,color=color,label=label)
    axes[0].axhline(0,color='#294b45',lw=1.2,label='Full reference scene / navigation pose: zero')
    axes[1].scatter(x,[rows[i,'full_scene_navigation_pose']['normalized_residual_rms'] for i in indices],
                    marker='D',s=50,color='#294b45',zorder=5,label='Full reference scene / navigation pose')
    for ax in axes:
        ax.set(xticks=x,xticklabels=[f'Frame {i}' for i in indices],xlabel='Same original raw pixels within each frame')
        ax.grid(axis='y',alpha=.2);ax.set_axisbelow(True)
    axes[0].set(ylabel='Composite score difference from full scene / navigation pose',
                title='Lower score is better within identical queries')
    axes[1].set(ylabel='Dimensionless normalized-power residual RMS',
                title='Per-frame brightness is profiled, not per-pixel')
    handles,legend = axes[1].get_legend_handles_labels()
    legend_axis = fig.add_subplot(grid[1,:]);legend_axis.axis('off')
    legend_axis.legend(handles,legend,loc='center',ncol=2,fontsize=10)
    fig.suptitle('Assessment-only reference-scene support changes the prediction\n'
        'A fitted flat background can compensate for omitted reflectors; more views alone do not repair that family',fontsize=16)
    caption = fig.add_subplot(grid[2,:]);caption.axis('off')
    caption.text(.5,.5,'No reference mesh or pose enters the sealed geometry fit. Original source grid, query indices, material .001 and operator remain fixed within the support ablations.\n'
        'Removing a surface changes first-hit visibility; a dark opaque surface is not substituted for removal. True pose is interpolated from 40 ms body samples.\n'
        'These are conditional development controls, not calibrated likelihood-ratio tests, hardware qualification or a map acceptance threshold.',
        ha='center',va='center',transform=caption.transAxes,fontsize=10)
    fig.savefig(ROOT/'scene-support-comparison.png',dpi=140,bbox_inches='tight',pad_inches=.12)
    plt.close(fig)


if __name__=='__main__':main()
