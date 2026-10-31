"""Optional static scientific plot; no generated sensor or cloud imagery."""
from pathlib import Path
import json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parent
data = json.loads((ROOT/'summary.json').read_text())
cases = data['cases']; x = np.arange(len(cases))
fig, ax = plt.subplots(figsize=(12, 5), layout='constrained')
ax.axvspan(-.5, 2.5, color='#eff3f8', label='First-packet calibration configurations')
for key, label, color, marker in [
    ('continuous_cone', 'Continuous nearest-cone model', '#d35e2c', 's'),
    ('nominal_sampled', 'Unqualified nominal GPU grid', '#7658a7', '^'),
    ('calibrated', 'Frozen unit return directions', '#147eab', 'o')]:
    selected = [(i, c[key]['max_abs_error_m']*1000) for i, c in enumerate(cases) if key in c]
    ax.plot(*zip(*selected), color=color, marker=marker, label=label, linewidth=1.3)
ax.axvline(8.5, color='#6b7280', linestyle='--', linewidth=1)
ax.set_yscale('log'); ax.set_ylim(.0003, 80)
ax.set_xticks(x, [f"{'R' if c['group']=='reference' else 'U'}{c['case']+1}\n"
                 f"{c['height_m']:g} m\n{c['roll_deg']:g}/{c['pitch_deg']:g}°" for c in cases], fontsize=8)
ax.set_ylabel('Maximum absolute range-model residual (mm)')
ax.set_title('Static known-plane DVL controls — all 1,560 beam ranges retained')
ax.grid(axis='y', which='both', alpha=.18)
ax.legend(loc='upper left', fontsize=8)
fig.supxlabel('R: reference geometry (12 training ranges only); U: separately acquired unused orientations\n'
              'Residuals describe this static simulator model, not hardware precision or SLAM/map accuracy.', fontsize=9)
fig.savefig(ROOT/'comparison.png', dpi=150)
plt.close(fig)
