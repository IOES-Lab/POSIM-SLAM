"""Plot recorded returns and actual native acknowledgements; no scene inputs."""
from pathlib import Path
import json
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

ROOT = Path(__file__).resolve().parent
raw = json.loads((ROOT / 'live-wgpu.json').read_text())['all_raw_frames']
native = json.loads((ROOT / 'native-control-final.json').read_text())['processed_sonar']
ack = json.loads((ROOT / 'native-acknowledgements.json').read_text())
fig, axes = plt.subplots(3, 1, figsize=(10, 6), sharex=True, layout='constrained')
x = np.array([row['stamp_ns'] for row in raw]) / 1e9
axes[0].step(x, [row['has_returns'] for row in raw], where='mid', color='#0072b2')
axes[0].set(yticks=[0, 1], yticklabels=['No return', 'Return'], title='192 actual WGPU sonar frames; original sensor time')
axes[1].plot([row['stamp_ns'] / 1e9 for row in native], [row['observations'] for row in native], color='#e69f00')
axes[1].set_ylabel('Image candidates')
t = np.array([row['stamp_ns'] for row in ack]) / 1e9
axes[2].plot(t, np.cumsum([row['dvl'] for row in ack]), label='DVL', color='#009e73')
axes[2].plot(t, np.cumsum([row['pressure'] for row in ack]), label='Pressure', color='#cc79a7', linestyle='--')
axes[2].set(xlabel='Simulation time (s)', ylabel='Optimized states')
axes[2].legend(loc='upper left')
for axis in axes:
    axis.grid(alpha=.2)
fig.suptitle('Input-continuity control: stationary synthetic IMU / DVL / pressure; no qualified map', fontsize=11)
fig.savefig(ROOT / 'continuity.png', dpi=160)
plt.close(fig)
