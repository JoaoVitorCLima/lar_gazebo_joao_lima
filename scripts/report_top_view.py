#!/usr/bin/env python3
"""Generate metrics and figures from real synchronized measurements."""
import argparse
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

p = argparse.ArgumentParser()
p.add_argument('csv', type=Path)
a = p.parse_args()
d = np.atleast_1d(np.genfromtxt(a.csv, delimiter=',', names=True))
if len(d) < 2:
    raise SystemExit('At least two paired measurements required')
metrics = {'samples': len(d), 'duration_s': float(d['time'][-1]-d['time'][0])}
for name, values in [('position_m', d['error_position_m']), ('yaw_deg', np.rad2deg(d['error_yaw_rad']))]:
    metrics[name] = {'rmse': float(np.sqrt(np.mean(values**2))), 'mae': float(np.mean(abs(values))),
                     'max_abs': float(np.max(abs(values))), 'p95_abs': float(np.percentile(abs(values), 95))}
(a.csv.parent/'metrics.json').write_text(json.dumps(metrics, indent=2)+'\n')
fig, ax = plt.subplots(1, 3, figsize=(15, 4))
ax[0].plot(d['gt_x'], d['gt_y'], label='Gazebo'); ax[0].plot(d['x'], d['y'], label='AprilTag')
ax[0].axis('equal'); ax[0].set(xlabel='x [m]', ylabel='y [m]'); ax[0].legend()
t = d['time']-d['time'][0]
ax[1].plot(t, d['error_position_m']); ax[1].set(xlabel='tempo [s]', ylabel='erro de posição [m]')
ax[2].plot(t, np.rad2deg(d['error_yaw_rad'])); ax[2].set(xlabel='tempo [s]', ylabel='erro de yaw [graus]')
fig.tight_layout(); fig.savefig(a.csv.parent/'errors.png', dpi=160)
print(json.dumps(metrics, indent=2))
