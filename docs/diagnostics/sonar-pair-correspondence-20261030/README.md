# Sonar correspondence and finite-object control

[한국어](README.ko.md)

These diagnostics compare the existing wall/flat-floor scene with a second
physical acquisition containing three finite boxes. Both use the same authored
mission, sensor calibration and extraction settings. Wave phase, initial timing
and realized motion differ. This is not an identical-input method ablation.
The combination is sonar + raw IMU + pressure + DVL, without Ping or camera
input to navigation. Production defaults and native code are unchanged.

## Complete observations

The second run completes 12.54 m with 49.83° yaw span, retaining 111 sonar,
278 RGB, 5,556 IMU, 56 DVL and 555 pressure samples. All DVL samples are valid.
Callbacks have no recorded errors; the supervisor exits 0 without escalation,
and all four owned process groups are empty. The acoustic-free calibrated
navigation replay retains all 175 states, with 1.36 cm trajectory RMSE.
This is navigation evidence, not a useful sonar map.

The first finite-box layout is retained as a setup failure. It overlaps the
unchanged 2.5 m planning radius and is rejected before mission start. The
corrected fixture passes the actual planner with 1.20 m minimum clearance and
the separate 0.5 m body-envelope check. Sensor range remains 15 m, with
256 horizontal beams, 256 vertical rays and 120° × 20° field of view.

## Third-view compatibility

Features are extracted without poses or truth. LK retains the existing native
OpenCV 4.13 preprocessing and forward/backward test. A-KAZE uses OpenCV 4.13.0
defaults and global mutual matching with a 0.8 nearest-neighbour ratio.
Positive-source/contrast masks screen endpoints after descriptor matching.

Two-view range/bearing fitting uses historical calibrated navigation means,
nine elevation seeds and unchanged 0.0250836 m / 0.00821331 rad scales. A third
image is excluded from fitting and candidate selection. Lost third-view tracks
remain in the denominator. Counts below are compatible/training-selected;
the cost cutoff of 9 is a diagnostic convention, not a calibrated probability.

| Frontend | Walls/floor | Finite boxes added |
| --- | ---: | ---: |
| LK, raw | 549/1,746 | 719/2,057 |
| LK, positive source | 135/383 | 190/478 |
| LK, source contrast | 49/113 | 67/134 |
| A-KAZE, raw | 31/132 | 38/125 |
| A-KAZE, positive source | 13/56 | 19/51 |
| A-KAZE, source contrast | 2/26 | 8/22 |

For contrast-LK, the last column includes 87 recovered validation tracks:
67/87 are compatible, while all 134 training-selected candidates remain in the
reported denominator. Compatibility is optimistic over sampled training modes
and excludes pose covariance. It does not prove physical point identity.
Lost tracks and outliers need explicit handling; 100% retention is not an
acceptance requirement. No new pose constraint or 3D map is installed here.

## Matched sensor and field views

![Actual RGB and two sonar frames with an isometric field reference](observations.png)

RGB and sonar share origin and 20.1° downward pitch. RGB is at 35.00 s;
the first sonar is at 35.01 s. Orange outlines the ±10° sonar elevation fan.
Numbered circles show all eight contrast-LK pairs for this fixed-time example;
they are image observations, not map points. The sonar display uses distance,
with near at the bottom and starboard on the right.

RGB uses 1% transmission at 2 m, particles, caustics and waves. The field
reference re-renders the recorded body pose with haze/absorption disabled,
retaining other effects; its wave/particle phase differs. Authored object and
vehicle annotations are assessment-only. The actual foggy field RGB is retained
in the original record. Tracking overlays use the common metric projection.

## Reproduction and next gate

From the independent POSIM-SLAM checkout, using Python 3.10+ and NumPy:

```sh
PYTHONPATH=src python3 docs/diagnostics/sonar-pair-correspondence-20261030/test_pair_geometry.py
PYTHONPATH=src python3 docs/diagnostics/sonar-pair-correspondence-20261030/verify.py
```

The verifier refits every prepared match and all nine modes, checks both full
summaries, and retains missing-pose classifications. `--output NEW_DIRECTORY`
exports every recomputed mode. The five independent numeric controls include
pure-yaw height ambiguity, infeasible measurements and constrained-boundary
stationarity. Prepared data reproduces fitting and held-out predictions;
raw-image extraction and native replay require the canonical recordings.
Raw inputs and the complete original mode outputs are retained once externally.
Private application code is not part of this independent diagnostic.

Next test shared geometric consensus and observable relative-motion information
before delayed XYZ promotion. This follows established
[under-constrained-landmark](https://www.cs.cmu.edu/~kaess/pub/Westman18icra.html)
and [degeneracy-aware sonar](https://www.cs.cmu.edu/~kaess/pub/Westman20joe.html)
methods. The current comparison does not demonstrate those methods, navigation
benefit, physical correspondence or research novelty. Representative Orca5
terrain, low visibility, DVL loss/recovery and absent-DVL operation remain gates.

[Previous source-support and joint-uncertainty record](../sonar-source-support-20261030/README.md)
retains the separate XYZ admission failures.
