# Native conditional power and reflectivity scores

[한국어](README.ko.md)

![All selected native moments and power correlations](native-moments.png)

The current POSIM WGPU kernel (`ffecee9b`, #6/#31–36 unmerged integration)
runs 512 independent frame counters on a retained renderer-derived 256×256
range grid. Source range-derived normals replace the old RGB8 approximation.
The fixed 15 m, 599-bin input preserves every ray. No Gazebo world, SDK copy
or new field acquisition is needed.

Before execution, the protocol fixes the 64 pixel queries, native seed/frame
counters and 1,024 correlated complex-Gaussian reference repetitions. It fixes
the 99.5th percentile for each of two statistics, without fitting a gain.
The maximum standardized mean error is **2.215**, below **3.920**; normalized
covariance error is **0.1989**, below **0.2774**. These simulated sampling
thresholds qualify conditional moments on these supplied inputs. They do not
calibrate hardware, rasterization, inferred geometry or a complete image law.

`native-output.npz` retains every selected complex value for all 512 counters,
complete image power/squared-power sums and three predeclared full-complex
outputs. `inputs.npz` retains the complete ranges, normals, mixing and window.
These 64 pixels are statistical queries, not a point-cloud display limit.

## Common reflectivity and correlated evidence

`sonar_power_evidence.profile_power_evidence` profiles a positive common gain
as the average observed/predicted power ratio. The exponential marginal mean
score provides the objective; sensitivity and score covariance stay separate.
Full power correlation enters the conditional sandwich covariance. The method
does not invert a nearly singular pixel covariance or replace it with diagonal
noise. SVD returns identifiable and nullspace bases separately; an unknown
gain direction does not become a zero-variance geometry estimate.

This uses the established composite-score/sandwich principle described by
[Varin, Reid and Firth (2011), sections 2.2–2.4](https://www3.stat.sinica.edu.tw/statistica/oldpdf/A21n11.pdf).
The source-specific implementation conditions on the supplied mean, covariance
and optional log-mean Jacobian. It is not the full dependent-image likelihood
or joint navigation/map uncertainty. Known geometry/material grouping and
independent-frame averaging assumptions must remain explicit.

Seven unit controls cover profile gradients, duplicated correlated pixels,
gain nullspaces, power dynamic range, excluded support, invalid covariance and
an independent nonlinear ratio estimator. Replicating three pixels eight times
with their correlations leaves covariance unchanged; pretending independence
reduces it falsely by eight. A separate 5,000-trial ratio-estimator control
matches the conditional covariance. No native SLAM factor is enabled.

## Original translated-image baseline

Three previously inspected raw frames (48/96/144) use causal DVL-plane means
and navigation means solely to construct a uniform-ground hypothesis. Scene,
truth, RGB and map points are excluded. Full selected raw powers, predictions,
covariances and residuals remain. Profiled gains are **3.148/1.364/1.923**;
relative-residual RMS is **3.901/1.681/3.302**. These are dimensionless baseline
statistics, not metre errors or calibrated probabilities. A common brightness
change does not explain all observed shape. Terrain extrapolation, objects,
material differences and uncertain plane/pose means remain unresolved.

Next fit and compare finite-ground/occluder hypotheses on the retained views,
using this score and explicit nuisance parameters. Keep alternative visibility
branches and source correlations. These frames are development data, including
any temporal split. Independent scenes, joint uncertainty and actual DVL-loss/
absence benefit remain required before map or native-factor admission.

```sh
python docs/diagnostics/sonar-native-power-moments-20261031/verify.py
```

The portable verifier uses saved native output and needs no GPU. `control.py`
accepts `--binary`, `--input` and `--output` to reexecute the source-bound native
operator on Apple Silicon; `--verify` performs the mathematical reproduction.
The selected binary SHA and resolved-symbol identity are checked during
acquisition. The unchanged renderer-derived input is a source control, not an
estimator observation. All original maps and failure records remain.
