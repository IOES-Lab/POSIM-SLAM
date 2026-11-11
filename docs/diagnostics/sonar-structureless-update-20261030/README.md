# Conditional sonar update and identical-clock DVL control

[한국어](README.ko.md)

This is an unqualified numerical/recorded-input diagnostic. No native sonar
update, map admission rule, sensor noise, provider or product default changes.
The [preceding correspondence record](../sonar-pair-correspondence-20261030/README.md)
retains the source images and every candidate. The prepared arrays here make
the new conditional calculation reproducible without WWW-POSIM imports.

## One posterior, new measurements only

Use the fixed finite-object block at 35.50 / 36.01 s. A native replay prefix
retains 108 states, matching the original prefix, and exports one posterior
at 36.01 s. Both pose means, their complete 12 × 12 covariance and their cross
correlation come from this snapshot. The query needs 4,976,768 bytes under the
unchanged 32 MiB limit. The native root has 226 columns/rank; the queried pose
marginal has no free columns. `pair-joint.json` preserves the source digest.

`structureless.py` uses public POSIM-SLAM point-return derivatives in the
snapshot's fixed left-rotation coordinates. It whitens the complete measurement
covariance before eliminating unknown point coordinates with the existing
square-root split. The navigation posterior is counted once; fitted points
supply no independent prior. Seven independent controls cover owner-coordinate
derivatives, all six global geometry null directions, correlated noise and
poses, full-joint least-squares mean/covariance including point back-substitution,
pure-yaw height degeneracy, empty input and invalid/unbounded uncertainty.

Nullspace feature elimination is established practice, described by
[OpenVINS](https://docs.openvins.com/update-null.html). Its
[observability discussion](https://docs.openvins.com/fej.html) also explains the
risk of false information after inconsistent linearization. No OpenVINS code
is copied, imported or bundled. This diagnostic does not certify sequential
FEJ consistency, physical correspondence or a hardware noise model.

## Complete retained outcomes

The unchanged training-cost cutoff is 9, with nine elevation initializations.
All 25/14 original pairs, failed fits, selection masks and ten coherent sampled
modes per variant remain. Alternative modes are not an exhaustive mixture.
The two variants are separate controls; their overlapping source pixels are
never fused together. Range/azimuth sigmas remain 0.0250836117 m /
0.00821331413 rad, a diagonal diagnostic assumption rather than calibrated
cross-echo or hardware uncertainty. Truth and physical boxes enter only the
separate assessment and figure.

| Fixed best mode | Positive-source LK | Contrast-source LK |
| --- | ---: | ---: |
| Training pairs / selected | 25 / 20 | 14 / 13 |
| Relative translation error before → after | 2.131 → 2.149 mm | 2.131 → 1.214 mm |
| Relative rotation error before → after | 0.06131 → 0.06008° | 0.06131 → 0.06032° |
| Linear point back-substitutions inside both fans | 5 / 20 | 2 / 13 |
| Minimum local point position 3σ | 7.465 m | 7.466 m |
| Nonlinear refit surface RMSE | 0.2501 m | 0.1441 m |
| Held-out C compatibility before → refit | 17 → 18 / 20 | 10 → 10 / 13 |

All ten sampled modes are assessed. A small pose correction does not make the
linearized point correction physically valid: most points leave the sonar
fan. Reprofiling the original A/B observations at the corrected poses retains
20/13 bounded fits, but their geometry and uncertainty still do not qualify a
useful map. The refit is a separate bounded nonlinear point calculation, not
the conditional point posterior. Held-out C uses its historical navigation mean
only; its covariance is not mixed with the B posterior. The original gauge
alignment stays fixed, with no pair-specific or trajectory fit.

![Matched field, complete contrast candidates and original sonar/RGB](comparison.png)

The native isometric field reference places BlueROV at its recorded 36.01 s
pose. The candidate views use the same isometric camera, fixed gauge, common
Turbo height range and 64 pt² markers; all 13 selected contrast candidates are
shown. There are zero admitted map points. The original RGB and 15 m sonar
share their origin and 0.35 rad downward pitch; near range is below. RGB has
2 m visibility, particles, caustics and water-surface effects. Only the separate
field-context re-render disables haze/absorption so the physical layout is
visible; its wave phase is not the original exposure. It is never estimator
input. `comparison.json` retains timestamps and display bounds.

## DVL likelihood control on the full original clock

The application replay adds `--dvl-clock-only`: prepare/validate the same raw
DVL samples and retain their state-event times, while supplying no native DVL
packet or calibration. It is mutually exclusive with `--joint-dvl`. Sonar
factor omission no longer incorrectly requires DVL fusion. Gravity startup
uses the original raw IMU; no DVL velocity or pose is substituted for it.

Both full runs retain identical original inputs, frontend bytes, 175 state
timestamps and 54 pressure-optimized states. The 55-packet DVL control matches
every original state mean within 2e−9. The clock-only control has zero DVL
packets, zero DVL-owner receptions and zero DVL-optimized states.

| Offline control | Full-trajectory position RMSE | Maximum |
| --- | ---: | ---: |
| IMU + pressure + DVL, acoustic factors omitted | 0.013608 m | 0.024937 m |
| Identical clock, every DVL factor omitted | 19.223638 m | 41.582245 m |

This isolates the contribution of DVL on this recording. It is neither a
sonar-improvement result nor a physical bottom-lock-loss/absent-hardware test.
The explicit noiseless-simulator bias calibration is shared by both controls;
no hardware initialization guarantee follows. Each replay command exits zero;
the worker's expected `LiveClient.close` termination is SIGTERM (−15), matching
the original control. A private staging failure and an evaluator's incorrect
zero-worker-exit assumption remain with their full logs.

## Reproduction and next gate

From WWW-POSIM, run with NumPy and the public component on `PYTHONPATH`:

```sh
PYTHONPATH=libraries/posim-slam/src python3 scripts/run_checks.py docs/validation/sonar-structureless-update-20261030/test_structureless.py docs/validation/sonar-structureless-update-20261030/verify.py
```

From POSIM-SLAM, use `PYTHONPATH=src` and the corresponding
`docs/diagnostics/sonar-structureless-update-20261030/` paths. The verifier checks
204 conditional-update, all-point and identical-clock assessment comparisons.
`sha256.json` binds every shared file. Recorded library input is `8ea4368`,
with frozen native source `44aa93e` and the same Basalt
`0f3b2b52c807f70ff4e2973ce253c73329eea7bc` worker. No native rebuild occurs.

Original raw sensors remain once at
`/Volumes/ExternalHDD/POSIM/validation-data/posim-slam/sonar-finite-objects-20261030b/complete`.
New full prefix/mode records and DVL replays remain in the neighbouring
`sonar-structureless-update-20261030` and `sonar-dvl-ablation-20261030` directories.
Raw native replay requires that canonical dataset and the recorded SDK/worker;
the prepared numerical verification above does not.

Next use longer persistent tracks with independent later observations and
physical fan support before proposing native rows or XYZ promotion. Compare
sonar benefit during matched-timing DVL loss and without DVL, then return to
representative Orca5 terrain and actual loss/recovery. Do not expand a large
suite, install the fitted six-DOF mean or add height/odometry priors to conceal
this two-view limitation. Product/installed, VRX and later WAM-V survey gates
remain open.
