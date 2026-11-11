# Close inspection with an explicit sonar mount

[한국어](README.ko.md)

This engineering control includes current POSIM main `8c47601c`, #6 and every #31–36 head in separate integration `ffecee9b`. All source PRs remain open and unmerged. Actual loaded sensor/system hashes match the corrected-source receipt. The existing explicit Gazebo loader/render dependency controls remain; the installed SDK is unchanged.

| PR | Head | SLAM input responsibility |
| --- | --- | --- |
| [#31](https://github.com/IOES-Lab/POSIM/pull/31) | `7dc96021` | One sensor instance and the configured initial mount |
| [#32](https://github.com/IOES-Lab/POSIM/pull/32) | `f1055f48` | ROS context lifetime and captured-frame timestamps |
| [#33](https://github.com/IOES-Lab/POSIM/pull/33) | `5f327fd5` | Partial matrix-tile input loads |
| [#34](https://github.com/IOES-Lab/POSIM/pull/34) | `801f8866` | Original-length range transform and floating spectral summation |
| [#35](https://github.com/IOES-Lab/POSIM/pull/35) | `02f6dccd` | Empty returns remain black; raw zero power remains valid |
| [#36](https://github.com/IOES-Lab/POSIM/pull/36) | `fbfd000d` | Physical range spacing, final sample and CPU bin alignment |

## Configuration and actual acquisition

The original [95-frame inspection record](../sonar-inspection-scene-20261031/README.md) stays unchanged. At the safe close position, its 20.1° sonar mount cannot see the first block top in the ideal view calculation. This experiment applies one explicit 45° downward mount through the existing sensor-recipe settings, before model creation. The matching RGB camera and calibration use the resulting physical SDF. Product defaults and saved vehicles are unchanged. Mount **and** operation change together; this is not a mount-only ablation.

- Position: x 6 m, y −1.35 m, depth 1.5 m; headings 0°, −45°, +45°, −45°, +45°, 0°, with six-second holds and return home.
- Actual mission completes; native preview minimum clearance is 1.4325 m under the existing full planning footprint. No tracking margin or safety limit is reduced.
- Sonar: 15 m, 256 beams, 256 vertical rays, 120° × 20°, 2 Hz, 700 kHz model carrier and 29.9 kHz bandwidth. All 150 original frames have 599 × 256 values and the real 15 m endpoint.
- Original acquisition: 376 vehicle RGB, 7,529 raw IMU, 753 pressure, 75 DVL, 151 same-mount reference RGB and 76 isometric frames. All 75 DVL reports have four-beam bottom lock; Ping is physically absent.
- All original raw hashes/grids/stamps pass the private audit; reference RGB differs by at most 10 ms. All four owned process groups exit normally. Reuse 752.6 MB of immutable assets by hardlink; copy 3,983 source bytes. The new raw recording occupies about 121 MB and is retained once externally.

The assessment's broad close-position window contains 78 sonar frames; 76 can geometrically see the known block top. Two unsupported frames during the departure/return turn remain. This verifies physical visibility, not image-feature identity. Simulator poses and scene surfaces never enter SLAM.

![Original same-mount RGB/sonar with the actual field and sensor directions](sensor-comparison.png)

The original RGB retains 2 m optical visibility, underwater lighting, particles and caustics. Orange lines show the sonar's ±10° vertical fan within the reference camera's wider vertical view; both have the same origin/direction and 120° horizontal view. Near is at the bottom of the sonar; axes are metres with explicit polar-cell edges. The native isometric context re-render uses the recorded 28.01 s body pose. It disables haze/absorption to expose the targets while retaining waves, lighting, particles and caustics; their phase differs. The context and reference cameras are assessment-only.

## Complete outcomes

The matched native profiles use the existing unknown-bias initialization, raw IMU/DVL/pressure, identical clocks and unchanged diagnostic noise. The navigation control excludes sonar and RGB. All 223 states are assessed in each profile. Position RMSE is **0.0587 m without sonar points**, versus **0.1792 m with native intensity-point constraints**. Native maps retain all 27 points: surface RMSE 0.152 m, median 0.0639 m, seven beyond 10 cm. No navigation benefit is accepted.

The unchanged [offline association control](../sonar-return-support-20261031/README.md) uses 148 replayable frames and the same 2,554 A-KAZE detections for both methods. Both original startup frames remain in the raw archive. Navigation means are fixed; their uncertainty is not propagated.

| Offline conditional result | Descriptor | Elevation-arc association |
| --- | ---: | ---: |
| Repeated chains / unavailable fits | 170 / 2 | 59 / 0 |
| All emitted and assessed points | 168 | 59 |
| Surface RMSE / median (m) | 0.333 / 0.164 | 0.350 / 0.211 |
| Points beyond 10 cm | 109 / 168 | 38 / 59 |
| Median fixed-pose measurement-only position 3σ (m) | 4.520 | 1.731 |
| Points with that optimistic 3σ ≤10 cm | 1 | 3 |
| Chains / observations surviving the unused +2 s window | 4 / 12 | 3 / 10 |

![Every native and conditional point beside the actual native field](full-cloud-comparison.png)

All 254 points, unavailable fits, nine-seed outcomes and missing/out-of-fan later observations remain. Markers are 64 pt² with a common full-height Turbo scale. Initial yaw/translation alignment uses unit scale. Offline points do not enter the native optimizer. A 10 cm cutoff is a diagnostic count, not a universal marine acceptance standard.

## What the changed operation resolves

Physical coverage improves, but tracked features rarely preserve the informative motion. Descriptor/projective repeated-chain median yaw spans are only 0.34°/1.56°, with median lifetimes 1.49/3.51 s. Only 11/170 and 18/59 chains span at least 20°; only 11/170 and 15/59 do so in the construction window. Vehicle-wide rotation is not the same as multiview support for one fixed reflector.

The 65/257-sample comparison changes raw numeric IDs in 82 frames. Comparing membership by birth stamp/pixel instead of arbitrary IDs identifies **two frames and three detections** with actual membership differences. Both counts are retained; sampled curves still provide no continuous-locus guarantee. Better physical visibility has not established reliable point correspondence, joint uncertainty or useful mapping. Keep these maps unqualified and establish persistent physical return support before another native point-factor change or larger field suite.

Run `python verify.py` with the matching public POSIM-SLAM source and NumPy. It reuses the shared prepared-input replayer with this record's explicit calibration; no OpenCV, SDK or private application import is needed. It checks all histories, all points/seeds/future views, unavailable fits, numeric-ID invariance, original metadata and the retained representative raw frame. Full raw data remains once on ExternalHDD, with its checksum index; the public record does not include every raw image. Scene triangles and their [Orca5 MIT notice](../sonar-inspection-scene-20261031/Orca5-LICENSE.txt) reuse the preceding record after exact array equality. Source/transport tests, useful-map acceptance, installed/browser/platform behavior, DVL-loss benefit, VRX and the later WAM-V survey remain separate gates.
