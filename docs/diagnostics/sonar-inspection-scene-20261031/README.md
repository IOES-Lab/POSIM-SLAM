# Physical inspection scene and matched sonar-point controls

[한국어](README.ko.md)

The default BlueROV2 field retains the 60 × 60 m Orca5 seafloor and adds three grounded blocks beside an L-shaped wall. Block heights are 0.45, 0.75 and 0.60 m; the wall is 1.6 m high. One box specification supplies actual collision/visual geometry, browser dimensions and the conservative route-clearance surface. The unchanged physical seafloor remains separate from this clearance grid. The previous weak-structure field record is retained as a regression case.

## Source and acquisition

Separate integration `ffecee9b` includes current main `8c47601`, #6 and every current #31–36 head. [The source review](source-review.json) records their exact identities and ancestry. #31 owns sensor creation/mount pose; #32 owns ROS lifetime/captured-frame stamps; #33 fixes partial matrix tiles; #34 supplies the exact-length transform, frequency aperture and floating weak-return calculation; #35 handles an empty preview; #36 supplies the physical range endpoint. The SLAM consumer preserves zero-return `-inf`, uses the published range vector and keeps latent-point physical bounds. No source PR is merged into #6 or remote main.

The native run records 95 sonar frames (599 ranges × 256 beams, 15 m), 238 vehicle RGB frames, 4,748 IMU, 475 pressure and 47 native/status DVL samples. All DVL samples report four-beam bottom lock; single Ping is physically absent. All original sonar hashes/grids/stamps pass the [95-frame audit](field-audit.json); same-mount reference RGB differs by at most 10 ms. The route completes with minimum planned clearance 1.351 m. All four owned acquisition groups exit normally, and the separate native context render exits 0. The installed SDK, source acoustic model and sensor parameters remain unchanged during this scene comparison.

![Actual sonar, matched RGB and annotated native field](sensor-comparison.png)

Top: recorded same-mount RGB and actual sonar. The RGB keeps 2 m visibility, underwater light, particles and seabed caustics. Sonar axes are distances, with near at the bottom. The vehicle RGB camera looks horizontally; the sonar and its comparison camera pitch down 20.1°. Bottom: native field re-render at the recorded 28.01 s body pose with haze/absorption off to expose geometry; waves, light, particles and caustics remain, with a different wave/particle phase. This clear overview is assessment-only.

## Matched native replays

Three default-profile runs use identical original timestamps, unknown-bias initialization, IMU, bottom-locked DVL and pressure. They retain 139 states each over 2.5–48.5 s, and use no RGB input. Two startup sonar intervals are retained raw but excluded for the existing one-second IMU startup history. Simulator poses/scene geometry never enter these estimators. Trajectory assessment aligns only initial yaw and translation, with unit scale and gravity tilt retained.

| Mode | Position RMSE | Rotation RMSE | Complete point output | Surface RMSE | Beyond 10 cm |
| --- | ---: | ---: | ---: | ---: | ---: |
| IMU + DVL + pressure | 0.0297 m | 0.671° | — | — | — |
| + intensity-image point tracks | 0.1175 m | 1.466° | 64 | 0.431 m | 40 / 64 |
| + echo/shadow-boundary point tracks | 0.0549 m | 1.285° | 32 | 0.826 m | 27 / 32 |

![All emitted points beside the actual native field](full-cloud-comparison.png)

Both clouds show every retained point, including outliers, using 64 pt² markers and one complete Turbo height range. The field and cloud views share the isometric direction, with independent framing. Grey surfaces are assessment references. Complete original PLY/JSON exports and every-point errors are included. The 10 cm counts describe this record; they are not a universal marine-SLAM acceptance standard. Neither point method qualifies as a useful map or navigation aid here. A separate model-known bias navigation control reaches 0.0181 m position RMSE; it is not the matched point-profile baseline. Native workers close through the existing client SIGTERM path (child -15, replay Python 0), without SIGKILL.

## Exact-pose diagnostic

An offline control freezes the same observations at each emitted point's last accepted native history. It substitutes exact assessment poses and reruns the existing nine elevation seeds with twelve bounded iterations, retaining every seed outcome. It selects the lowest measurement cost before consulting any scene surface. This is an optimistic diagnostic; no truth enters native replay.

| Same observations, exact poses | Points assessed | Surface RMSE | Beyond 10 cm | Median conditional position 3σ |
| --- | ---: | ---: | ---: | ---: |
| Intensity tracks | 64 / 64 | 0.395 m | 40 / 64 | 0.826 m |
| Echo/shadow-boundary tracks | 32 / 32 | 0.796 m | 29 / 32 | 0.875 m |

The conditional 3σ uses measurement-only point information with poses fixed. It omits shared pose uncertainty and unresolved physical association. It is not joint uncertainty or map acceptance. Median fitting residuals are 0.750 and 1.104 in the assumed whitened measurement units, despite large surface errors.

For observations unused by each fit and at least two seconds later, intensity tracks retain 13/64 chains: 80 observations, 56 inside predicted geometry and 24 outside. Range RMSE is 0.124 m over the 56 geometrically valid predictions. Echo-boundary tracks retain 2/32 chains: three observations, two valid and one outside; valid range RMSE is 0.0905 m. Tracks without later observations remain in the denominator. These small surviving subsets do not demonstrate reliable prediction or data association.

The measured failure persists after removing pose error. It motivates handling ambiguous elevation and view-dependent acoustic support before adding more fixed XYZ constraints. This is consistent with the limitations discussed in primary work on [under-constrained sonar landmarks](https://www.cs.cmu.edu/~kaess/pub/Westman18icra.html) and [degeneracy-aware sonar constraints](https://www.cs.cmu.edu/~kaess/pub/Westman20joe.html); this record does not implement or validate those papers' complete methods. Next compare range/azimuth support and an explicit unresolved-elevation representation on this same raw input. Physical DVL-loss/absence, useful geometry, general initialization, installed/platform operation, VRX and WAM-V survey gates remain open.

## Reproduction and scope

From the public POSIM-SLAM source checkout, install the diagnostic dependencies and run `python docs/diagnostics/sonar-inspection-scene-20261031/verify.py`. In the application, initialize the selected POSIM-SLAM submodule and run the mirrored `docs/validation/.../verify.py`. Verification recomputes all trajectories, every emitted point's triangle-surface distance, retained exact-pose costs and unused-observation predictions, as well as all published hashes and the representative original raw sample. It does not reacquire sensors or verify raw files retained only externally. `plot_cloud.py` redraws the complete-cloud figure from these bounded data.

Full raw acquisition, failed staging/profile controls and source/binary traces remain once externally. This record contains no private application implementation, provider ABI or installed SDK. [Source identity](source-identity.json), complete measurement outputs and separate acquisition/replay/assessment scopes remain explicit. Orca5-derived geometry/images retain the [upstream MIT notice](Orca5-LICENSE.txt); original library code remains Apache-2.0.
