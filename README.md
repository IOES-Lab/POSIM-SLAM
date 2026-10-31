# POSIM-SLAM

`delayed_surface_update` prepares correlated late-range rows with sequential
block elimination. It updates retiring-state conditionals and retains live-state
uncertainty without freezing recovered poses. The optional native
`SurfaceMarginalOwner` installs new live rows, conditional replacements and the
original-reading ledger together, while active IMU/DVL/pressure factors retain
their own observations. It keeps unknown planes through optimization and state
retirement. `native_surface.prepare_native_ranges` prepares only new rows from
a native snapshot; `LiveClient` queues their installation on the estimator
thread. Snapshot identity, original-reading reuse and explicit archive expiry
are checked by the native owner. This optional transport is off by default;
physical correspondence, calibrated uncertainty and useful sonar maps still
require qualification. [Streaming API and recorded controls](docs/diagnostics/native-surface-transport-20261029/README.md),
[native owner API](docs/diagnostics/native-surface-owner-20261029/README.md),
[reference method](docs/diagnostics/delayed-surface-update-20261029/README.md).

`native_surface_ranges` forms joint original cone/fan rows at native-owned
coordinates. It initializes new unknown planes with the actual correlated
factor rows and rechecks their complete support before preparing an update.
Support-only range seeds add no likelihood. Read-only inspection preserves
unbounded plane modes and reevaluates the nonlinear range equations.
[Original-input initialization and rejected correspondence](docs/diagnostics/native-surface-ranges-20261029/README.md).

`NativeSurfaceSnapshot.joint_marginal` reads the same native owner's undamped
information and propagates live-state uncertainty through shared retiring-state
conditionals. Householder QR and rank-aware SVD retain pose/plane/pressure cross
terms and explicit unbounded directions. The result is a local approximation
with frozen robust weights and declared sensor noise.
[Joint uncertainty and original-input checks](docs/diagnostics/native-surface-information-20261029/README.md).

The optional native surface-owner profile accepts `InitialImuBiasCalibration`
for its first IMU state. Full covariance supplies finite prior rows; zero
variance modes use exact affine support through optimization, retirement and
delayed updates. IMU noise and later bias random walks remain separate. A
matching worker must acknowledge this calibration.
[Calibration input, original replay and limits](docs/diagnostics/native-imu-bias-calibration-20261029/README.md).

[Wall/corner geometry control](docs/diagnostics/sonar-wall-corner-20261030/README.md) compares complete navigation and acoustic outputs on one moving recording, with matched RGB/sonar and identically projected field/cloud views.

[Sonar input diagnostics](docs/diagnostics/sonar-pr31-36-20261030/README.md) check empty-frame recovery, the maximum-range grid and synchronized RGB/sonar input on Orca5 terrain. Saved complex operator outputs have an independent NumPy verifier.

[Inspection-scene point controls](docs/diagnostics/sonar-inspection-scene-20261031/README.md) retain full clouds, matched native RGB and exact-pose comparisons using the corrected #31–36 integration. Point-map usefulness remains unqualified.

[Projective association and motion design](docs/diagnostics/sonar-return-support-20261031/README.md) compare the same detections with descriptor and full-elevation search, retaining every conditional point and unresolved observability.

[Close-view support](docs/diagnostics/sonar-close-view-20261031/README.md) separates actual target visibility from persistent feature correspondence and preserves complete unqualified native and offline maps.

[Conditional shadow geometry](docs/diagnostics/sonar-shadow-support-20261031/README.md) compares angular and physical-plane backgrounds, preserving every observation hypothesis and the unresolved plane/pose uncertainty.

[Known-plane DVL calibration](docs/diagnostics/dvl-plane-calibration-20261031/README.md) separates conditional return-direction means from unknown range uncertainty and retains all static forward-model controls.

[Translated-view control](docs/diagnostics/sonar-translated-view-20261031/README.md) retains the measured 62.87° viewing baseline, matched sensors and every failed-map point; wider motion alone does not qualify the existing point model.

[Translated shadow measurements](docs/diagnostics/sonar-translated-shadow-20261031/README.md) retain every conditional region observation and supplied-edge/mean-image controls. Intensity boundaries do not certify corresponding physical edges.

`sonar_input` preserves zero-power `-inf` bins and validates the broadband source's
physical range grid from bandwidth and sound speed. Feature candidates require
measured echo support; image working values never replace raw observations.
[Input contract and native continuity controls](docs/diagnostics/sonar-input-contract-20261030/README.md).

`sonar_range_response.PosimRangeResponse` evaluates the source-specific frequency
aperture and conditional diffuse-return power moments. Explicit scattering
geometry, variances and beam mixing preserve dependence between range/beam
pixels. [Assumptions and calibration controls](docs/diagnostics/sonar-range-response-20261030/README.md).

`sonar_source_surface.SourceSurfaceObservation` evaluates supplied finite
surfaces in the sonar frame, retaining opaque first-hit visibility, POSIM's
range-derived ray normals and correlated power moments. Geometry and
reflectivity remain caller-supplied hypotheses. [Source and numerical controls](docs/diagnostics/sonar-source-local-power-20261031/README.md).

[Conditional sonar-row diagnostics](docs/diagnostics/sonar-structureless-update-20261030/README.md)
provide a public numerical reproduction of single-posterior conditioning,
retained weak-height modes and identical-clock DVL omission controls.

`BeamPhaseBounds.query_image()` bounds source magnitudes across a raw acoustic
image with at most 8 MiB of multiplication temporary storage. It retains unknown
phase/elevation and does not turn source support into independent 3D points.
[Separated-view geometry and complete candidate comparison](docs/diagnostics/sonar-source-support-20261030/README.md)
includes joint pose uncertainty, original-pixel ownership and retained failures.

`retired_pose_archive` jointly recovers old poses from native Gaussian
conditionals, preserving shared noise and unbounded directions. It is a read-only
diagnostic that conditions on known live coordinates; capture is off by default.
[Delayed-state recovery and its covariance limits](docs/diagnostics/retired-pose-conditionals-20261029/README.md).

`surface_prior` retains joint pose, unknown-plane and pressure-datum information
when source states retire. It preserves cross terms and fixed owner references,
with a bounded original-range ledger. The native owner bridge reads actual
Basalt coordinates. [Delayed-update controls and remaining live-integration requirements](docs/diagnostics/persistent-surface-prior-20261029/README.md).

`range_surface_joint` and the native `makeOwnedRangeSurfaces` adapter keep
per-beam DVL cones, sonar fans, current poses and unknown planes in one
correlated block. Original range keys identify every factor and support source.
They require explicit surface hypotheses and uncertainty; live surface fusion
is not enabled. [Geometry, native-owner checks and retention limits](docs/diagnostics/joint-surface-ranges-20261029/README.md).

`dvl_surface` provides independent finite-cone/local-plane range residuals,
pose/plane Jacobians, full covariance whitening and local support checks.
It requires an associated plane and explicit uncertainty; the application still
fuses DVL velocity. [Model, controlled measurements and limits](docs/diagnostics/dvl-cone-model-20261029/README.md).

`sonar_plane_intersection` retains every elevation branch allowed by measured
range/azimuth and a supplied plane. [Moving inputs and overlap diagnosis](docs/diagnostics/dvl-moving-overlap-20261029/README.md)
describe the correspondence and spatial-support requirements.

`evaluate_dvl_sonar_prediction.py` predicts FLS geometric ranges from DVL
patches and poses that omit acoustic factors. It selects support without using
observed onset ranges. [Inputs, complete errors and data availability](docs/diagnostics/sonar-low-altitude-15m-20261029/README.md).

[Native VIO duration and mode lifecycle](docs/validation/native-vio-lifecycle-20261028/README.md) preserves complete long-run maps, sensor-combination transitions and independent navigation receipts.

[Measured startup scoring controls](docs/validation/seed-scoring-20261028/README.md) preserve sensor-only numerical comparisons and complete source-bound recovery maps.

[Status publication and VIO input recovery / 상태 발행과 VIO 입력 복구](docs/validation/status-publication-vio-20261028/README.md) retains six complete maps, matched RGB/cloud overviews and actual GUI switching evidence.

[Default-budget reference tracking](docs/validation/default-reference-vio-20261028/README.md) compares equal-input native frontends and measured application ROS/managed maps.

[Measured pressure and IMU-delivery comparisons](docs/validation/pressure-event-imu-20261028/README.md) retain full maps, default-budget failures and source-hashed recovery evidence.

[IMU delivery at state events / 상태 시각에 맞춘 IMU 전송](docs/diagnostics/event-imu.md) keeps future raw samples in bounded history while a camera or sonar state awaits its native acknowledgement.

[Static visual-map correspondence / 시각 지도의 고정 특징 대응](docs/diagnostics/visual-static-tracks.md)
describes the physical-identity assumptions behind visual track and map checks.
시각 트랙과 지도 검사의 물리적 동일성 가정을 설명합니다.

[Visual-map information contract / 시각 지도 정보 계약](docs/diagnostics/visual-map-quality.md)
provides an independent native request/report interface for world-position
uncertainty and retained-point reprojection checks.
세계 좌표 위치의 불확실성과 저장점 재투영 검사를 위한 독립적인 네이티브 요청·응답
인터페이스를 제공합니다.

[Camera/IMU exposure timing / 영상·IMU 시각 동기화](docs/diagnostics/image-imu-synchronization.md)
preserves original-time images until received IMU samples cover each exposure.
원본 시각을 유지한 영상을 수신 IMU가 해당 노출 시각을 덮을 때까지 보관합니다.

The camera-free `SonarSession` requires an explicit pressure-enabled native calibration by default. Explicit pressure-off configurations are research ablations. The application requires raw IMU and pressure depth for sonar SLAM, with optional DVL. It checks recent acknowledged pressure fusion separately from sensor reception and waits for valid pressure before worker startup. Multiple-view tracks constrain shared point hypotheses; pressure constrains vehicle depth, not seabed distance. Repeated observations from a single pose do not resolve elevation, and physical correspondence/map accuracy remain unqualified.

`SensorEvents.advance` records a validated but intentionally omitted observation without creating a factor or repeating a measurement. It keeps input ordering independent of factor rate limits. Shared auxiliary acknowledgement and freshness checks distinguish received measurements from recent optimization on both sensor and wall clocks.
[Mixed camera/sonar pressure input and same-view maps](docs/diagnostics/mixed-sonar-pressure.md).

[한국어](README.ko.md) · [Third-party notices](THIRD_PARTY.md)

> This project is developed with assistance from large language models (LLMs).

Experimental camera-free local Ping planes and their matched RGB/point-cloud controls are described in [terrain Ping diagnostics](docs/diagnostics/terrain-ping.md). This mode is off by default; reliable terrain reconstruction remains unqualified. [Acoustic-model rerun and same-view map](docs/diagnostics/sonar-normal-rerun.md).
[Camera-free echo boundaries and vertical survey comparisons](docs/diagnostics/sonar-causal-depth.md) document the optional raw-sonar frontend and its map limitations.
[Camera-free streaming diagnostics](docs/diagnostics/sonar-streaming.md) cover the independent session owner, ROS replay recovery and terrain/visibility qualification boundaries.
[Ground-entry surface mapping](docs/diagnostics/sonar-ground-entry.md) fits conditional seabed observations from raw sonar with an optional single-Ping check; fixed-pose experiments include a retained failure and do not qualify terrain-aided SLAM.
The same guide documents the correlated native range/plane factor and its solver checks; live surface fusion is not enabled.

Independent sensor estimation components used by POSIM applications. Python
algorithms accept measured sensor arrays and calibrated transforms. WWW-POSIM
owns sessions, vehicle composition, runtime supervision, UI and map downloads.

Version **0.1.0** includes the following components. This version is a package
version; acoustic/DVL fusion retains its algorithm version **0.2.1**.

| Component | Method and current scope |
|---|---|
| Visual provider | Stella VSLAM 0.7.0 monocular ORB features and g2o/Eigen optimization; ROS 2 adapter and exact dependency pins supplied; arbitrary scale; empty vocabulary configuration has no qualified loop closure/relocalization |
| Visual + single Ping | Robust local-plane fit and synchronized range with mount/bias compensation; stable median metric scale; requires a suitable downward view and locally planar seabed |
| Visual + DVL | Bottom-locked velocity integration with calibrated axes and rotating lever arm; metric scale calibration, not joint drift correction |
| Acoustic frontend | OpenCV Shi–Tomasi corners and forward/backward pyramidal Lucas–Kanade flow; robust 2D SVD/RANSAC registration in the sonar fan plane |
| Offline sonar regions | NumPy median/quantile intensity segmentation, analytic plane/fan bounds and censored shadow intervals; experimental candidates without 3D landmarks; [method and limits](docs/diagnostics/sonar-surface-research.md) |
| Offline acoustic magnitude bounds | Calibrated complex beam response and polygon inequalities; optional ground-onset screening; [native replay and limits](docs/diagnostics/sonar-phase-ground.md); [comparisons and limits](docs/diagnostics/sonar-phase-bounds.md) |
| Offline terrain support | Conditional onset curves, multi-frame local PCA planes and convex-hull support; Huber point-to-plane ICP probes remain outside the estimator; [comparisons and limits](docs/diagnostics/sonar-terrain-support.md) |
| Offline finite-surface observation | NumPy triangle BVH, opaque first-hit mean power and reflectivity/local vertex adjoints; conditional geometry, without runtime fusion; [model and limits](docs/diagnostics/sonar-finite-surfaces.md) |
| Acoustic + DVL | Bracketed velocity integration for translation, acoustic yaw/features and displacement consistency gate; planar experimental odometry; elevation unresolved |
| Surface filter | Pressure/orientation-derived mean-water plane and calibrated metric-map clipping; does not classify individual surface echoes |
| Inertial core | Raw gyro/accelerometer midpoint SO(3) preintegration, bias Jacobian and 15-state error covariance; preparation code, not a complete fused estimator |
| Experimental inertial filter | Algorithm 0.1.0: 15-state right-error EKF, raw-IMU prediction, mounted DVL velocity, calibrated pressure height and correlated cloned planar relative-pose updates; explicit initialization/calibration; streaming IMU/DVL session owner |

The native Basalt camera path jointly fuses raw IMU and visual observations; optional single-Ping factors participate in the same optimization and marginalization. Stella-only visual and planar acoustic/DVL paths remain separate from this VIO path.
No estimator accepts simulator pose or ideal orientation as a localization
measurement. Sonar intensity features are not classified 3D obstacles.

Ping scale support requires every ray intersection inside the observed plane hull and within 5% of plane distance of a landmark. The `ping_factor` module supplies pose/plane range Jacobians and a supported-patch wrapper for joint optimization; the native worker implements the corresponding joint factors in `native/basalt/ping_factors.hpp` and `ping_linearization.hpp`. `ping_joint` provides a bounded reference linearization and correlated plane/landmark elimination, with explicit range and surface noise.

## Joint fusion building blocks

See the [paired point-cloud/RGB comparison protocol](docs/comparison.md) for
qualifying and documenting each completed sensor combination.

`sonar_triangulation.point_hypotheses` initializes conditional point candidates
from associated range/azimuth tracks and fixed estimated poses, using multiple
elevation seeds. It keeps alternative solutions and reports cost, local rank and
termination reason. Pose uncertainty and physical return association are not
resolved; the output is an initialization diagnostic, not an accepted map.
Diagnostics include gradient, proposed step, range/FOV margins and invalid trial
counts. `no_feasible_trial` means every tested step left the valid geometry;
it does not prove that no constrained solution exists or that the track is wrong.
Point updates use an active-set least-squares step with linearized authored FOV
and range bounds, followed by a nonlinear feasibility check. Boundary convergence
is reported as `constrained_stationary`; it does not certify an echo association
or acceptable residual. No FOV expansion or artificial elevation prior is used.
`heldout_residuals` predicts observations excluded from point fitting, without
refitting. It reports range/azimuth residuals in physical units and counts
out-of-domain predictions. Fit timestamps cannot appear in the held-out set;
these diagnostics do not define a calibrated acceptance threshold.

`native/basalt/sonar_geometry.hpp` supplies unwhitened range/azimuth rows for a
map point using Basalt's additive map translation and left map-frame rotation.
It includes point and pose Jacobians and authored range/FOV checks. It is a
native geometry building block. The [native worker](native/basalt/README.md)
adds weighted sonar factors, measurement ownership and joint optimization.
Physical echo association and terrain-map accuracy remain unqualified.

`sonar_inertial.optimize_window` jointly adjusts position, velocity, attitude,
IMU biases and shared sonar points over 2–4 states. It requires an initial-state
prior with covariance, calibrated gravity and sensor mounts, point seeds,
associated range/azimuth observations and explicit sensor noise. Raw IMU
preintegration is refreshed after nonlinear steps. Tracks may contain temporal
noise correlations; noise between separate tracks is assumed independent.
This bounded NumPy reference solver reports local rank and retains unobservable
directions. Elevation branch ambiguity and point association require separate
qualification. Streaming integration and navigation readiness remain unqualified.

Optional `visual_geometry` and each track's `visual_observations` /
`visual_measurement_covariance` add rectified pinhole pixel residuals to the same
pose/point solve. Camera-only points can share pose states with sonar-only points;
a shared point ID asserts an externally established physical correspondence.
Camera and sonar noise blocks are assumed independent. Camera pose, intrinsics,
image bounds and clip limits are explicit. Duplicate consumed visual observations
and camera-model changes are rejected when using history. This reference path
does not replace the native VIO worker or provide live cross-modal association.

Optional `ping_patches` add single Ping returns to the same solve. Each patch
references shared sonar track IDs and estimates its normal and signed distance.
Planarity residuals occur once per support point; each independent range return
occurs once at its state timestamp. The predicted footprint must lie within the
observed support. Patch association, local planarity and independent calibrated
range noise are required assumptions. Overlapping patch support and duplicate
return timestamps are rejected.

Sonar and Ping observations may belong to different states: a later downward
Ping can constrain the same landmarks observed earlier by the forward sonar.
Every measurement uses its exact state timestamp; no sonar observation is
required at a Ping-only state. The later footprint must still be covered by
the associated landmarks. Timestamp coincidence alone does not establish
shared surface support. The 2–4-state reference window allows at most 3 seconds
between adjacent states; longer delays require persistent graph/patch handling,
which is not implemented by this function.

`retain_state_stamps_ns` optionally exports `linearized_prior` by eliminating
the other states from the final joint linearization. The rectangular square
root retains correlations between the remaining states, landmarks and planes,
including unobservable directions. Frozen anchors, plane tangent bases and
absorbed measurement IDs accompany it. `linearized_prior` consumes sonar/IMU
history instead of a new initial-state prior. Retained landmarks may have no new
sonar observation, allowing a later first Ping constraint to use their joint
history. Duplicate sonar observations, overlapping IMU intervals and changed
sensor/noise models are rejected. New observations must be explicitly declared
independent of absorbed measurements using `independent_new_measurements=True`;
cross-window correlated measurement noise is unsupported. The prior is frozen
at its stored anchors. Ping planes can be retained with the same ordered support,
sensor geometry and surface model. Subsequent windows add only new range rows;
the previously absorbed planarity rows are not repeated. A retained patch may
have no new return. Duplicate Ping timestamps and changed patch models are
rejected. Plane deltas use the stored tangent basis; antipodal normals are
rejected. Persistent physical patch association and live streaming remain
unimplemented.

Exported priors use QR compression to keep at most one more row than retained
variables. A zero-Jacobian row preserves the residual cost orthogonal to those
variables. Rank-deficient directions remain unregularized. Measurement ownership
records still grow with history; this matrix bound is not a whole-stream memory
bound.

`inertial_factor.pair_residual` uses measured `ImuWindow` preintegration to
constrain endpoint position, velocity, rotation and biases with their full
15-state covariance. Gravity and endpoint states are explicit optimization
variables. Bias corrections are first order; nonlinear bias steps require
re-integration. This supplies joint solver residuals, not a live estimator.

`sonar_joint.linearize_track` builds correlated pose/point rows for up to 16
observations of one associated acoustic return. It requires exact state times
and an explicit full range/azimuth covariance, including temporal correlations.
Shared visual/Ping rows must be assembled before the existing rank-aware
`ping_joint.eliminate_patch` removes point variables. No artificial elevation
or pose prior is added. This is a solver building block, not live fused SLAM.

`acoustic_tracks.AcousticTracks` retains up to 300 intensity-flow chains across
frames. Surviving pixels seed the next flow pass; fresh corners receive fresh
segment-local IDs. Time gaps, invalid calibration and complete loss break the
segment. Colliding one-pixel footprints are rejected. This bounded frontend
retains unresolved elevation and does not prove physical scatterer identity;
it is not yet connected to the live joint optimizer.

`acoustic_flow.correspondences` exposes at most 300 two-frame intensity track
candidates with integer nanosecond timestamps, paired pixels, calibrated
range/azimuth and forward/backward pixel error. It runs before planar consensus
and can retain candidates when the planar fit fails. Pair indices are not global
landmark IDs; pixel consistency does not establish a persistent point scatterer
or calibrated covariance. `estimate(..., include_correspondences=True)` includes
the candidate payload explicitly; default streaming status stays compact.

`sonar_factor.point_return_factor` supplies range/azimuth residuals and analytic
pose/landmark Jacobians for an associated 3D point return. It uses calibrated
IMU-to-sonar mounts and checks the sensor's range and field of view. Its two
measurement dimensions leave elevation unobserved. This original Apache-2.0
geometry helper uses NumPy and is not yet wired into a live joint optimizer.
The caller must establish point-return association, model uncertainty and retain
cross-sensor correlations; a collapsed intensity bin is not a known seabed point.

## Install and use

Python 3.10 or later and NumPy are required. The pure geometric components do
not require ROS, Gazebo or WWW-POSIM.

```sh
git clone https://github.com/IOES-Lab/POSIM-SLAM.git
cd POSIM-SLAM
python -m venv .venv
. .venv/bin/activate
python -m pip install -e .
```

On Windows, activate `.venv\Scripts\activate`.

```python
from posim_slam.inertial import ImuWindow

imu = ImuWindow()
for i in range(201):
    imu.add(i / 200., [0., 0., 0.], [0., 0., 9.80665])
delta = imu.preintegrate(0., 1., gyro_bias=[0., 0., 0.],
                        accel_bias=[0., 0., 0.],
                        noise_density=[.002, .02, .0001, .001])
```

These noise values are an example, not hardware calibration. Deltas exclude
gravity and initial velocity. A downstream estimator must jointly update its
pose, velocity and biases from independent observations.

Acoustic image tracking requires a checked OpenCV 4.13.0 native provider or the
research extra (`python -m pip install -e '.[research]'`). The native provider
uses `POSIM_SLAM_ACOUSTIC_LIBRARY`; the legacy `WWOS_ACOUSTIC_NATIVE_LIBRARY`
setting is also accepted. Its matching SDK, source/library hashes and ABI are
checked before loading. The research cv2 package is not a replacement for native
runtime acceptance.

## Native builds

In a matching OpenCV 4.13.0 development environment:

```sh
cmake -S native -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel 2
cmake --install build --prefix install
```

The visual ROS 2 adapter additionally requires the pinned Stella dependency
build. Enable `POSIM_SLAM_BUILD_VISUAL=ON` in that environment. Native provider
qualification and relocation receipts are separate from a successful CMake
build. The current acoustic provider receipt uses the macOS `.dylib` layout.
Native Windows provider execution is not qualified.

## Integrity and tests

```sh
python -m pip install -e '.[research]'
python -m unittest discover -s tests
```

Tests cover analytic scale/rotation/acceleration, interval coverage, clock and
session resets, invalid measurements, surface clipping, bias derivatives and
covariance. `test_integrity.py` verifies source pins, original notice hashes and
the absence of imports from the WWW-POSIM application. Tests do not establish
long-duration SLAM accuracy. Native sensor and independent truth comparisons
must qualify each application/runtime integration separately.

Source pins and patches are in `src/posim_slam/data/visual`; native adapters are
in `src/posim_slam/data/native`. These resources and third-party notices are
included in the Python package so checkout, wheel and restored runtime consumers
use the same source ownership.

Original code: [Apache-2.0](LICENSE). See [NOTICE.md](NOTICE.md). Third-party
licenses remain applicable. No ORB-SLAM3 code, vocabulary or model weights are
included. Basalt is the optional native VIO backend; Kimera is not used.

## Experimental visual-inertial research

The [headless Basalt target](native/basalt/README.md) builds a pinned square-root
VIO core and runs noisy 6DoF track/image plus raw-IMU regressions. It jointly
updates pose, velocity and biases in that standalone executable. The Python package does not bundle the executable. The host application registers
and supervises the worker and chooses its startup/default-mode policy.
Native-world timing, calibration, recovery and package acceptance remain separate.

The native camera interface offers previous-frame reference tracking (default)
and birth-frame reference tracking through `optical_flow_type`. Birth-frame
references retire after their last camera observation. See the
[reference-patch contract](native/basalt/README.md#visual-reference-patches--시각-참조-패치).

## Experimental inertial observation filter

`posim_slam.inertial_filter.InertialFilter` advances one local segment using
position/velocity, IMU attitude and gyro/accelerometer bias states. Continuous
noise densities are ordered gyro, accelerometer, gyro-bias walk, accel-bias
walk. All calibration, initial state and positive covariance are explicit.
The owner supplies synchronized measurements at the propagated timestamp;
delayed measurements require replay, and duplicate observation times are rejected.
DVL vectors use the actual mounted axes and rotational lever arm. Pressure input
is calibrated ENU sensor height, not a Ping range. A cloned pose preserves
cross-correlation for planar acoustic relative translation/yaw; normal movement
and tilt beyond configured limits reject the planar observation. It does not
resolve sonar echo elevations or create a global loop closure.

Updates use innovation gates, Joseph covariance updates and right-attitude
error reset. Three consecutive innovation rejections of one kind or missing
IMU coverage invalidate the segment. The owner must clear dependent maps and
reinitialize; it must also enforce required-sensor freshness and prediction
age limits. Statistical gates cannot establish initialization observability.
The math follows the [error-state formulation](https://arxiv.org/abs/1711.02508);
this module is original Apache-2.0 code with NumPy as its sole dependency.

```sh
PYTHONPATH=src python tools/evaluate_inertial_filter.py --output /tmp/inertial.json
```

This six-seed synthetic 6DoF experiment uses a known local-frame pose prior and
a two-second stationary raw-gyro calibration. It compares IMU-only propagation
with DVL/pressure updates, with no truth observation updates or scale alignment.
It does not qualify native sensor initialization, field accuracy, acoustic map
integration or application defaults.

## Moving camera/IMU startup

`visual_inertial_seed.tracked_imu_seed` forms a provisional metric velocity and
gravity seed from calibrated multi-view image tracks and raw IMU. Explicit
gravity magnitude and pixel-noise inputs enable bounded RANSAC track consensus
and tangent-plane gravity refinement. Biases remain conditioned inputs and
initialization uncertainty needs qualification. [Inputs and method](docs/diagnostics/visual-inertial-seed.md).

## Moving IMU/DVL initialization

`posim_slam.inertial_alignment.moving_initialize`, algorithm **0.1.0**, fits a
bounded factor graph to 7–30 seconds of raw IMU and 8–24 bottom-locked DVL
endpoints. Adjacent endpoints must be at most 1.5 seconds apart. It estimates
gravity, endpoint rotations and velocities, and constant interval gyro/accel
biases using explicit calibration priors. DVL mounting and rotating lever-arm
velocity are included. It supplies a local endpoint origin and initial yaw gauge.

Wave/current motion does not require a stationary RMS threshold. Missing IMU,
lost bottom lock, unknown covariance, timing gaps, inconsistent residuals and
failed convergence reject the window. Posterior attitude/velocity three-sigma
bounds and state cross-covariances accompany the estimate. Small residuals or
numerical rank alone do not qualify bias observability or navigation readiness.
Weakly excited motion depends on bias calibration. The transport owner supplies
source validation and lifecycle. Pressure must share the local vertical origin.

The synthetic moving benchmark uses an independent noisy gyro calibration
measurement, then 12 seconds of moving initialization and 8 seconds of tracking.
It uses no pose/orientation observations. `--weak-prior` exercises uncalibrated
zero-mean gyro bias against the same accuracy limits; this profile can fail even
when residuals pass. These profiles do not qualify an installed sensor.

```bash
PYTHONPATH=src python tools/evaluate_moving_initialization.py --output /tmp/moving.json
```

Stationary initialization below is an optional calm-condition method. An elapsed
30-second information period does not guarantee convergence or readiness.


A positive-semidefinite bias prior can express exactly calibrated directions.
Those directions are eliminated from optimization, not approximated with tiny
variance. Unknown covariance must not be encoded as zero. The caller must verify
the calibration source; noisy hardware requires a measured prior. State
pose/velocity covariance remains positive definite.

## Streaming inertial ownership

`InertialDvlSession` owns moving initialization and subsequent IMU/DVL updates.
It waits for at least 12 seconds of actual bottom-locked DVL coverage and raw IMU,
then checks maximum posterior 3-sigma bounds (3 degrees attitude, 0.1 m/s velocity).
It buffers 32,768 IMU and 2,048 DVL samples and bounds the initialization graph to
24 DVL endpoints. `ImuWindow` retains 8,192 samples by default and supports an
explicit capacity of 32–65,536 samples for high-rate owners.

DVL arriving ahead of IMU waits for real endpoint coverage. Invalid timestamps,
IMU gaps, bottom-lock loss, repeated innovation rejection or stale sensor input
invalidate the segment and clear its pose. Recovery collects a fresh moving
window. Snapshot timestamps are DVL acquisition times; poses are never retimed
to publication time. The transport owner validates source/session identity and
bounds its callback queue. Output is local inertial odometry without a map or
navigation-readiness claim.

## Measured stationary initialization

`posim_slam.inertial_initialization.stationary_initialize`, algorithm **0.1.0**,
starts an independent filter from 2–3 seconds of raw IMU plus at least three
bottom-locked DVL samples bracketing the whole interval (maximum DVL gap 1.5 s).
It uses physical IMU/DVL mounts, explicit noise densities, accelerometer bias
calibration/covariance and residual-motion uncertainty. Zero or unknown DVL
covariance is rejected. Conservative low-dynamics gates require mean gyro norm
≤0.03 rad/s, gyro RMS ≤0.02 rad/s, accel RMS ≤0.12 m/s², gravity-norm mismatch
≤0.25 m/s² and each DVL speed plus 3σ ≤0.06 m/s. These are experimental
low-noise stationary-profile limits, not calibrated defaults for every sensor.

Mean specific force supplies roll/pitch, including inverted physical IMU axes.
Initial position and yaw define a gravity-aligned local gauge, not geographic
heading. Mean stationary gyro supplies its bias estimate. Accelerometer bias
is retained as a calibration prior, with its cross-covariance to initial tilt;
rest cannot identify all its components separately from gravity. DVL supports
linear stationarity, while IMU dynamics support angular stationarity. Constant
motions may remain ambiguous; satisfying these gates does not prove full
observability or navigation readiness. No ideal orientation or pose enters the
initializer. The application has no adapter for this initialization path yet.

```sh
PYTHONPATH=src python tools/evaluate_stationary_initialization.py --output /tmp/initialization.json
```

The synthetic fixture starts from stationary measured inputs, then follows 20 s
of smooth 6DoF motion. Truth is used only to generate measurements and evaluate
in the initial local position/yaw gauge, without trajectory or scale fitting.

Pressure observations must use the initialized local vertical reference; an
absolute sea-level height cannot be mixed with a zero local origin.

## Multi-sample gravity seed

`posim_slam.gravity_seed.gravity_seed` averages calibrated specific force over a
covered 0.2–3 second `ImuWindow`. Midpoint gyro integration transports every
sample into the final IMU frame before trapezoidal averaging. Supply explicit
gyro and accelerometer bias estimates. The returned direction is a numerical
seed, not a velocity, bias or observability estimate. Constant translational
acceleration can bias it even with noise-free measurements. The native Basalt
worker accepts this direction through a first-image timestamp-bound seed packet
when its calibration requests `require_gravity_seed`. Raw IMU packets remain
unchanged; velocity and bias seeds still require their own assumptions.

`sonar_input.projected_frame` decodes bounded float32 multibeam messages with either byte order. It validates timestamps, range bins and unit beam directions, and preserves negative infinity as a missing return. The decoder has no ROS dependency.

Acoustic/DVL intervals retain normal velocity and reject more than 5 cm excursion outside the sonar fan plane, including reversals with zero net displacement. This is an experimental model-validity gate, not elevation recovery or probabilistic confidence.

## Recorded live-worker comparison

`tools/stream_recording.py --gravity-seed` selects the first image with one second of measured IMU history and sends the rotation-compensated gravity seed plus a 200 ms IMU bootstrap buffer. Subsequent images remain in the evaluation sequence. The receipt records omitted initialization frames and the exact seed. Initial velocity and biases remain zero assumptions.

Add `--joint-ping` to fuse recorded ranges with exactly matching image timestamps. Both Ping files must appear in the verified recording hash inventory. Invalid returns are omitted; nearby timestamps are not substituted. Repeat without this flag on the same recording to compare camera/IMU and camera/IMU/Ping. `states.json` retains native diagnostics alongside the pose CSV. This reproduces sensor-time inputs and numerical initialization, not ROS callback timing or an accuracy qualification.

Native stream states include `initialization_window`: the oldest retained optimizer state after optimization and before marginalization. Its timestamp, pose, velocity and biases allow recorded startup diagnostics to distinguish the emitted prediction from subsequent window corrections. This field does not certify initialization or change estimator inputs.

Optional native input setting `record_visual_tracks: true` writes a sibling
`*.visual-tracks.jsonl` file for up to 1,800 map frames, followed by an explicit
truncation marker. Each row preserves actual frontend IDs/pixels, available
optimized 3D coordinates, reprojection error, full-weight status and current
mature archive membership. Pixel coordinates use the recorded processed image
width/height. The default is off. These records diagnose where geometry is lost;
they do not alter feature selection, triangulation or map admission. The native
`basalt_visual_trace_probe` checks identity and stage distinctions.

The native worker retains startup velocity/bias states until visual landmarks exist. The stream rejects a retained state window exceeding 128 states or 30 seconds of sensor time. This preserves variables for subsequent visual correction; it does not estimate initial velocity from IMU alone or qualify initialization.

Use `--first-image-stamp-ns` with `--gravity-seed` to reproduce a recorded live startup timestamp. The requested image must exist exactly and have covered IMU history. Delayed marginalization establishes a linearization reference for every newly retained pose before constructing its prior; existing references remain unchanged.

[Visual-map depth support / 시각 지도 깊이 지지와 한계](docs/diagnostics/visual-depth-admission.md).

[Fixed-pose sonar power reconstruction / 고정 자세 소나 강도 복원](docs/diagnostics/sonar-power-volume.md)
provides a separate bounded diagnostic, numeric input fixture and complete candidate-cell PLY.
It does not change the runtime estimator or qualify a navigation map.

[Sonar forward-model checks and deeper survey / 소나 관측 모델 검사와 깊은 수심 관측](docs/diagnostics/sonar-forward-model.md)

The [finite-surface model and conditional inverse / 유한 표면 모델과 조건부 역문제](docs/diagnostics/sonar-finite-surfaces.md) include opaque first-hit visibility, ideal surface normals, a fractional range response and matching WGPU power probes. The guide retains the unknown-shape diagnostic's failed geometry assessment and its matched RGB comparison.

[Shared scattering coordinates and correlated power / 공통 산란 좌표와 전력 상관](docs/diagnostics/sonar-power-statistics.md)
define the volume projector's train/test unit contract and conditional complex-Gaussian intensity moments. Corrected scores preserve all geometry; actual WGPU controls and a rejected relative-loss map retain their limits.


[CPU multiview depth reference / CPU 다중 시점 깊이 복원 기준실험](docs/validation/visual-dense-reference-20261027/README.md)
compares an unmodified BSD-2-Clause OpenSfM depth kernel against recorded sparse
landmarks using fixed estimated VIO poses. The complete PLY, matching field RGB,
all-point errors and missing-face evidence describe an offline reference;
runtime integration and complete-map qualification remain open.

추정 VIO 자세를 고정한 오프라인 기준실험으로 전체 PLY·동일 시점 RGB·모든 점의 오차와
누락된 면을 보존합니다. 실시간 통합과 완전한 지도 검증은 아직 완료되지 않았습니다.

[Multiview image support / 다중 시점 영상의 관측 지지](docs/diagnostics/multiview-inputs.md)
describes `image_neighbors`, an independent NumPy implementation of the standard
shared-track/parallax view-selection rule. Every supplied image ID remains in
the output graph; display point limits do not select reconstruction images.

`image_neighbors`는 공통 특징과 시차를 이용하는 기존 뷰 선정 규칙의 독립 NumPy
구현입니다. 모든 입력 영상 ID를 출력 그래프에 보존하며 표시 점 수로 복원 영상을
선택하지 않습니다.

[Temporal stereo and acquisition coverage / 시간차 스테레오와 촬영 범위](docs/validation/visual-temporal-stereo-20261027/README.md)
compares conventional OpenCV stereo with recorded sparse landmarks and matching
field RGB. It retains complete-map errors and a failed long native observation;
fast offline calculation does not qualify runtime mapping.

[Archive information audit](docs/diagnostics/multiview-inputs.md#archive-information-audit--지도-관측-정보-진단)
reports fixed-pose pixel information, camera baseline and cross-view reprojection
for every exported landmark without filtering its map.
