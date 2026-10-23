# POSIM-SLAM

[한국어](README.ko.md) · [Third-party notices](THIRD_PARTY.md)

Independent sensor estimation components used by POSIM applications. Python
algorithms accept measured sensor arrays and calibrated transforms. WWW-POSIM
owns sessions, vehicle composition, runtime supervision, UI and map downloads.

Version **0.1.0** includes the following components. This version is a package
version; acoustic/DVL fusion retains its algorithm version **0.2.0**.

| Component | Method and current scope |
|---|---|
| Visual provider | Stella VSLAM 0.7.0 monocular ORB features and g2o/Eigen optimization; ROS 2 adapter and exact dependency pins supplied; arbitrary scale; empty vocabulary configuration has no qualified loop closure/relocalization |
| Visual + single Ping | Robust local-plane fit and synchronized range with mount/bias compensation; stable median metric scale; requires a suitable downward view and locally planar seabed |
| Visual + DVL | Bottom-locked velocity integration with calibrated axes and rotating lever arm; metric scale calibration, not joint drift correction |
| Acoustic frontend | OpenCV Shi–Tomasi corners and forward/backward pyramidal Lucas–Kanade flow; robust 2D SVD/RANSAC registration in the sonar fan plane |
| Acoustic + DVL | Bracketed velocity integration for translation, acoustic yaw/features and displacement consistency gate; planar experimental odometry; elevation unresolved |
| Surface filter | Pressure/orientation-derived mean-water plane and calibrated metric-map clipping; does not classify individual surface echoes |
| Inertial core | Raw gyro/accelerometer midpoint SO(3) preintegration, bias Jacobian and 15-state error covariance; preparation code, not a complete fused estimator |
| Experimental inertial filter | Algorithm 0.1.0: 15-state right-error EKF, raw-IMU prediction, mounted DVL velocity, calibrated pressure height and correlated cloned planar relative-pose updates; explicit initialization/calibration; streaming IMU/DVL session owner |

Running visual/acoustic modes do not yet use the inertial preintegration core.
No estimator accepts simulator pose or ideal orientation as a localization
measurement. Sonar intensity features are not classified 3D obstacles.

Ping scale support requires every ray intersection inside the observed plane hull and within 5% of plane distance of a landmark. The `ping_factor` module supplies pose/plane range Jacobians and a supported-patch wrapper for joint optimization; it is not yet connected to the native VIO worker.

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
included. Basalt and Kimera are not active backends in this package.

## Experimental visual-inertial research

The [headless Basalt target](native/basalt/README.md) builds a pinned square-root
VIO core and runs noisy 6DoF track/image plus raw-IMU regressions. It jointly
updates pose, velocity and biases in that standalone executable. It is not
selected by default in WWW-POSIM or included in the default Python/native provider build.
Native-world timing, calibration, recovery and package acceptance remain separate.

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
