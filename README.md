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

Running visual/acoustic modes do not yet use the inertial preintegration core.
No estimator accepts simulator pose or ideal orientation as a localization
measurement. Sonar intensity features are not classified 3D obstacles.

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
