# Initial IMU bias calibration in the native surface owner

The optional camera-free surface-owner profile accepts
`initial_imu_bias_calibration`, schema
`posim-slam-initial-imu-bias-calibration-1`, scope `first_state`, frame
`input_imu`. Its mean is `[gyro_bias_xyz, accel_bias_xyz]`; the full 6 × 6
covariance keeps gyro/accelerometer and cross-axis correlations. Inputs require
an explicit source. `initial_bias_calibration_from_model` accepts the generated
noiseless-SDF certificate; absent calibration or ROS covariance zero supplies
no such information. Other native profiles reject this setting explicitly.

`InitialImuBiasCalibration` separates the covariance's finite and exact modes.
Finite directions contribute calibrated prior rows. Exact directions define
affine support. The existing generic initial bias prior is replaced once;
initial origin/yaw information remains. IMU noise densities and subsequent bias
random walks retain their configured values. This is an initial calibration,
not a claim that bias stays exactly constant throughout the recording.
Covariance rank uses a relative 64-machine-epsilon eigenspace tolerance; the
caller supplies a covariance resolved at that precision.

`constrained_information` / `constrained_information.hpp` eliminate exact modes
in their nullspace rather than approximate them with large weights. Native
optimization, marginalization and retired-state conditionals retain finite
noise, unbounded modes and deterministic support separately. Delayed finite
range updates preserve that support and its complete dependency closure.
The native owner rejects a replacement which changes exact support.

Same-snapshot information with live equalities uses schema
`posim-slam-native-surface-information-2`. Version 1 remains for unconstrained
information. This prevents an older reader from silently discarding exact rows.
`LiveClient` also requires the worker's source/scope/mode-count acknowledgment
and confirmation that generic initial bias information was replaced while
stochastic evolution stayed active. These interfaces require matching source
and worker versions; they do not update installed providers automatically.

## Independent and actual-native controls

Twelve additional Python controls cover dense constrained Gaussian/KKT
comparisons, shared deterministic ancestry, full PSD bias calibration,
malformed input, source certificates and old-worker acknowledgment. Frozen
source and an outside-installed final wheel each run 522 cases: 506 pass and
sixteen unrelated optional native/legacy cases explicitly skip. All separate
logs and receipts are retained.

Forty-three actual native controls cover finite/singular calibration, means and raw IMU frame,
optimization, unrelated-state retirement, inertial-to-pose downgrade, shared
cross covariance, delayed updates, rejection/rollback, pressure/DVL ownership,
source clocks and archive limits. Full owner traces retain the raw and compact
rows. Their independent KKT solutions and covariance reproduce from the public
library without simulator truth. Basalt is pinned at
`0f3b2b52c807f70ff4e2973ce253c73329eea7bc`; this Mac development worker uses
Eigen 5.0.1 and OpenCV 4.13.0. No dependency or third-party license changes.

The initial test copier omitted the new deterministic field; its failure and
corrected transport fixture are retained. A dense-control fixture initially
compared a step with absolute coordinates, omitting the existing 0.1 pressure
datum; its expected coordinates were corrected without relaxing tolerances.
Pre-publication protocol-1 prototypes remain locally indexed. Final protocol-2
controls are separate and preserve every original source reading.

## Same original 15 m recording

Four final replays use the original raw IMU, DVL, pressure and sonar event times
from [the single 15 m dataset](../sonar-low-altitude-15m-20261029/README.md).
No raw recording is copied. The SDF and its IMU metadata explicitly declare
zero bias/covariance. Calibrated runs consume that certificate at the first
state; they change no subsequent noise/random-walk parameter, state budget,
geometry/support gate, timestamp or observation. The uncalibrated controls
retain **exactly identical pose components** to the preceding published worker.
All four keep all 178 states; every native event and original request remains.
Client close returns SIGTERM (-15) after complete offline replay, distinct from
normal-zero simulator acquisition.

| Control | Position RMSE | Relative position 3σ major axis | Rotation 3σ major axis |
| --- | ---: | ---: | ---: |
| Uncalibrated DVL ranges | 2.1133 cm | 5.42438 m | 69.6491° |
| Calibrated DVL ranges | 2.5198 cm | 0.174584 m | 1.9055° |
| Uncalibrated rejected FLS candidate | 2.1139 cm | 5.42438 m | 69.6491° |
| Calibrated rejected FLS candidate | 2.5190 cm | 0.174584 m | 1.9055° |

Uncertainty is the local frozen-weight approximation at 33.01 s; relative
position compares 3.01 s and 33.01 s. RMSE assesses the complete 178-state
trajectory using separate body-origin truth, measured sensor mounting, initial
yaw/translation alignment and scale 1. Assessment truth is read only after
replay, never sent to the estimator. The slightly larger mean error is retained;
this result fixes calibration information, not demonstrated trajectory accuracy.

Initial snapshots show six exact bias modes. Later random walks retain bias
uncertainty. The DVL-only root remains rank 228/229, with one free plane mode;
all original nonlinear support still passes at 43.01 s. The first-causal FLS
correspondence still rejects as `dvl_support_not_planar`, leaving native
coordinates, references, revisions, archive, ledger and information identical.
No acoustic map is qualified. The original
[matched RGB/sonar/isometric views](../native-surface-ranges-20261029/README.md)
remain the input context; no conditional hypothesis is relabelled an accepted
landmark. The original 102 hypotheses and all 664 prediction outcomes remain
in their earlier records.

## Reproduce and next decision

With the current public library installed:

```sh
python docs/diagnostics/native-imu-bias-calibration-20261029/verify.py
```

The verifier reproduces initial support, nonlinear preparation/rejection,
continued support and joint marginals from the final snapshots, then checks
native owner covariance/modes against independent dense KKT systems. It needs
no simulator or application imports. Source/worker hashes, compiler/link
identity, all grouped logs, raw/compact owner traces, complete replay events,
processing-identity checks and the external data pointer are retained.

The past day's ownership/covariance work exposed a real calibration omission,
but point identity and operating geometry remain the main mapping questions.
Finish this calibration qualification, then test a small static wall/corner
scene with overlap, turns and lateral/depth changes. Evaluate navigation and
physical echo association separately before combining them. Preserve this
unchanged control, declared noise, every candidate and map-admission gates.
Do not add further optimizer constraints to compensate for failed association.
Installed/provider recovery, useful complete maps, BlueROV/product/VRX and the
later WAM-V lawnmower survey remain open.

한국어: 명시적인 초기 IMU 바이어스 보정을 전달하고, 유한 분산과 정확한 제약을
구분해 최적화·주변화·지연 갱신에서 유지합니다. 이후 잡음·랜덤워크는 그대로입니다.
같은 원본에서 상대 위치 3σ는 5.42 m에서 17.5 cm, 자세 3σ는 69.6°에서 1.91°로
줄지만 궤적 RMSE는 2.11 cm에서 2.52 cm로 소폭 커집니다. 평면의 미관측 방향과
잘못된 FLS 대응은 그대로 남습니다. 다음 우선순위는 작은 벽·모서리 장면에서의
물리적 대응과 운용 경로이며, 지도나 설치형 앱의 완료를 뜻하지 않습니다.
