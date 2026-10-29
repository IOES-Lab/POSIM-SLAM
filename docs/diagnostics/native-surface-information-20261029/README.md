# Same-snapshot native joint uncertainty

The optional camera-free surface-owner snapshot carries undamped local
Gauss–Newton information from the owned marginal, active continuous IMU
intervals, DVL velocity and pressure rows. Each store contributes once.
`surface_information.hpp` uses the optimizer's coordinates and robust weights;
Householder QR compacts the augmented Jacobian/residual while preserving
information, gradient and constant cost. Trial damping contributes no rows.

`NativeSurfaceSnapshot.joint_marginal(order)` uses SVD and propagates the root
noise and free directions through the complete shared conditional ancestry.
Queries can contain current or retired pose/inertial blocks, owned planes and
the pressure datum. Cross terms and each ancestor's noise identity remain
intact. `deltas` are nominal owner coordinates; `linearized_mode` also reports
the read-only undamped linear solve. Neither changes the estimator.

The covariance is a **local frozen-weight approximation** under the existing
sensor/noise assumptions. `observable_covariance` must always be interpreted
with `unbounded_basis`. It is not calibrated map uncertainty. DVL gyro/IMU
cross-correlation and velocity/range cross-correlation remain unmodelled;
reported DVL range variance is still unavailable. Echo identity remains a
separate requirement. No qualified sonar map or installed provider is produced.

## Independent controls

Seven Python controls compare joint retired/live/plane/pressure covariance and
mode with independent dense systems, retain shared free directions, exercise
inertial-to-pose downgrade, and reject malformed or oversized queries. The
existing known-root conditional API retains its original meaning.

Eleven compiled owner controls include an independent uncompressed row stack.
The new compact export preserves its full information, gradient and residual
cost before/after real IMU/DVL/pressure retirement. Twenty actual streaming
controls cover repeated read-only snapshots, expiry, delayed installation,
no-range equivalence, DVL/pressure ownership and stream clocks. All pass.
Two complete owner traces retain both raw and compact rows for reproduction.

The frozen source and outside-installed final wheel each run 510 tests:
494 pass and sixteen optional native/legacy controls explicitly skip. Their
complete separate logs and receipts are retained. The new owner/worker are
built from the source manifest in this record, using Basalt
`0f3b2b52c807f70ff4e2973ce253c73329eea7bc`, Eigen 5.0.1 and OpenCV 4.13.0.
The initial private-base-access compiler failure and its corrected build logs
are retained; dependency sources, the SDK and registered providers stay intact.

## Original 15 m measurements

The two replays reuse the single [15 m raw recording](../sonar-low-altitude-15m-20261029/README.md).
No raw data is copied. Both retain all 178 states, identical timestamps and
**exactly identical pose components** to their respective preceding range-owner
controls. Every native event is retained. Client close returns SIGTERM (-15)
after complete offline replay; this is separate from normal-zero acquisition.

At 33.01 s, the actual snapshot contains fourteen active IMU intervals, five
DVL velocity readings, four pressure readings and the owned marginal. The
DVL-only plane leaves one unbounded direction: root rank 228/229. Full joint
position three-sigma major axis at 33.01 s is **5.42448 m**; treating that live
coordinate as known reports zero conditional variance. The 3.01 s retired
position is 1.5195 cm jointly versus 1.4819 cm conditionally. These values are
uncertainty diagnostics, not measurement errors or hardware accuracy.

The native initial origin/yaw anchor already has information 1e8. Relative
translation from 3.01 s to 33.01 s still has a 5.42438 m three-sigma major axis,
so the large result is not removed by choosing a world-frame origin. Current
yaw-dominated rotation uncertainty has a 1.21561 rad three-sigma major axis;
the gyro-bias major axis is 0.0398627 rad/s. This identifies weak heading/bias
observability in the near-fixed-heading input; it does not establish that a
new maneuver or echo association will pass. No weight, prior or covariance is
changed to shrink these values.

At 43.01 s, all original DVL support still passes the nonlinear geometry check;
the same 33.01 s position is now retired and has a 3.36065 m joint major axis
versus 4.71195 cm with current coordinates fixed. The plane's unbounded mode
remains. The conservative largest query workspace is 28,690,432 bytes within
the existing 32 MiB recovery limit. The persistent state, archive, line-message,
raw-input and bulk-file limits are unchanged.

The first-causal DVL+FLS correspondence still rejects as
`dvl_support_not_planar` before installation. Before/after native coordinates,
references, conditional archive, revisions, ledger **and full information** are
identical. The previous [matched RGB/sonar/field comparison](../native-surface-ranges-20261029/README.md)
retains the original views and every rejected hypothesis; no conditional point
is promoted to an accepted native landmark.

## Reproduce

With the current public library installed, run:

```sh
python docs/diagnostics/native-surface-information-20261029/verify.py
```

The verifier needs no simulator, application modules or truth inputs. It
reproduces preparation/rejection, continued support and every joint marginal
from the retained native snapshots, then compares compact/raw row moments in
the complete owner traces. Source hashes, compiler/link commands, every grouped
check log, native snapshots/increments/events and raw-data location remain in
this record.

Next qualify the heading/bias observability and physical echo/support/noise
model. Test a short maneuver with turns and lateral acceleration against this
unchanged reference before expanding trials or enabling map admission. Product
recovery/export, latest-SDK qualification, VRX and the later WAM-V surveying
demo remain separate open requirements.

한국어: 같은 네이티브 상태의 IMU·DVL·수압·주변화 정보를 읽고 과거 자세와 평면의
상관관계를 유지합니다. 원본 178개 자세는 그대로지만, 현재 좌표를 확정값으로 놓은
계산이 숨겼던 큰 위치·방위 불확실성과 평면의 관측 불가능 방향이 드러납니다.
다음 검사는 회전·횡가속을 포함한 운용 동작과 반사면 대응입니다. 가중치나 잡음을
바꿔 통과시키지 않으며, 유용한 지도·설치형 앱·VRX·WAM-V 측량의 완료를 뜻하지 않습니다.
