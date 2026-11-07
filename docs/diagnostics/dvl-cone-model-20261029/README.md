# Finite DVL cone and local-plane range geometry

`posim_slam.dvl_surface` is an independent NumPy reference for a range constraint
between a sensor pose and an associated local plane. It is Apache-2.0 project
code and adds no third-party implementation or runtime dependency. It is not
enabled in the application's native estimator.

## Measurement and derivatives

The plane is `n dot X = d`, with a unit normal. In the sensor frame, let `h` be
the positive perpendicular distance to the plane, `a` the unit beam axis,
`alpha` the HALF aperture, and `beta = acos(n dot a)`. The continuous first
slant range is `h / cos(max(beta - alpha, 0))`. This is a minimum over a finite
cone, rather than the centre-ray range `h / cos(beta)`.

The API accepts a full aperture, fixed DVL reference-to-IMU rotation and lever
arm. It supports either signed plane representation. Analytic Jacobians use
additive map position, right IMU rotation, left normal rotation and signed plane
distance. Residuals are measured minus predicted; returned Jacobians describe
the prediction. `conditional_hit` is a plane-conditioned minimizer, not an
observed Cartesian point or verified landmark.

The cone/plane intersection is an ellipse only when every ray has a forward
non-grazing intersection. Grazing/unbounded footprints, an origin on the plane,
and first returns outside the sensor limits are rejected. No farther valid
sample is substituted for an unsupported blind-range minimum.

`require_cone_surface_support` checks the entire analytic ellipse against the
convex hull of caller-supplied local plane inliers. An exact ellipse support
function checks each hull edge. A conservative nearest-sample disk bound also
rejects large unobserved holes. The caller supplies plane-deviation and spatial
support limits. This assumes local planarity; it cannot prove dense occupancy,
assign echoes to planes or rule out an unobserved nearer object.

`whiten_plane_ranges` requires a full positive-definite covariance and a label
describing its provenance. Factors must share the same pose, mounting and
signed plane. Unknown/default-zero range variance is not converted to exact
information. Shared range errors remain correlated. The reported information
spectrum is conditional on the plane and correspondence, not a joint state
covariance. On a horizontal plane, the four-beam pose linearization retains
unobservable horizontal translation and yaw.

## Independent and retained-native controls

Six new tests cover 24 independent dense-ray comparisons, pose/plane numerical
derivatives for both plane signs, exact ellipse boundary geometry, whole-cone
support and unseen-hole rejection, correlated uncertainty/null directions,
zero-width/interior branches and invalid returns. Related point-support and
Ping geometry tests pass after sharing the hull helper. A clean Git source
export passes 443 library tests with 16 optional checks skipped. Protected user
work is excluded from that export.

A built wheel installed into a separate target directory also passes the same
443-test suite outside the source checkout, with 16 optional native probes/
decoder checks skipped. Its code hash, wheel hash and explicit skip reasons are
retained. This verifies the independent Python package, not a WWW-POSIM SDK,
application archive or native range-fusion deployment.

The retained native flat-floor fixture uses the installed gz-sensors 10.1.1
binary. Its original per-case measurement rows match the preserved source
record, with nine reports, 24 locked-beam returns and twelve unlocked beams.
The SDFs and engine hashes are included. The controlled model has a known
horizontal floor and sensor mounting; using that geometry here is assessment,
not a terrain prior supplied to SLAM. No simulator is rerun for this record.

All 24 continuous-cone residuals are retained: median/maximum absolute error is
6.1716/6.1717 mm. The centre-ray model has 9.4865/9.4868 mm. A continuous minimum
does not exactly reproduce the native renderer's angular raster or float depth.
These errors are not zeroed or promoted to a calibrated noise bound. The
explicit 20 mm independent / 10 mm shared covariance in the rank demonstration
is a diagnostic assumption. Bottom range variance remains unreported.

Reproduce the geometric comparison after installing POSIM-SLAM:

```sh
python tools/evaluate_dvl_cone_geometry.py \
  --record docs/diagnostics/dvl-cone-model-20261029 \
  --output /tmp/dvl-cone-results.json \
  --independent-sigma-m 0.02 --shared-sigma-m 0.01
python docs/diagnostics/dvl-cone-model-20261029/verify.py
```

The first failed wrapper invocation never imports the library; the next uses
an explicit independent source path. An export attempt names a nonexistent
notice filename and stops before tests. The corrected export includes the
actual `NOTICE.md`/`THIRD_PARTY.md` and produces the retained successful suite.
The installed suite completes successfully before a JSON-receipt serialization
error. Receipt repair stringifies skipped TestCase objects and handles solver
stdout mixed with progress; only those sixteen skipped checks are re-examined.
Both complete logs remain, with no library/test change to obtain the receipt.

## Operating boundary and references

The next field input must preserve original beam ranges and locks. Associate
only locally supported surfaces from independent observations and informative
viewpoints, keep pose/plane correlations in the joint estimator, and test steps,
walls, loss and recovery. Four bottom beams do not establish a global plane or
resolve all forward-looking sonar elevations. The existing 96/34-point acoustic
maps remain failed cases; this reference does not change them.

Using DVL ranges for terrain constraints is established practice. The
[HUGIN terrain-navigation study](https://www.navlab.net/Publications/An_Analysis_of_Real_Time_Terrain_Aided_Navigation_Results_from_a_HUGIN_AUV.pdf)
uses four DVL ranges with a prior bathymetric map. The
[Ozog et al. hull-mapping paper](https://robots.engin.umich.edu/publications/pozog-2015a.pdf)
describes individual DVL-beam/plane factors. Those methods do not establish
the validity of a perfect terrain prior or uncertain acoustic correspondences
in this demo. The installed simulator's actual minimum-cone/variance behaviour
is defined by its [tagged source](https://github.com/gazebosim/gz-sensors/blob/gz-sensors10_10.1.1/src/DopplerVelocityLog.cc).
The finite-cone reference/support controls are engineering model work; no
research novelty or hardware/navigation approval is claimed.

## 한국어

독립 `dvl_surface` 모듈이 빔 원뿔 안의 최단 반사 거리와 국소 평면의 제약을
계산합니다. 자세·평면의 미분, 빔의 전체 타원 발자국에 대한 관측 범위 검사와
공유 오차를 보존하는 공분산 처리를 제공합니다. 연결된 평면과 명시적 잡음
모델이 필요하며, 조건부 교점을 실제 관측한 3차원 점으로 간주하지 않습니다.
미관측 영역·수평 위치·방위각에 인위적인 정보를 만들지 않습니다.

새 시험 6개와 관련 검사 4개가 통과하고, 사용자 작업을 제외한 소스 내보내기에서
443개 시험 중 16개 선택형 검사를 건너뛰고 나머지가 통과했습니다. 보존된
실제 네이티브 평면 자료의 잠금 빔 24개를 모두 평가했습니다. 절댓값 오차는
원뿔 모델 약 6.172 mm, 중심 광선 모델 약 9.487 mm입니다. 연속 원뿔과
렌더러의 이산 샘플링은 다르며 이 오차를 0으로 처리하거나 보정된 잡음으로
삼지 않습니다. 네이티브 시뮬레이터나 SLAM을 새로 실행한 결과가 아닙니다.
별도 폴더에 설치한 wheel도 소스 밖에서 같은 443개 시험 중 16개를 제외한
나머지가 통과합니다. 이는 독립 Python 패키지 검사이며 앱 설치 검사는 아닙니다.

이 구현은 Apache-2.0 자체 코드와 NumPy를 사용합니다. 앱은 DVL 속도만
융합하며, 다음은 빔별 원본을 보존하는 이동 기록과 실제 국소 표면 연결·
관측 범위를 검증하는 단계입니다. 단일 Ping 제외·IMU·압력 수심 기본값을
유지합니다. 기존 실패 지도는 그대로 보존하며, 연구 신규성을 주장하지 않습니다.
