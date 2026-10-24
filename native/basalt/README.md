# Experimental headless visual-inertial target

[한국어](#한국어) · [Source pins](sources.json)

This research target runs Basalt's square-root visual-inertial estimator with
raw gyro/accelerometer measurements. It jointly updates pose, velocity and IMU
biases. It is separate from the application's selected Stella provider and is
not a ROS node, installable engine provider or loop-closing SLAM system.

The recipe compiles only estimation, optical flow and supporting core sources.
It does not link Basalt's GUI, dataset loaders, AprilTag calibration, mapping
applications or camera drivers. Exact Basalt, basalt-headers, Sophus, cereal,
magic_enum, OpenGV and JSON revisions are verified before building. Eigen
5.0.1, TBB, fmt and OpenCV come from the selected development SDK. This differs
from the selected Stella provider's Eigen 3.4.0 dependency.

```sh
python tools/build_basalt.py --engine /absolute/sdk \
  --source /absolute/research-sources --output /absolute/new-build --fetch
```

The output directory must not exist. `--fetch` only populates missing source
repositories. Existing revisions, source changes and untracked files are
rejected; the builder never resets a checkout. CMake 4 requires the explicit
3.5 policy floor for the pinned OpenGV build. OpenGV is compiled as C++17 with
an explicit `cassert` include. Upstream source is not patched. Parallelism is
limited to 1–4 build jobs and two estimation workers.

The executable generates a deterministic 12-second six-axis motion with a
one-second stationary start, 5 Hz camera observations and 200 Hz IMU. Two paths
run separately: noisy projected feature tracks and rendered grayscale images
through Basalt's optical-flow frontend. The renderer projects fixed textured
3D tiles; the estimator receives pixels, calibration and raw inertial channels.
It never receives the renderer's points, true pose or ideal IMU orientation.
Injected gyro/accelerometer biases start with zero estimator bias priors.

The receipt records 61 output states per path, position/rotation errors and
final bias errors. Position evaluation uses the initial coordinate frame
directly, without scale fitting or trajectory alignment. The regression gates
are 0.12 m position RMSE, 0.03 rad rotation RMSE, 0.001 rad/s final gyro-bias
error and 0.025 m/s² final accel-bias error. These are short synthetic test
limits, not hardware accuracy or underwater/native-world acceptance.

Retain the pinned source trees, copied notices and SDK notices with any binary
redistribution. `EIGEN_MPL2_ONLY` applies to this build too. The build receipt
does not establish complete licensing compliance for a different SDK or build.

## Recorded pixel/IMU replay

The same recipe builds `basalt_replay`. It accepts a recording directory with
`calibration.json`, `imu.csv`, `frames.json` and grayscale PNG images. Camera
intrinsics describe undistorted pinhole images. `T_i_c` transforms optical RDF
camera coordinates into the physical IMU axes, including the rotating baseline.
Gyro is rad/s, specific force is m/s², and all timestamps are integer nanoseconds
from one sensor clock. The calibration requires explicit continuous noise
densities and bias random walks; a zero sensor covariance is not calibration.
No pose/orientation truth channel is accepted.

```sh
python tools/replay_vio.py --binary /absolute/new-build/build/basalt_replay \
  --inputs /absolute/recording/inputs \
  --recording-receipt /absolute/recording/receipt.json --output /absolute/new-replay
python tools/evaluate_vio.py --states /absolute/new-replay/states.csv \
  --truth /absolute/recording/evaluation-truth.csv \
  --recording-receipt /absolute/recording/receipt.json --output /absolute/new-evaluation
```

The wrapper verifies recording and pixel hashes before replay. The C++ reader
rejects clock reversals, gaps above the declared IMU limit (at most 100 ms),
saturation, missing endpoint coverage, image gaps above 500 ms, changed image
dimensions and files outside the recording directory. It keeps bounded input
queues and uses two estimation workers. Existing output paths are rejected.
The state output contains estimated position, orientation, velocity and biases
at each exposure. It is a VIO research replay, not a live provider or SLAM map.

The independent evaluator reads truth from a separate directory/file argument.
It transforms body-origin truth to the IMU origin, interpolates bracketing
poses, fixes initial yaw/translation, and retains gravity tilt and scale errors.
It reports ATE, one-second RPE, traveled distances and evaluated coverage. It
never fits scale or the entire trajectory. Missing truth brackets fail by
default; `--trim-boundary-states` explicitly permits at most two uncovered edge
states and reports them. Internal truth gaps still fail. Four analytic tests
cover a rotating sensor baseline, gauge, scale drift, gravity tilt and gaps:

```sh
python -m unittest discover -s native/basalt -p 'test_*.py'
```

## Live worker transport

`basalt_stream calibration.json` accepts framed raw IMU and grayscale pixels
over stdin and emits JSON states/accumulated-landmark point samples over stdout. The
independent `posim_slam.live_client.LiveClient` provides asynchronous transport.
The worker uses two computation workers, one OpenCV thread, two queued images, 256 queued IMU
measurements and four queued state/map outputs. Python queues allow at most
256 input packets / 4 MiB and eight outputs. Backlog is a segment fault, not a
silent packet drop. Display samples contain at most 1,000 points.

The client rejects sensor packets until a single valid protocol-ready message
arrives. The owner keeps a bounded sensor buffer during process startup and
bootstraps from the current sensor window after readiness. Recorded streaming
starts its playback clock after the same handshake. Startup latency therefore
does not turn into a burst of stale sensor packets.

클라이언트는 유효한 준비 신호를 한 번 받은 뒤 센서 패킷을 전달합니다.
실시간 소유자는 프로세스 시작 중 제한된 센서 버퍼를 유지하고, 준비 후
현재 센서 구간으로 초기화합니다. 기록 재생도 같은 준비 절차 후 시계를
시작하므로 시작 지연이 오래된 입력의 일괄 전송으로 이어지지 않습니다.

Protocol readiness only means the worker accepts input. It does not establish
navigation readiness, scale observability, calibrated noise or a loop-closed
map. State/map packets share the exposure timestamp and describe the active
local VIO window. Clock reversal, the declared IMU gap, saturation, image gaps
over 500 ms and backlog terminate the isolated worker. A ROS/application owner
must invalidate its entire segment, clear old samples and create a new worker
with a new frame; the worker never joins trajectories across a failure.

```sh
python tools/stream_recording.py --binary /absolute/new-build/build/basalt_stream \
  --inputs /absolute/recording/inputs \
  --recording-receipt /absolute/recording/receipt.json \
  --output /absolute/new-stream-result --speed 1
POSIM_SLAM_STREAM_BINARY=/absolute/new-build/build/basalt_stream \
  python -m unittest discover -s native/basalt -p 'test_*.py'
```

The paced-packet test includes native state/map outputs and thread/process
shutdown. Native clock/saturation/protocol tests require the explicit binary;
they are skipped in Python-only CI. ROS transport, live sensor timing, local-map
display and failure recovery need their own application acceptance. This target
is not selected by WWW-POSIM's default SLAM panel.

## 한국어

이 연구용 실행 파일은 Basalt의 square-root 시각·관성 추정기로 위치·속도·자세와
IMU 바이어스를 함께 갱신합니다. 앱에서 선택하는 Stella 공급자와 별개이며 ROS
노드·엔진 배포 공급자·루프 폐쇄 SLAM은 아닙니다.

버전 파일의 일곱 저장소를 고정하고 추정·광류 코어만 빌드합니다. GUI·데이터셋
로더·AprilTag 보정·지도 앱·카메라 드라이버는 연결하지 않습니다. SDK의 Eigen
5.0.1·TBB·fmt·OpenCV를 사용하며 Stella의 Eigen 3.4.0과 구분합니다.
기존 소스의 변경·미추적 파일·버전 불일치는 거부하며 체크아웃을 초기화하지 않습니다.

12초의 합성 6자유도 운동, 1초 정지 시작, 5 Hz 영상·200 Hz IMU로 시험합니다.
잡음이 있는 2차원 특징 관측과 원본 흑백 영상의 광류 경로를 각각 실행합니다.
추정기에는 보정값·픽셀/특징·원본 관성 채널만 전달하며 정답 위치·자세·3차원
타일 좌표는 주지 않습니다. 주입한 바이어스를 알려주지 않고 0에서 추정을 시작합니다.
출력 61개의 위치·회전 오차와 최종 바이어스 오차를 기록합니다. 축척 적합이나 경로
정렬은 하지 않습니다. 위 오차 한도는 짧은 합성 회귀 기준이며 실제 수중 정확도나
네이티브 월드·설치형 앱 검증을 대신하지 않습니다. 바이너리 배포 시 고정 소스,
복사된 고지와 SDK 고지를 보존해야 합니다.

`basalt_replay`는 보정 JSON·원본 IMU CSV·영상 목록·흑백 PNG를 받아 영상과 관성을
재생합니다. 래퍼는 파일·픽셀 해시를 확인합니다. 물리 IMU 축과 카메라 optical RDF
사이의 고정 장착값, 같은 시계의 나노초 시각과 명시적 잡음 설정을 사용합니다.
정답 위치·자세 채널은 입력하지 않습니다. 시각 역전·IMU 누락·포화·구간 미포함·
영상 크기 변경·외부 파일 경로를 거부하며 큐와 worker 수를 제한합니다.

별도 평가기는 정답 본체 자세를 IMU 원점으로 변환하고 실제로 구간을 덮는 자세만
보간합니다. 초기 yaw·이동 원점만 맞추며 중력 기울기와 축척 오차는 남깁니다.
ATE·1초 RPE·이동 거리와 평가 비율을 기록합니다. 기본값은 정답 구간이 부족하면
실패합니다. 명시적 경계 자르기 옵션은 앞뒤 상태 최대 2개만 제외하고 수를 표시하며,
내부 정답 누락 구간은 계속 거부합니다. 실시간 앱 공급자나 지도 출력은 별도 구현입니다.

`basalt_stream`과 `LiveClient`는 원본 IMU·흑백 영상의 비동기 실시간 전송을 제공합니다.
두 계산 worker, 영상 2개·IMU 256개·상태/지도 각각 4개의 네이티브 큐와 Python 입력
256패킷/4 MiB·출력 8개의 한도를 사용합니다. 출력 점군은 활성 로컬 VIO 창의 최대
1,000개 표본입니다. 큐 초과를 숨기고 버리거나 고장 전후 지도를 연결하지 않습니다.
프로토콜 준비는 항법 준비 판정이 아닙니다. 오류가 나면 앱 소유자가 구간 전체를
무효화하고 새 프레임으로 워커를 다시 만들어야 합니다. Python 전용 CI는 네이티브
실행 파일이 필요한 시험을 명시적으로 건너뜁니다. ROS 연결·실시간 센서·점군 표시·
복구는 앱에서 별도로 검증해야 하며 기본 SLAM 패널의 공급자는 바뀌지 않습니다.

The development owner accepts `--max-image-pixels` (default 120,000) and
`--max-image-rate` (default 10 Hz). It sends the original grayscale image and
calibration. The common native input boundary derives a bounded resolution,
applies OpenCV area resampling and scales pinhole intrinsics with the same
pixel-center transform. Odd dimensions are supported. Replay uses this same
boundary; recordings without a pixel budget keep their original resolution.
An 800×600 input at the default budget becomes 400×300. Basalt's default
feature grid and five-frame keyframe hold apply.

Current optimized landmark projections are matched to observed feature IDs.
Status reports their count, pixel RMSE and the count inside the backend's
Huber threshold. Local-window cardinality and display sampling do not establish
current visual support. Outputs distinguish visual measurements from inertial
prediction; either remains an experimental estimate. Sparse maps do not reset
the estimator. Sensor/worker faults still invalidate the whole segment.
These diagnostics do not qualify observability, covariance or navigation.

개발용 소유자는 `--max-image-pixels`(기본 120,000)와
`--max-image-rate`(기본 10 Hz)를 받습니다. 원본 흑백 영상과 보정을 전송하면
공통 네이티브 입력 경계가 처리 해상도를 계산합니다. OpenCV 영역 평균과 동일한
픽셀 중심 변환으로 영상·내부 보정을 함께 조정하며 홀수 해상도도 지원합니다.
재생도 같은 경계를 사용합니다. 픽셀 예산이 없는 기록은 원래 해상도를 유지합니다.
기본 예산에서 800×600 입력은 400×300이 됩니다. Basalt 기본 특징 격자와
다섯 프레임의 키프레임 대기 설정을 사용합니다.

현재 지도 특징의 최적화된 투영을 영상의 특징 ID와 대응해 개수·픽셀 RMSE·
백엔드 Huber 경계 안의 개수를 보고합니다. 로컬 지도와 표시 표본의 점 수를
현재 시각 관측의 근거로 사용하지 않습니다. 시각 관측이 있는 출력과 관성 예측을
구분하며 둘 다 실험용 추정값입니다. 희소한 지도만으로 추정기를 재시작하지 않습니다.
센서·워커 오류는 전체 구간을 무효화합니다. 이 진단은 관측 가능성·공분산·항법
준비를 보장하지 않습니다.

Native image-contract checks include odd/asymmetric dimensions, normalized-ray
invariance, area sampling, input budgets and known reprojection residuals:

```sh
POSIM_SLAM_INPUT_PROBE=/absolute/new-build/build/basalt_input_probe \
POSIM_SLAM_STREAM_BINARY=/absolute/new-build/build/basalt_stream \
  python -m unittest discover -s native/basalt -p 'test_*.py'
```


## Segment-local accumulated landmark snapshots

Optional calibration `max_feature_cells` (32–512) sets the spatial detection
budget on the processed image. The grid is at least 16 pixels and otherwise
`ceil(sqrt(processed_width * processed_height / max_feature_cells))`; aspect
ratios that cannot provide two detector cells along each image axis are rejected. Live and
replay share this calculation. The application default is 200 cells, independently
of the 1,000-point display sample. This budgets new-feature search cells, not the
number of retained landmarks. Without the field, upstream's 50-pixel grid remains
available for controlled baseline comparison. `tools/stream_recording.py` accepts
`--max-image-pixels` and `--max-feature-cells`, verifies the original input hashes,
and writes the overridden runtime calibration outside the read-only recording.
The client verifies the worker's reported raw/processed dimensions and detector
spacing before accepting a requested processing budget, so an older worker cannot
silently ignore it.

선택형 보정 값 `max_feature_cells`(32–512)는 처리 영상의 공간 검출 예산입니다.
격자는 최소 16픽셀이며 그 외에는 위 식으로 계산합니다. 각 영상 축에 검출 셀을 두 개
이상 배치할 수 없는 종횡비는 거부합니다. 실시간·재생 경로가 같은 계산을 사용합니다. 앱 기본값은
200셀이며 표시용 1,000점 표본과 독립적입니다. 새 특징을 탐색하는 셀 예산이며 유지하는
랜드마크 수의 상한은 아닙니다. 값이 없으면 비교 시험을 위한 원본 50픽셀 격자를
사용합니다. 기록 재생 도구의 영상·특징 셀 옵션은 원본 입력 해시를 검증한 뒤
읽기 전용 기록 밖에 실행용 보정을 작성합니다.
클라이언트는 워커가 보고한 원본·처리 해상도와 검출 간격을 확인한 뒤 요청한 처리
예산을 수락하므로 오래된 워커가 설정을 무시한 채 실행되지 않습니다.

The build-local Basalt export modification enumerates the landmark database by
landmark ID once. Multiple image observations do not create extra map points.
Distinct landmark IDs remain distinct even if their estimated coordinates coincide.

빌드 시 적용하는 Basalt 내보내기 수정은 랜드마크 데이터베이스에서 ID마다 한 번씩
점을 읽습니다. 여러 영상에서 관측해도 지도점 수를 늘리지 않습니다. ID가 다른
랜드마크는 추정 좌표가 같아도 별도 점으로 유지합니다.

The native archive admits an ID after full-weight reprojection support in three
separate exposures. Active IDs receive optimized coordinates; departed IDs keep
last coordinates. Unconfirmed departed tracks are discarded. Nonfinite or out-of-bound triangulation
candidates are rejected and counted without invalidating valid state estimation.
Previously retained IDs with invalid coordinates are removed and require fresh admission. Up to 100,000
tracks are retained, with oldest-track eviction and explicit eviction counts.
The archive lifetime is exactly one worker/coordinate segment. It provides
sparse odometry mapping without loop closure, dense reconstruction or retrospective
correction of departed points. New IDs on revisits may describe the same feature.

`LiveClient.snapshot(request_id)` exports all retained mature landmarks, separately
from the at-most-1,000-point display sample. The worker atomically writes
`worker-map.ply` beside runtime calibration and returns request ID, exposure
stamp and count. The ready contract is `segment_landmark_archive_ply_1`; PLY
scope is `accumulated_local_vio_landmarks`. `landmarks_window` reports active
estimator points; `landmarks_accumulated` reports retained mature points and
`archive_evicted` reports capacity eviction. Owners validate provenance and allow
one pending export. Recordings remain read-only. Legacy active-window PLY files
remain readable by `map_snapshot`, preserving their original scope.

네이티브 누적 저장소는 서로 다른 영상 세 장에서 정상 가중치의 재투영 검사를 통과한
ID를 채택합니다. 활성 ID는 최적화 좌표로 갱신하고 추정 창에서 빠진 점은 마지막
좌표를 유지합니다. 확인되지 않은 이탈 트랙은 버립니다. 유한하지 않거나 허용 범위를 벗어난 삼각측량
후보는 제외하고 거부 수를 보고하며 유효한 자세 추정은 유지합니다. 기존 ID도 좌표가
무효하면 제거하고 다시 관측 검사를 거칩니다. 최대 100,000개 트랙을
보존하고 가장 오래 유지된 트랙부터 제거하며 제거 수를 보고합니다. 워커·좌표 구간이
바뀌면 새 지도를 시작합니다. 희소 odometry 지도이며 루프 폐쇄, 조밀한 표면 복원,
이탈 점의 사후 보정은 제공하지 않습니다. 재방문 시 같은 특징에 새 ID가 생길 수 있습니다.

내보내기는 표시용 최대 1,000점과 별도로 보존된 성숙 랜드마크 전체를 담습니다.
워커는 실행용 보정 옆에 PLY를 원자적으로 기록하고 요청 ID·시각·점 수를 반환합니다.
소유자는 세션·프레임·단위를 검증하고 한 번에 하나의 내보내기를 처리합니다.
기존 활성 로컬 창 파일도 원래 범위를 유지한 채 검증할 수 있습니다.

## Joint Ping adapter development / Ping 공동 최적화 어댑터

`prepare_joint_headers.py` verifies the exact pinned Basalt landmark headers
and creates a build-local include overlay. The original checkout and its
BSD-3-Clause notices remain intact. The added interface exports unscaled visual
rows before QR and applies validated landmark increments in Basalt's own
stereographic-direction/inverse-distance coordinates. `basalt_joint_landmark_probe`
compares raw-row reduction and back substitution against Basalt. The live
optimizer does not yet select or consume Ping groups.

`prepare_joint_headers.py`는 고정된 Basalt 특징점 헤더의 해시를 확인하고
빌드 폴더 안에 확장 헤더를 생성합니다. 원본 체크아웃과 BSD-3-Clause 고지는
유지됩니다. 추가 인터페이스는 QR 이전의 원본 시각 잔차를 내보내고 Basalt의
방향·역거리 좌표에서 검증된 보정량을 적용합니다. `basalt_joint_landmark_probe`는
변수 제거와 역대입을 기존 Basalt 계산과 비교합니다. 실행 중인 최적화기는
아직 Ping 그룹을 선택하거나 사용하지 않습니다.

`ping_geometry.hpp` expresses mounted range and surface residual derivatives in
Basalt's map-frame left-rotation pose coordinates and stereographic landmark
coordinates. `basalt_ping_geometry_probe` checks them against numerical
derivatives using Basalt's actual pose update, including an inverted IMU pose.
The helper is not yet assembled into the streaming optimizer.

`ping_geometry.hpp`는 거리·표면 잔차 미분을 Basalt의 지도 기준 왼쪽 회전
증분과 특징점 방향·역거리 좌표로 표현합니다. `basalt_ping_geometry_probe`는
뒤집힌 IMU 자세를 포함해 실제 Basalt 자세 갱신의 수치 미분과 비교합니다.
아직 스트리밍 최적화기에 이 관측을 조립하지는 않습니다.

`ping_group.hpp` assembles raw Basalt visual rows and shared surface/range
constraints with native host-pose and landmark columns. It checks footprint
support, explicit noise, duplicate identity and exact-time pose association.
Landmark trial updates are preflighted as a group. `basalt_ping_group_probe`
checks a nonlinear trial with 25 landmarks and three host frames. Streaming
group selection, trial acceptance and marginalization are not yet connected.

`ping_group.hpp`는 원본 시각 잔차와 공유 표면·거리 제약을 호스트 자세 및
특징점 변수에 함께 조립합니다. 관측 영역, 명시한 잡음, 중복 식별자와
자세·측정 시각 일치를 검사하고 특징점 보정량을 모두 확인한 뒤 적용합니다.
`basalt_ping_group_probe`는 특징점 25개와 호스트 프레임 3개로 비선형 보정
단계를 검증합니다. 스트리밍 그룹 선택·보정 수락·주변화는 아직 연결되지 않았습니다.

`ping_patch.hpp` selects candidate planes from active visual landmarks supported
by at least three observations. Replay enables this with `--ping-patches` and
writes `<states>.patches.jsonl`. The selector requires eight plane inliers,
uses a 0.03 m surface tolerance and checks every modeled beam intersection
against the observed hull. Its nearest-sample support radius is 1.5 times the
median projected nearest-neighbor spacing, capped at 10% of predicted range.
Spacing is measured before selecting up to 64 solver landmarks. This defines
the interpolation resolution; holes smaller than that resolution cannot be
excluded. Observed foreground points reject a farther surface. These are
candidate diagnostics; replay still reports `ping_fused: false`.

`ping_patch.hpp`는 세 번 이상 관측된 현재 시각 특징점에서 평면 후보를
선택합니다. 재생 시 `--ping-patches`를 사용하면 `<states>.patches.jsonl`에
진단 결과를 저장합니다. 평면 내점은 최소 8개이며 표면 허용 오차는 0.03m입니다.
각 빔 교점이 관측 영역의 볼록 껍질 안에 있는지 검사합니다. 최근접 특징점
지원 반경은 평면에 투영한 최근접 이웃 간격 중앙값의 1.5배이며, 예측 거리의
10%를 넘지 않습니다. 간격은 최적화용 특징점을 최대 64개로 줄이기 전에
계산합니다. 이 값은 보간 해상도이며 그보다 작은 구멍은 배제할 수 없습니다.
앞쪽 물체의 관측점이 있으면 뒤쪽 평면을 거부합니다. 후보 진단 단계이므로
재생 결과의 `ping_fused`는 계속 `false`입니다.

`ping_return.hpp` computes the expected minimum and variance of 1–256 independent
Gaussian ray ranges with a common positive standard deviation and fixed bias.
It integrates each ray's winning density with 512 Simpson intervals over
±10 standard deviations around the shortest noiseless range. Winning
probabilities give derivatives of the expected return with respect to ray
ranges. It rejects geometry within ten noise standard deviations of clipping
limits. Missing returns, shared random bias and correlated noise require a
different model. `basalt_ping_return_probe` checks analytic one/two-ray cases,
finite-difference derivatives and a seeded 200,000-sample simulation. This
helper is not yet used by the streaming optimizer; it is not real-sonar
calibration or a model of surface uncertainty.

`ping_return.hpp`는 동일한 양의 표준편차와 고정 바이어스를 갖는 독립 가우스
잡음 광선 1~256개의 최솟값 평균·분산을 계산합니다. 가장 짧은 무잡음 거리의
±10 표준편차 구간을 512개 Simpson 구간으로 적분합니다. 각 광선이 최솟값이
될 확률은 광선 거리별 기대 반환값의 미분입니다. 거리 제한에서 잡음 표준편차의
10배 이내인 형상은 거부합니다. 누락 반환, 공통 확률적 바이어스와 상관 잡음에는
별도 모델이 필요합니다. 검증은 광선 1·2개의 해석적 정답, 수치 미분과 고정 시드
난수 실험 20만 회를 사용합니다. 아직 스트리밍 최적화기에 연결되지 않았으며,
실제 소나 보정이나 표면 불확실성 모델을 대신하지 않습니다.

`expectedPingRange` combines winning probabilities with mounted ray-plane
Jacobians and returns the conditional noise variance. Equal noiseless ray
ranges are supported. A `PingGroupObservation` may select this mean model with
`per_ray_sigma`; its `sigma` remains a separately specified, fixed residual
weight. This is a Gaussian moment approximation to the minimum-return
distribution. The group does not recompute weights during a trial or implement
an exact minimum-return likelihood. Reference-mode Jacobians evaluate all
ray derivatives and winning probabilities at the same fixed pose/plane
reference. Surface rows keep their separate explicit noise scale.

`expectedPingRange`는 각 광선이 최솟값이 될 확률과 장착 위치를 반영한
광선·평면 미분을 결합하고 조건부 잡음 분산을 반환합니다. 무잡음 거리가 같은
광선도 처리합니다. `PingGroupObservation`의 `per_ray_sigma`를 지정하면
이 기대값 모델을 사용하며, `sigma`는 별도로 지정한 고정 잔차 가중치입니다.
이는 최솟값 분포의 평균·분산을 이용하는 가우스 근사입니다. 보정 시도 중에
가중치를 다시 계산하지 않으며 정확한 최솟값 우도를 구현하지 않습니다.
기준 상태 모드에서는 모든 광선 미분과 선택 확률을 동일한 기준 자세·평면에서
계산합니다. 표면 잔차의 잡음 크기는 별도로 명시합니다.

For marginalization, `PingLinearization` accepts Basalt's retiring-host and
retiring-landmark filters. The owner must include every landmark of each
retiring Ping group in that selection, and later retire the same factors after
installing the prior. Partial group selection fails construction. Independent
visual tracks outside the selection remain excluded from the new prior. These
constructor filters do not implement the live retirement schedule themselves.

주변화 시 `PingLinearization`은 Basalt의 제거 대상 호스트·특징점 필터를
전달받습니다. 소유자는 제거할 Ping 그룹의 특징점을 모두 선택에 포함하고,
사전정보를 설치한 뒤 같은 관측들을 제거해야 합니다. 그룹 일부만 선택하면
생성을 거부합니다. 선택되지 않은 독립 시각 특징점은 새 사전정보에서 제외합니다.
생성자의 필터 전달 기능만으로 실시간 제거 일정이 구현되는 것은 아닙니다.

`planPingRetirement` reads actual landmark hosts and visual target frames,
plus each group's declared Ping observation poses. Losing one member or
removing any dependent pose selects the entire group. It returns the complete
landmark, plane, Ping stamp and required-frame sets without changing the graph.
Required frames can be newer than Basalt's usual marginalization boundary;
the caller must include and retain their columns in the new prior. Planning
during a trial, missing dependencies and duplicate ownership are rejected.
The linearization checks that built Ping stamps match the declared dependency
set. The owner still installs the prior and retires exactly those factors.

`planPingRetirement`는 실제 특징점의 호스트·시각 관측 대상 프레임과 그룹에
명시한 Ping 관측 자세를 읽습니다. 구성 특징점을 잃거나 의존 자세가 제거되면
그룹 전체를 선택합니다. 그래프를 변경하지 않고 특징점·평면·Ping 시각·필요
프레임 집합을 반환합니다. Basalt의 기본 주변화 경계보다 최신 프레임도 포함될
수 있으므로 호출자는 해당 열을 새 사전정보에 유지해야 합니다. 보정 시도 중
계획, 누락 의존관계와 중복 소유는 거부합니다. 선형화는 실제로 구성한 Ping
시각이 명시한 의존관계와 일치하는지 검사합니다. 사전정보 설치와 동일 관측의
제거는 여전히 소유자의 책임입니다.

`planPingMarginalLayout` builds source and retained orders from the actual
estimator states, retaining recent required frames beyond the usual marginal
boundary. Pose-only blocks precede full inertial blocks as in the optimizer.
It supports removing a state and demoting a 15-variable inertial state to its
six pose variables. The returned IMU selection consumes only intervals whose
endpoint loses velocity/bias variables. Intervals between fully retained states
remain live. Missing frame dependencies and invalid transitions are rejected;
the function does not install a prior or mutate the estimator.

`planPingMarginalLayout`은 실제 추정 상태에서 주변화 입력·유지 상태의 열
순서를 구성하며, 기본 경계보다 최신인 필요 프레임도 유지합니다. 최적화기와
같이 자세 전용 블록을 전체 IMU 상태 블록 앞에 배치합니다. 상태 삭제와
15변수 IMU 상태에서 자세 6변수만 남기는 전환을 지원합니다. 반환하는 IMU
선택에는 끝점의 속도·바이어스 변수가 제거되는 구간만 포함합니다. 전체 상태로
남는 두 끝점 사이 구간은 실시간 관측으로 유지합니다. 누락 프레임과 잘못된
전환은 거부하며, 사전정보 설치나 추정 상태 변경은 수행하지 않습니다.

`installPingMarginalPrior` validates retirement dependencies, state layout and
the supplied square-root prior before changing live containers. It stages state
demotion/removal, landmark and plane retirement, group removal and consumed IMU
removal on copies. Retained reference states are frozen if needed; existing
linearized references are preserved. It installs the prior and invalidates
cached Ping rows through the owner's revision. Call it on the owner thread
after building the prior from those exact factors, with no intervening state
edits and after releasing old linearizations. The caller separately maintains
keyframe IDs, optical-flow history and queues. This API is not yet called by
the streaming estimator's schedule.

`installPingMarginalPrior`는 실제 컨테이너를 바꾸기 전에 제거 의존관계,
상태 구성과 제곱근 사전정보를 검증합니다. 복사본에서 상태 전환·삭제,
특징점·평면·그룹·반영된 IMU 구간 제거를 준비합니다. 유지할 기준 상태는
필요하면 고정하고 기존에 고정된 기준은 보존합니다. 사전정보 설치 후 소유자의
개정 번호를 올려 캐시된 Ping 행렬을 무효화합니다. 정확히 해당 관측들로
사전정보를 만든 뒤, 중간 상태 변경 없이 이전 선형화 객체를 해제하고 소유자
스레드에서 호출해야 합니다. 키프레임 ID·영상 추적 이력·큐는 호출자가 별도로
관리합니다. 아직 스트리밍 추정기의 실행 순서에는 연결되지 않았습니다.

The build-local pinned `SqrtKeypointVioEstimator` has a pre-marginalization callback
configured before input submission. The streaming worker uses it to deliver an
exact image/Ping pair on the estimator thread after the current visual/IMU optimization and before
marginalization. `ping_owner_received` reports this delivery in
state output; it is false without a matching Ping. `ping_fused` remains false:
the callback currently records delivery and the original optimization schedule
still runs. Upstream headers/source are hash-checked and remain unmodified.

빌드용으로 복사한 고정 버전 `SqrtKeypointVioEstimator`에는 입력 제출 전에
설정하는 주변화 직전 콜백이 있습니다. 스트리밍 워커는 현재 영상·IMU 최적화 후,
주변화 전에 동일 시각 영상·Ping 쌍을 추정기 스레드에 전달합니다.
상태 출력의 `ping_owner_received`가 이 전달 여부이며 대응 Ping이 없으면
거짓입니다. 현재 콜백은 전달만 기록하고 기존 최적화 순서를 실행하므로
`ping_fused`는 계속 거짓입니다. 원본 헤더·소스는 해시로 확인하며 수정하지
않습니다.

When `ping-calibration.json` is present beside the worker calibration, the
owner callback selects candidate patches from current raw landmarks. A point
needs a current full-weight reprojection and support in three distinct frames.
Replay and streaming share `pingInputCalibration` and `selectPingPatch`.
`ping_candidate` reports support counts and candidate geometry; it does not
mean the range has been accepted or fused. Candidate fitting is independent
of display/archive point limits. Initial depth/scale errors may still produce
a geometrically coherent candidate inconsistent with the measured range.

워커 보정 파일 옆에 `ping-calibration.json`이 있으면 소유자 콜백이 현재
원본 특징점에서 평면 후보를 선택합니다. 현재 영상에서 완전 가중치를 받는
재투영과 서로 다른 세 프레임의 지지가 필요합니다. 기록 재생과 스트리밍은
`pingInputCalibration`과 `selectPingPatch`를 공유합니다. `ping_candidate`는
지원 점 개수와 후보 형상을 나타내며 거리 관측의 수락·융합을 의미하지 않습니다.
후보 적합은 표시·보관 점군의 점 개수 제한과 독립적입니다. 초기 깊이·축척
오차가 있으면 형상상 일관된 후보도 실제 거리와 다를 수 있습니다.

`makeOwnedPingGroup` builds the native group callback from an already owned
plane, raw landmark IDs and exact-time Ping observations. It copies measurement
values, calibration, weights and membership, then reads current geometry on each
linearization. Pose/plane references stay consistent with the existing prior.
It supports deterministic-minimum and expected-noisy-minimum means. Missing
hosts/landmarks, duplicate observation stamps and invalid weights are rejected.
The caller owns group registration and retirement; this builder does not by
itself activate the streaming coupled optimizer.

`makeOwnedPingGroup`은 소유 중인 평면, 원본 특징점 ID와 동일 시각 Ping
관측에서 네이티브 그룹 콜백을 만듭니다. 측정값·보정값·가중치·구성 목록을
복사한 뒤 선형화할 때마다 현재 기하를 읽습니다. 자세·평면 기준은 기존
사전정보와 일치하게 유지합니다. 무잡음 최솟값과 잡음을 반영한 기대 최솟값
모델을 지원합니다. 누락 호스트·특징점, 중복 관측 시각과 잘못된 가중치는
거부합니다. 그룹 등록·제거는 호출자가 관리하며 이 생성 함수만으로 스트리밍
공동 최적화가 활성화되는 것은 아닙니다.

`optimizePingGroups` runs bounded damped joint updates using visual, IMU, prior
and Ping costs. It rebuilds rows after every rejected trial, applies state and
landmark/plane changes in one `JointPingTrial`, and accepts only a finite actual
cost decrease with positive predicted decrease. Unsupported trial geometry
restores the complete state before increasing damping. Defaults are six outer
iterations and six attempts per iteration. The report gives accepted/rejected
counts, costs and termination reason; reaching the iteration limit is not a
convergence claim. This routine still needs activation in the streaming group
schedule.

`optimizePingGroups`는 시각·IMU·사전정보·Ping 비용을 함께 사용하는 제한된
감쇠 공동 보정을 실행합니다. 거부된 시도 뒤에는 행렬을 다시 만들고, 상태와
특징점·평면 변경을 하나의 `JointPingTrial`로 처리합니다. 유한한 실제 비용
감소와 양의 예측 감소가 있을 때만 수락합니다. 보정한 형상이 관측 영역을
벗어나면 전체 상태를 복원한 뒤 감쇠를 높입니다. 기본값은 외부 반복 6회와
반복당 시도 6회입니다. 수락·거부 횟수, 비용과 종료 사유를 반환하며 반복
한도 도달은 수렴을 의미하지 않습니다. 스트리밍 그룹 실행 순서에서 이
루틴을 활성화하는 연결은 아직 필요합니다.

`ping_state.hpp` stores up to eight local planes in the actual Basalt estimator
backup/restore lifecycle. `JointPingTrial` retains accepted finite cost decreases
and restores poses, inertial states, landmarks and planes on rejection or
exception. Associations stay fixed during a trial; rebuild linearization rows
after completion. `basalt_ping_state_probe` tests these transitions on the native
estimator. Live grouped-Ping optimization remains unconnected.

`ping_state.hpp`는 최대 8개 국소 평면을 실제 Basalt 추정기의 백업·복원에
포함합니다. `JointPingTrial`은 수락한 비용 감소만 유지하고 거부·예외 시
자세, 관성 상태, 특징점과 평면을 복원합니다. 보정 중 관측 연결을 고정하고
종료 후 선형화 행렬을 다시 구성해야 합니다. `basalt_ping_state_probe`가
실제 추정기로 이를 검증하며, 실시간 Ping 공동 최적화 연결은 남아 있습니다.


### Exact image/Ping transport / 영상·Ping 시각 정합 전송

`LiveClient.image(..., ping=(stamp_ns, range_m))` sends an optional single-range
observation atomically with its image. The timestamps must match exactly. The
worker advertises `image_ping_transport: exact_stamp_range_v1`; unsupported workers
are rejected before sending. Packet type 5 carries the image header, Ping timestamp
and float64 range, then grayscale pixels. State output echoes `ping_observation`
only at that exact timestamp. Pending observations are bounded to 64 per process.
`ping_fused: false` identifies this transport stage; the live optimizer does not
consume these observations yet. Sensor-specific bounds and calibration belong to
the caller; the wire format accepts finite positive ranges up to 1000 m.

`LiveClient.image(..., ping=(stamp_ns, range_m))`은 영상과 단일 거리 관측을
하나의 패킷으로 전달합니다. 두 타임스탬프는 정확히 같아야 합니다. 워커는
`image_ping_transport: exact_stamp_range_v1` 지원 여부를 알리며, 지원하지 않으면
전송 전에 거부합니다. 패킷 유형 5에는 영상 헤더, Ping 시각과 float64 거리,
흑백 픽셀이 들어갑니다. 자세 출력의 `ping_observation`에는 동일 시각의 관측만
포함됩니다. 대기 관측은 프로세스당 64개로 제한됩니다. `ping_fused: false`는
입력 전달 단계임을 나타내며, 실시간 최적화기는 아직 이 관측을 사용하지 않습니다.
센서별 거리 범위와 보정은 호출자가 검증합니다. 전송 형식은 1000 m 이하의
유한한 양수 거리를 허용합니다.


`posim_slam.exact_pair.ExactImagePing` joins independently delivered images and
single ranges by exact nanosecond timestamps. It waits up to 0.15 seconds for a
missing pair, then releases the image without a range. Invalid returns are
represented by `None`; late observations never modify an already released image.
It bounds pending input to four images and 64 ranges. The single-threaded owner
must replace the buffer when its sensor clock or estimator segment resets.

`posim_slam.exact_pair.ExactImagePing`은 따로 도착하는 영상과 단일 거리를
나노초 타임스탬프가 정확히 같은 경우에만 묶습니다. 짝이 없으면 최대 0.15초
기다린 뒤 거리 없이 영상을 내보냅니다. 무효 반환값은 `None`으로 표현하며,
늦게 온 관측으로 이미 내보낸 영상을 수정하지 않습니다. 대기 입력은 영상 4개와
거리 64개로 제한됩니다. 단일 스레드에서 사용하는 호출자는 센서 시계나 추정
구간이 초기화될 때 버퍼도 교체해야 합니다.


### Joint marginal prior / 공동 주변화 사전정보

`ping_marginal.hpp::marginalizeJointSystem` eliminates shared plane/landmark
coordinates and departing states together, returning Basalt `MargLinData<double>`
in square-root form. Retained blocks may contain a six-coordinate pose or a
15-coordinate pose/velocity/bias state; a departing inertial state can retain
only its pose. Explicit block orders support reordered retained columns and
reject gaps, overlaps and unknown states. The residual is shifted to Basalt's
existing linearization point using the supplied current state delta. No small
artificial prior is added in unobservable directions. State-only rows from a
previous joint group reduction are also accepted. If all state columns remain,
the adapter preserves the rows and only reorders/rebases them.

The caller must include each raw factor once, retire its consumed observations,
and supply consistent first-estimate Jacobians. This adapter does not choose
patches or activate the live optimizer. `basalt_ping_group_probe` checks joint
covariance and a subsequent measurement against the full problem;
`basalt_ping_marginal_probe` compares mixed inertial/pose blocks with Basalt's
marginalization routine. Basalt omits a constant term from prior costs, so
`JointPingTrial` accepts finite decreases even when the reported costs are negative.

`ping_marginal.hpp::marginalizeJointSystem`은 공유 평면·랜드마크 좌표와
제거할 상태를 함께 주변화해 Basalt의 제곱근 형식 `MargLinData<double>`을
반환합니다. 유지할 블록은 자세 6개 좌표 또는 자세·속도·바이어스 15개 좌표를
포함합니다. 관성 상태에서 자세만 남길 수도 있습니다. 명시적인 블록 순서는
유지할 열의 재배치를 지원하고, 빈 구간·중복·알 수 없는 상태를 거부합니다.
현재 상태 변화량을 사용해 잔차를 Basalt의 기존 선형화 기준점으로 옮깁니다.
관측할 수 없는 방향에 임의의 작은 사전정보를 추가하지 않습니다.
공동 그룹 제거 후 상태 열만 남은 행렬도 받습니다. 모든 상태 열을 유지하는
경우에는 행을 보존하고 열 순서와 잔차 기준점만 조정합니다.

호출자는 원본 관측을 한 번씩만 포함하고, 사전정보에 반영한 관측을 제거하며,
일관된 최초 추정 기준 야코비안을 제공해야 합니다. 이 어댑터는 평면 연결을
선택하거나 실시간 최적화 경로를 활성화하지 않습니다. 두 네이티브 검증기는
전체 문제의 공분산·추가 관측 결과와 Basalt의 주변화 함수를 비교합니다.
Basalt는 사전정보 비용에서 상수항을 생략하므로 `JointPingTrial`은 보고된 비용이
음수여도 유한한 비용 감소를 수락합니다.


### Coupled linearization / 공동 선형화

`ping_linearization.hpp::PingLinearization` implements Basalt's
`LinearizationBase<double,6>` interface for fixed Ping groups. It moves selected
landmark blocks out of independent QR and reduces their raw visual, shared
surface and range rows together. Other visual landmarks, IMU blocks and the
existing prior retain Basalt's path. Both square-root and normal-equation outputs
contain each selected visual block once. Robust visual costs remain separate
from the Gaussian surface/range costs.

Back substitution requires `JointPingTrial`. Duplicate plane/landmark ownership
and duplicate Ping timestamps across groups are rejected. Applying a trial
invalidates its cached reduction. Base-state restoration advances a revision,
so even matrices rebuilt at the trial state cannot be reused after rollback.
The caller relinearizes before retrying. `basalt_ping_linearization_probe` checks
this interface with 25 grouped landmarks, one independent landmark, two actual
IMU preintegrations and an existing prior. Its accepted trial lowers cost from
3.50 to 0.43. This numerical fixture does not select live patches or activate
streaming Ping fusion; live association, FEJ consistency and observation
retirement remain caller responsibilities.

`ping_linearization.hpp::PingLinearization`은 고정된 Ping 그룹에 대해
Basalt의 `LinearizationBase<double,6>` 인터페이스를 구현합니다. 선택한
랜드마크를 개별 QR 처리에서 분리하고 원본 영상·공유 표면·거리 잔차를 함께
축약합니다. 나머지 영상 랜드마크, IMU 블록과 기존 사전정보는 Basalt 경로를
유지합니다. 제곱근 행렬과 정규방정식 모두 선택한 영상 관측을 한 번만 포함합니다.
강건 영상 비용과 가우시안 표면·거리 비용을 구분합니다.

역대입에는 `JointPingTrial`이 필요합니다. 평면·랜드마크의 중복 소유와 그룹 간
중복 Ping 시각을 거부합니다. 갱신 적용 후에는 저장된 축약 행렬을 무효화합니다.
상태 복구 시 변경 번호가 증가하므로 갱신 후보에서 다시 만든 행렬도 복구 후에는
재사용할 수 없습니다. 재시도 전에 다시 선형화해야 합니다. 네이티브 검증기는
그룹 랜드마크 25개, 별도 랜드마크 1개, 실제 IMU 사전적분 2개와 기존 사전정보를
사용하며, 수락한 갱신의 비용은 3.50에서 0.43으로 감소합니다. 이 수치 검증은
실시간 평면 선택이나 스트리밍 Ping 융합을 활성화하지 않습니다. 실시간 관측 연결,
최초 추정 기준 야코비안의 일관성, 반영한 관측의 제거는 호출자가 관리해야 합니다.


The coupled route requires an explicit plane reference, reference host geometry
and reference Ping pose. Residuals and footprint checks use the current state;
Jacobians use the host/measurement `getPoseLin()` states and the plane's fixed
reference. Landmark coordinates remain current, matching the visual block's
landmark parameterization. The active beam is chosen at the current state and
that same ray is differentiated at the reference. Mixed/missing references are
rejected. `PingPlaneStore.set` initializes a plane once; increments accumulate
in its fixed two-coordinate rotation chart plus distance. Backup/restore includes
that reference and accumulated delta.

The native probe checks six global-frame gauge directions of the visual/Ping
group after an accepted state update. Its reference-consistent Jacobian has
`||J N||` below 3e-13; using current-state auxiliary derivatives with the existing
reference gives about 0.89 in the same fixture. This is a group consistency test,
not a claim that the entire gravity-aided VIO has six unobservable directions or
that live trajectory covariance has been validated.

공동 경로에는 평면 기준점, 호스트 자세 기준점에서 계산한 랜드마크 기하와
Ping 자세 기준점이 필요합니다. 잔차와 관측 영역 검사는 현재 상태를 사용하고,
야코비안은 `getPoseLin()` 자세와 고정된 평면 기준점을 사용합니다. 랜드마크
좌표는 영상 블록의 매개변수와 맞도록 현재 값을 사용합니다. 현재 상태에서
선택한 빔을 기준점에서도 그대로 미분합니다. 기준점이 누락되거나 섞이면 거부합니다.
`PingPlaneStore.set`은 평면을 한 번 초기화하며, 증분은 고정된 두 회전 좌표와
거리 좌표에 누적합니다. 백업·복구에는 기준점과 누적 변화량도 포함됩니다.

네이티브 검증은 상태 갱신 후 영상·Ping 그룹의 전역 좌표계 자유도 6개를
검사합니다. 일관된 기준점의 야코비안에서는 `||J N||`이 3e-13 미만이며,
기존 기준점에 현재 상태의 보조 잔차 미분을 섞으면 같은 검증에서 약 0.89입니다.
이는 그룹의 기준점 일관성 검사입니다. 중력을 사용하는 전체 VIO의 비관측 방향이
6개라는 뜻이나, 실시간 궤적 공분산을 검증했다는 뜻은 아닙니다.
