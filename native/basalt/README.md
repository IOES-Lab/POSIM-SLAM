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
over stdin and emits JSON states/local-window point samples over stdout. The
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


## Complete local-window snapshots

The build-local Basalt export modification enumerates the landmark database by
landmark ID once. Multiple image observations do not create extra map points.
Distinct landmark IDs remain distinct even if their estimated coordinates coincide.

빌드 시 적용하는 Basalt 내보내기 수정은 랜드마크 데이터베이스에서 ID마다 한 번씩
점을 읽습니다. 여러 영상에서 관측해도 지도점 수를 늘리지 않습니다. ID가 다른
랜드마크는 추정 좌표가 같아도 별도 점으로 유지합니다.

`LiveClient.snapshot(request_id)` requests a complete optimized active-window
PLY instead of the 1,000-point display sample. The worker writes `worker-map.ply`
atomically beside its runtime calibration file and replies with request ID,
exposure timestamp and valid point count. At most 100,000 finite points are
accepted. An empty map returns a nonfatal unavailable response. Owners allow one
outstanding export, read/validate that response before another request, and add
session, frame and meter-unit provenance using `posim_slam.map_snapshot`.
The file explicitly identifies its active local-window scope and unqualified
navigation state. It does not accumulate old marginalised landmarks into a
claimed loop-closed map. Input recordings remain read-only; the paced test tool
materializes runtime calibration in its output directory before requesting PLYs.

`LiveClient.snapshot(request_id)`는 표시용 1,000개 표본 대신 현재 최적화된 로컬
창 전체의 PLY를 요청합니다. 워커는 실행용 보정 파일 옆의 `worker-map.ply`를 원자적으로
기록하고 요청 ID·노출 시각·유효 점 수를 반환합니다. 유효한 점은 최대 100,000개이며
빈 지도는 워커 재시작 없이 사용 불가 응답을 반환합니다. 소유자는 내보내기를 하나씩
처리하고 결과를 검증한 뒤 `posim_slam.map_snapshot`으로 세션·프레임·미터 단위 고지를
추가합니다. 파일은 활성 로컬 창이라는 범위와 미검증 항법 상태를 기록합니다.
오래된 주변화 랜드마크를 모아 전역 루프 폐쇄 지도라고 표시하지 않습니다.
