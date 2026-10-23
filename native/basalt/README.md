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
