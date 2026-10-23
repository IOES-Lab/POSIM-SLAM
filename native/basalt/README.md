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
