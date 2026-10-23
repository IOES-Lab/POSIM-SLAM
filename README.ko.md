# POSIM-SLAM

[English](README.md) · [제3자 고지](THIRD_PARTY.md)

POSIM 애플리케이션에서 사용하는 독립 센서 추정 라이브러리입니다. Python
알고리즘은 센서 측정 배열과 보정된 변환을 받습니다. WWW-POSIM은 세션, 차량
구성, 실행 관리, UI와 지도 다운로드를 담당합니다.

패키지 버전은 **0.1.0**, 음향/DVL 융합 알고리즘 버전은 **0.2.0**입니다.

| 구성 | 방법과 현재 범위 |
|---|---|
| 시각 공급자 | Stella VSLAM 0.7.0 단안 ORB 특징과 g2o/Eigen 최적화; ROS 2 어댑터와 정확한 의존성 버전 포함; 임의 축척; 빈 사전 구성의 루프 폐쇄·재위치 추정은 미검증 |
| 시각 + 단일 Ping | 강건한 국소 평면 적합, 동기화 거리와 장착 위치·바이어스 보정, 안정된 중앙값으로 미터 축척 추정; 적합한 하향 시야와 국소 평면 해저 필요 |
| 시각 + DVL | 바닥 잠금 속도 적분과 축·회전 레버암 보정; 미터 축척 보정이며 누적 오차의 공동 보정은 아님 |
| 음향 전처리 | OpenCV Shi–Tomasi 코너, 양방향 피라미드 Lucas–Kanade 광류, 소나 팬 평면의 SVD/RANSAC 강건 2D 정합 |
| 음향 + DVL | 구간을 덮는 속도 적분으로 이동량 추정; 음향 회전·특징과 이동량 일관성 검사; 실험적인 평면 오도메트리이며 높이 미확정 |
| 수면 필터 | 수압·자세로 평균 수면을 보정하고 미터 지도를 자름; 개별 수면 에코 분류는 지원하지 않음 |
| 관성 코어 | 원본 각속도·가속도의 중점 SO(3) 전적분, 바이어스 Jacobian, 15상태 오차 공분산; 완전한 융합 추정기가 아닌 준비 코드 |

실행되는 시각·음향 모드는 아직 관성 전적분 코어를 사용하지 않습니다.
시뮬레이터의 정답 위치·자세를 위치 추정 관측으로 사용하지 않습니다.
소나 강도 특징점은 분류된 3D 장애물이 아닙니다.

## 설치와 사용

Python 3.10 이상과 NumPy가 필요합니다. 순수 기하 알고리즘에는 ROS·Gazebo·
WWW-POSIM이 필요하지 않습니다.

```sh
git clone https://github.com/IOES-Lab/POSIM-SLAM.git
cd POSIM-SLAM
python -m venv .venv
. .venv/bin/activate
python -m pip install -e .
```

Windows에서는 `.venv\Scripts\activate`로 활성화합니다.

```python
from posim_slam.inertial import ImuWindow

imu = ImuWindow()
for i in range(201):
    imu.add(i / 200., [0., 0., 0.], [0., 0., 9.80665])
delta = imu.preintegrate(0., 1., gyro_bias=[0., 0., 0.],
                        accel_bias=[0., 0., 0.],
                        noise_density=[.002, .02, .0001, .001])
```

잡음 수치는 사용 예시이며 하드웨어 보정값이 아닙니다. 전적분 결과는 중력과 초기
속도를 포함하지 않습니다. 후속 추정기가 독립 관측으로 위치·속도·바이어스를 함께
갱신해야 합니다.

음향 영상 추적에는 검증된 OpenCV 4.13.0 네이티브 공급자 또는 연구용 추가 패키지
(`python -m pip install -e '.[research]'`)가 필요합니다. 네이티브 공급자는
`POSIM_SLAM_ACOUSTIC_LIBRARY`를 사용하며 기존 `WWOS_ACOUSTIC_NATIVE_LIBRARY`도
지원합니다. SDK, 소스·라이브러리 해시와 ABI를 검사합니다. 연구용 cv2 설치는 실제
네이티브 실행 검증을 대신하지 않습니다.

## 네이티브 빌드

일치하는 OpenCV 4.13.0 개발 환경에서 실행합니다.

```sh
cmake -S native -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel 2
cmake --install build --prefix install
```

시각 ROS 2 어댑터에는 고정된 Stella 의존성 빌드도 필요합니다. 해당 환경에서
`POSIM_SLAM_BUILD_VISUAL=ON`을 설정합니다. 공급자 실행·경로 이동 검증은 CMake
빌드 성공과 별개입니다. 현재 음향 공급자 기록은 macOS `.dylib` 레이아웃을
사용하며 Windows 네이티브 공급자 실행은 미검증입니다.

## 무결성과 시험

```sh
python -m pip install -e '.[research]'
python -m unittest discover -s tests
```

시험은 해석 가능한 축척·회전·가속, 구간 데이터, 시각·세션 초기화, 비정상 측정,
수면 자르기, 바이어스 미분과 공분산을 확인합니다. `test_integrity.py`는 소스 버전,
원본 고지 해시와 WWW-POSIM 애플리케이션 import 부재를 확인합니다. 장시간 SLAM
정확도는 별도 센서 실행과 독립 정답 비교로 검증해야 합니다.

소스 버전과 패치는 `src/posim_slam/data/visual`, 네이티브 어댑터는
`src/posim_slam/data/native`에 있습니다. 이 자료와 제3자 고지는 Python 패키지에도
포함되어 체크아웃·wheel·복원 환경에서 같은 소스를 사용합니다.

자체 코드는 [Apache-2.0](LICENSE)입니다. [NOTICE.md](NOTICE.md)와 제3자 라이선스를
확인하세요. ORB-SLAM3 코드·특징 사전·학습 가중치는 포함하지 않습니다.
Basalt와 Kimera는 이 패키지에서 실행되는 백엔드가 아닙니다.

## 실험적인 시각·관성 연구

[Headless Basalt 대상](native/basalt/README.md#한국어)은 버전을 고정한 square-root
VIO 코어를 빌드하고 잡음이 있는 6자유도 특징·영상과 원본 IMU 회귀를 실행합니다.
이 독립 실행 파일에서는 위치·속도·자세와 바이어스를 함께 갱신합니다. WWW-POSIM이
선택하는 백엔드나 기본 Python·네이티브 공급자 빌드에는 포함하지 않습니다. 실제
월드의 시각·보정·복구와 배포 패키지는 별도로 검증해야 합니다.
