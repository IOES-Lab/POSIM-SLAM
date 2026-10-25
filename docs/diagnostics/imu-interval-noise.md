# IMU interval noise / IMU 구간 잡음

The native research worker uses continuous white-noise densities from its
calibration for every actual integration duration. Basalt's midpoint attitude
and state propagation remain in use. For acceleration spectral covariance
`Qa` at a fixed orientation, the position/velocity noise block is
`Qa × [[dt³/3, dt²/2], [dt²/2, dt]]`.

네이티브 연구용 작업자는 보정 파일의 연속 백색잡음 밀도를 실제 적분 구간에
적용합니다. 자세와 상태 전파는 Basalt의 중간점 근사를 사용합니다. 자세가
일정할 때 가속도 스펙트럼 공분산 `Qa`에 대한 위치·속도 잡음 블록은
`Qa × [[dt³/3, dt²/2], [dt²/2, dt]]`입니다.

The added position term is `R Qa Rᵀ dt³/12`, where `R` is the midpoint
attitude. It represents within-interval acceleration variation after accounting
for the interval's mean velocity increment. It is not a minimum variance,
position prior, step clamp or extra measurement. This first-order model retains
Basalt's midpoint approximation for angular-rate noise and acceleration coupling.
It does not model coloured sensor noise, interpolation correlation, saturation
or unresolved motion between samples.

추가 위치 항은 중간점 자세 `R`에 대한 `R Qa Rᵀ dt³/12`입니다. 구간의 평균
속도 변화로 설명하고 남는 구간 내부 가속도 변화를 나타냅니다. 최소 분산·위치
사전정보·이동량 제한·추가 측정값이 아닙니다. 각속도 잡음과 가속도의 결합은
Basalt의 중간점 근사를 유지합니다. 유색 잡음·보간 상관·포화·샘플 사이의
미해상 움직임은 모델링하지 않습니다.

The discrete constant-sample covariance has rank six for one IMU integration
step. Its singular inverse can discard a position relation. Asynchronous sensor
events can create such short intervals even with a low-rate camera or sonar.
`basalt_imu_interval_probe` reproduces that rank deficiency, verifies positive
continuous covariance and whitening, and checks the analytic covariance against
nonuniform partitions at 1, 10 and 37 ms. The native lifecycle regression checks
all owned historical poses with DVL and pressure events 10 ms apart, including
repeated marginalization.

한 번만 적분한 이산 상수 샘플 공분산은 랭크가 6입니다. 특이 공분산의 역행렬
처리에서 위치 관계가 사라질 수 있습니다. 카메라나 소나가 저주파여도 비동기
센서 이벤트가 이런 짧은 구간을 만듭니다. `basalt_imu_interval_probe`는 기존
랭크 결손을 재현하고 연속 공분산의 양의 정부호·백색화와 1·10·37 ms 구간의
불균일 분할을 해석식으로 검증합니다. 네이티브 회귀 검사는 DVL과 수압 이벤트를
10 ms 간격으로 넣고, 반복 주변화 중의 모든 과거 보유 자세를 확인합니다.

`prepare_joint_headers.py` verifies the pinned Basalt header hash and inserts
`imu_continuous.inc` into a build-local copy. The original BSD-3-Clause notice
is preserved; the original discrete integration API remains available. The
worker reports `continuous_white_midpoint_acceleration_bridge_v1`. A registered
provider built earlier keeps its own binary until rebuilt and qualified.

`prepare_joint_headers.py`는 고정한 Basalt 헤더 해시를 검사하고 빌드 사본에
`imu_continuous.inc`를 삽입합니다. 원래 BSD-3-Clause 고지와 이산 적분 API는
보존합니다. 작업자는 `continuous_white_midpoint_acceleration_bridge_v1`을
보고합니다. 이전에 등록한 공급자는 재빌드·검증하기 전까지 기존 바이너리를
사용합니다.

Continuous/discrete IMU noise conventions: [Forster et al., RSS 2015](https://www.roboticsproceedings.org/rss11/p06.html).
The interval covariance is a standard integrated white-acceleration model;
this correction is not a research novelty claim.

연속·이산 IMU 잡음의 단위 관계는 위 논문을 참고합니다. 구간 공분산은 표준적인
백색 가속도 적분 모델이며, 이 수정에 연구 신규성을 주장하지 않습니다.
