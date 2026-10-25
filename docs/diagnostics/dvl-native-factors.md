# Native raw DVL factors / 네이티브 원본 DVL 제약

[Machine-readable evidence](dvl-native-factors.json) · [Interface and assumptions](../../native/basalt/README.md)

These are analytic sensor and ownership tests of the independent native
optimizer. They do not measure real-world sonar mapping accuracy. No new
physical point-cloud comparison or qualified application mode is established.

The native factor probe checks all 15 state-increment columns by numerical
differentiation, a rotated mount and nonzero lever arm, full correlated
covariance whitening, frozen Jacobian references, world-frame gauge invariance,
Huber cost, malformed packets, stale retirement, and rollback on failed prior
installation. Its Jacobian error norm is 6.94e-10. The marginal-prior information
matches the independently assembled stacked system; retained DVL observations
are excluded from the consumed rows. A velocity optimization decreases cost.

A 20-second moving-start fixture feeds only raw IMU and DVL velocity: 101
states, 81 DVL observations, and four seconds of deliberate bottom-lock loss.
Propagation and recovery retain a bounded state window. A separate 12-second
fixture jointly feeds IMU, range/bearing tracks and DVL: 61 states, 55 with active
acoustic constraints, and at most 15 retained states. Its maximum position
error is 5.40e-5 m for these ideal analytic observations; this is not a sensor
accuracy estimate. No pixels, truth poses or point seeds enter that worker.

Camera-clock fixtures use 32×32 and 320×240 blank images with three original-time
DVL states between images, for 65 states each. They verify the optional clock
contract without visual landmarks. The default detection cell is bounded by
image dimensions, including small accepted images. Invalid input decoder
exceptions are reported before joinable reader threads are unwound.

Existing camera-free sonar, stream/input contracts, camera VIO synthetic paths,
Ping marginalization, sonar-factor and joint-landmark probes also pass. Native
source hashes, binary hash and measured regression metrics accompany this file.
No new third-party dependency is introduced.

Actual DVL rates, noise, bottom-lock failures and moving-start observability
remain to be measured. High-rate states can shorten acoustic triangulation
history. The gyro/preintegration noise correlation is not modelled. Pressure
and terrain factors, calibrated 2 m optical visibility, physical point-cloud
quality and application integration remain outstanding.

이 기록은 독립 네이티브 최적화기의 해석적 센서·상태 관리 시험입니다. 실제 소나
지도의 정확도를 측정한 결과가 아니며 새 물리 점군 비교나 앱 모드 합격을 뜻하지
않습니다.

네이티브 제약 시험은 15개 상태 증분 열의 수치 미분, 회전된 장착 방향과 장착
위치, 전체 공분산 백색화, 고정 Jacobian 기준점, 월드 좌표계 불변성, Huber 손실,
잘못된 패킷, 오래된 주변화 요청과 설치 실패 시 복원을 검사합니다. Jacobian
오차 노름은 6.94e-10입니다. 주변화 사전정보는 별도로 쌓아 계산한 시스템과
일치하며 유지할 DVL 관측을 중복 반영하지 않습니다. 속도 최적화의 비용도 감소합니다.

20초 이동 시작 시험은 원본 IMU와 DVL만 입력합니다. 상태 101개와 DVL 관측
81개를 처리하고 4초간 바닥 잠금을 끊었다가 복구하며 상태 창을 제한 안에
유지합니다. 별도의 12초 공동 시험은 IMU·거리/방위각 트랙·DVL을 입력합니다.
상태 61개 중 55개에 음향 제약이 있고 최대 15개 상태를 유지합니다. 이상적인
해석 관측의 최대 위치 오차는 5.40e-5 m이며 센서 정확도 수치가 아닙니다.
영상·정답 자세·점 초기값은 이 워커에 들어가지 않습니다.

카메라 시계 시험은 32×32와 320×240의 빈 영상에 영상 사이 원래 시각의 DVL
상태 3개를 넣어 각각 65개 상태를 처리합니다. 시각 특징 없이 시계 협약을
검증합니다. 기본 특징 검출 셀은 작은 입력에서도 영상 크기 안에 들어갑니다.
잘못된 입력의 예외는 실행 중인 읽기 스레드가 해제되기 전에 오류 사유를 보고합니다.

기존 카메라 없는 소나, 스트림·입력 협약, 카메라 VIO 합성 경로, Ping 주변화,
소나 제약과 공동 특징점 검증도 통과했습니다. 소스·바이너리 해시와 회귀 수치는
연결된 JSON에 보존합니다. 새 제3자 의존성은 없습니다.

실제 DVL 빈도·잡음·바닥 잠금 손실과 이동 시작의 관측 가능성은 추가 검증이
필요합니다. 높은 상태 빈도는 음향 삼각측량의 이력 길이를 줄일 수 있습니다.
자이로와 IMU 사전적분 잡음의 상관은 모델링하지 않습니다. 수압·지형 제약,
보정된 RGB 시야 2 m 시험, 물리 점군 품질과 앱 연결은 남아 있습니다.
