# Current native state information

Set `record_current_state_information` to `true` in a camera/IMU worker's JSON
calibration to request a current-state uncertainty report. Optional Ping, DVL
and pressure factors participate in the same information assembly. The worker
acknowledges `undamped_joint_selected_state_v1` in
`current_state_information_transport`. The default is disabled. Camera-free
sonar and surface-owner profiles refuse this request.

At most once per simulated second, a state reply's `initialization_window`
contains `current_state_information`. Its timestamp identifies the current
state; the other window fields describe the oldest retained state. The query
runs on the estimator owner thread before marginalization. It reuses the map
information linearization without building landmark covariance reports or
changing states, priors, damping, noise or admission.

The 15-dimensional order is world position (m), world-left rotation (rad),
world velocity (m/s), IMU gyroscope bias (rad/s) and IMU accelerometer bias
(m/s²). The current physical tangent includes the Jacobian of retained rotation
increments. `observable_covariance` retains the full 15×15 finite covariance,
including cross terms. `unbounded_directions` retains projected free modes.
Read both together: an affected block has `observed: false` and a null
`three_sigma_max`, rather than zero uncertainty. Blocks follow the same order.

This is local conditional information under the declared sensor noise,
existing initial position/yaw and bias priors, retained linearization and frozen
robust weights. The report includes those initial prior weights. It keeps
`statistically_calibrated`, `calibration_uncertainty_included` and
`navigation_ready` false. Bootstrap qualification and application readiness
remain separate. An unavailable query carries a reason while tracking continues.

## 현재 네이티브 상태 정보

카메라·IMU 워커의 JSON 보정에 `record_current_state_information: true`를
설정하면 현재 상태 불확실성을 요청합니다. 선택한 Ping·DVL·수압도 같은
정보 계산에 포함됩니다. 워커는 `current_state_information_transport`에서
`undamped_joint_selected_state_v1`을 확인합니다. 기본값은 꺼짐이며 카메라
없는 소나·surface-owner 구성은 이 요청을 거부합니다.

시뮬레이션 시각으로 최대 1초에 한 번 상태 응답의 `initialization_window`에
`current_state_information`이 들어갑니다. 이 정보의 시각은 현재 상태이며
다른 윈도 필드는 가장 오래 보존한 상태를 설명합니다. 주변화 전에 추정기
소유 스레드에서 기존 지도 정보 계산을 재사용합니다. 랜드마크 공분산을
추가 계산하거나 상태·사전분포·감쇠·잡음·채택 기준을 바꾸지 않습니다.

15차원 순서는 세계 위치(m), 세계 기준 왼쪽 회전 증분(rad), 세계 속도(m/s),
IMU 자이로 바이어스(rad/s), IMU 가속도계 바이어스(m/s²)입니다. 현재 물리
접공간으로 바꿀 때 보존된 회전 증분의 야코비안도 반영합니다.
`observable_covariance`는 교차항을 포함한 15×15 유한 공분산이며
`unbounded_directions`는 투영된 관측불능 방향입니다. 두 값을 함께
읽습니다. 관측불능 방향에 영향을 받는 블록은 `observed: false`,
`three_sigma_max: null`이며 불확실성을 0으로 표시하지 않습니다.
블록도 같은 순서입니다.

이 정보는 선언한 센서 잡음, 기존 초기 위치·방위각·바이어스 사전분포,
보존된 선형화와 고정된 강건 가중치를 조건으로 합니다. 초기 사전분포의
가중치도 보고합니다. `statistically_calibrated`,
`calibration_uncertainty_included`, `navigation_ready`는 false를 유지합니다.
초기화 검증과 앱 준비 판정은 별도입니다. 정보를 계산할 수 없으면 이유를
보고하며 추적은 계속합니다.
