# Visual/IMU factor ownership / 시각·IMU 관측 관리

Camera/IMU estimation and combinations with optional sensors use the same
optimizer and marginalization ownership after visual landmarks are created.
The complete state order includes a feature's host even when that host remains
a full IMU state. Retiring a lost feature preserves its information in the
correlated prior and consumes each observation once. Optional Ping, sonar,
DVL and pressure stores can be empty; an empty store supplies no measurement.

Native `max_recent_states` accepts 3–64 and is acknowledged in the ready event.
The ordinary camera default is three states. Increasing this number consumes
more computation and memory and does not ensure an informative map.
`native/basalt/test_visual_window_native.py` checks young lost-feature hosts,
121 translated-texture images and bounded retirement in a 16-state window
with no auxiliary sensors. It is an ownership regression, not a map-accuracy
or inertial-observability certificate.

The [recorded controls](../validation/common-visual-window-20261028/README.md)
include complete maps, all-point assessments and matching RGB figures. With
the same retained startup proposal, three states export 24 points and sixteen
export 146; both miss required scene geometry. The camera/IMU/Ping ROS control
passes bounded IMU-loss recovery. Retained-startup controls are two-pass
experiments, not a live initialization API. Provider and field acceptance
remain separate.

## 한국어

카메라·IMU 추정과 선택 센서 결합은 시각 특징점 생성 뒤 같은 최적화·관측
정리 경로를 사용합니다. 특징점을 소유한 프레임이 IMU 상태로 남아 있어도
전체 계산 순서에 포함합니다. 유실 특징점을 정리할 때 관측 정보는 상관관계를
포함한 사전정보로 남으며 각 관측을 한 번씩 반영합니다. Ping·소나·DVL·수압
저장소가 비어 있으면 해당 센서의 측정값을 공급하지 않습니다.

네이티브 `max_recent_states`는 3~64를 허용하고 ready 이벤트로 실제 값을
확인합니다. 일반 카메라 기본값은 3입니다. 큰 값은 계산량·메모리를 늘리며
충분한 지도 정보를 보장하지 않습니다. 위 회귀시험은 선택 센서 없이 영상
121장과 이력 16개의 특징점 소유·정리를 검증합니다. 지도 정확도나 IMU
관측가능성을 인증하는 시험은 아닙니다.

위 기록에는 전체 점군·모든 점의 오차·같은 시점 방향의 RGB가 있습니다.
같은 초기값의 대조 시험에서 이력 3개는 24점, 16개는 146점이지만 필요한
장면이 빠집니다. 카메라·IMU·Ping ROS 시험은 제한된 IMU 유실·복구를
통과합니다. 초기 관측 유지 시험은 기록을 두 번 처리하는 수치 실험이며,
실제 운영 초기화 API와 배포·현장 검증은 별도입니다.
