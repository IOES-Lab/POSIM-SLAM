# Visual-map information contract / 시각 지도 정보 계약

`posim_slam.map_quality.native_settings()` requests persistent native mapping,
recovered joint information and a 0.1 m world-position three-sigma budget.
The bare worker keeps persistent mapping off; callers opt in and validate the
acknowledgement with `validate_ready()`.

`PersistentMapper` receives immutable, post-commit VIO retirement snapshots.
Shared velocity, bias and the unknown pressure datum are eliminated together
with their cross-information. Basalt NfrMapper recovers approximate nonlinear
pose factors and performs multiview bundle adjustment on retained images.
Each graph is rebuilt from unique input identities. The original estimated
initial position/yaw reference is reused once; truth and damping do not add
world information. See the [pinned Basalt source](../../native/basalt/sources.json)
and [implementation](../../native/basalt/persistent_mapper.cc).

Two-view tracks can constrain the graph. Export eligibility requires at least
three distinct full-weight exposures, positive inverse-range support and the
requested world-position uncertainty budget, including shared pose uncertainty.
The configured VIO pixel noise and Huber threshold are used unchanged. The
reference camera profile uses 0.5 px noise and 1 px Huber threshold. Calibration
is fixed; NFR is approximate and cross-layer statistical independence remains
unverified. `map_qualified` stays false.

Input ingestion and optimization use separate threads. The input queue holds two
snapshots. Optimization runs after eight new snapshots, after a five-second wait
with pending input, or at end of input. Keyframe images are deduplicated by
original timestamp and checked for identical pixels. Limits are 203 keyframes,
96 MiB of retained images, 512 retirement snapshots and 100,000 point-information
candidates. A bound or malformed input invalidates map output while the navigation
estimator can continue. Long-run rollover beyond these bounds remains unsupported.

`validate_geometry(report, window, total_points=..., stamp_ns=...)` validates
provisional status, policy, counts and optimization timing. The current VIO
landmark window is separate from the persistent candidate count. The tracking
publication timestamp remains synchronized with the current pose;
`optimized_through_ns` records the latest keyframe input used by the background
map. A map cannot claim inputs newer than its paired tracking publication.

`LiveClient.snapshot()` exports all eligible points independently of the display
sample. PLY scope `reoptimized_sparse_mapping_landmarks` requires
`map_qualified false`, a positive `optimization_revision` and a positive
`optimized_through_ns` no later than `stamp_ns`. A diagnostic
`record_mapping_candidates: true` additionally writes every candidate row,
including nonfinite coordinates encoded as JSON null. Neither that option nor
viewer density changes estimator observations or export admission. Legacy
`persistent_mapping: false` archive scopes remain readable.

`native_settings()`는 지속 지도 생성, 복원한 결합 정보와 세계 좌표 위치의
3시그마 한도 0.1m를 요청합니다. 워커 자체의 기본값은 지속 지도 비활성이며
호출자가 선택하고 응답을 검증합니다.

`PersistentMapper`는 VIO 주변화 확정 뒤 전달한 불변 사본을 받습니다. 공유
속도·바이어스·미지 압력 기준값은 교차정보와 함께 소거합니다. Basalt NfrMapper가
비선형 자세 제약을 근사 복원하고 보존한 영상으로 번들 조정을 수행합니다.
중복 없는 입력으로 매번 그래프를 재구성하고, 추정한 초기 위치·방위 기준을
한 번만 사용합니다.

두 시점 트랙부터 그래프에 참여할 수 있으며 점 채택에는 서로 다른 세 영상의
정상 가중치 관측, 양의 역거리 지지와 자세 오차를 포함한 세계 좌표 불확실성
기준을 요구합니다. 설정한 픽셀 잡음과 Huber 기준을 유지합니다. 기준 카메라
구성은 0.5픽셀 잡음과 1픽셀 Huber 기준을 사용합니다. 보정값 고정, NFR 근사와
추정 단계 간 측정 상관의 미검증을 명시하며 `map_qualified`는 false입니다.

입력 수신과 지도 최적화는 별도 스레드입니다. 입력 큐는 사본 두 건을 보관하고,
새 입력 8건, 남은 입력의 5초 대기 또는 입력 종료 시 최적화합니다. 원래 시각으로
영상을 중복 제거하고 같은 시각의 픽셀 일치를 확인합니다. 한도는 키프레임 203개,
영상 96MiB, 전달 사본 512건과 점 정보 후보 100,000개입니다. 초과·입력 오류 시
지도는 무효화하고 항법 추정은 계속할 수 있습니다. 한도 이후의 장기 지도 전환은
지원하지 않습니다.

현재 VIO 창의 특징 수와 지속 지도 후보 수를 구분합니다. 발행 시각은 현재 자세와
동기화하고, `optimized_through_ns`는 지도에 반영한 최신 키프레임 입력 시각입니다.
전체 PLY는 표시 제한과 독립적이며 최적화 회차·입력 시각과 잠정 상태를 담습니다.
`record_mapping_candidates: true`는 비유한 좌표를 JSON null로 보존한 모든 후보를
추가 기록합니다. 기존 저장 방식의 PLY도 원래 범위를 유지해 읽을 수 있습니다.

- [Information calculation](../../native/basalt/mapping_information.hpp)
- [Geometry contract](../../src/posim_slam/map_quality.py)
- [Snapshot contract](../../src/posim_slam/map_snapshot.py)
- [Threaded ownership and failure controls](../../native/basalt/persistent_mapper_probe.cc)
- [Contract tests](../../tests/test_map_quality.py)
- [Export tests](../../tests/test_map_snapshot.py)
