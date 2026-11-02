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

The two-layer design follows [Usenko et al., section V-A](https://www.usenko.net/pdf/usenko2020nfr.pdf):
mapping detects descriptor features separately from VIO patch tracking.
Independent observation errors are a model assumption; different feature IDs
or detectors alone do not measure cross-layer error covariance.

두 단계 구조는 [Usenko 외, V-A절](https://www.usenko.net/pdf/usenko2020nfr.pdf)을
따릅니다. 지도는 VIO 패치 추적과 별도로 기술자 특징을 검출합니다. 관측 오차의
독립성은 모델 가정이며, 다른 특징 ID나 검출기만으로 단계 간 오차 공분산을
측정한 것은 아닙니다.

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

`LiveClient.snapshot()` pins the mapper's currently accepted retirement revision,
wakes pending optimization and exports an immutable result covering that target.
A separate bounded writer keeps camera/IMU input and state processing active.
The native computation deadline is 10 seconds, the ROS owner deadline is 11
seconds and the browser adapter service deadline is 12 seconds. These job bounds are distinct from
existing state, sensor, segment and completed-snapshot freshness checks. Only one
native export is active; another request reports busy. Session changes cancel
owner admission and cannot relabel an old map.

`LiveClient.snapshot(id, refresh=False)` reads the immutable published snapshot
without waking optimization. A separate capacity-one writer produces
`worker-view.ply`; it can complete while an explicit refresh waits. It does not
replace the export PLY or candidate diagnostics. The ROS `/snapshot` service
uses a separate pending request and writes `view.ply`; its owner/adapter/client
bounds are 2/2.5/3 seconds. The 2,000/5,000/all-points viewer uses this service;
the 1,000-point ROS sample is unchanged. PLY and replies identify
`snapshot_policy current` or `snapshot_policy update`.

PLY and the export response carry `requested_mapping_revision`,
`optimization_revision` and `optimized_through_ns`. The result revision must
cover the fixed request target; its input timestamp cannot exceed tracking time.
The adapter reports server `processing_seconds`; the client deducts transport
latency from the completed snapshot's lifetime instead of counting computation
as stale transport. The three-second freshness budget remains bounded.

전체 지도 요청은 수신 완료된 입력 회차를 고정하고 최적화를 깨웁니다. 별도 출력
스레드가 그 기준을 반영한 결과를 기다리는 동안 센서 수신과 항법은 계속합니다.
다운로드 계산 대기는 네이티브 10초·ROS 소유자 11초·브라우저 서비스 12초입니다.
상태·센서·구간·완료된 결과의 신선도 검사는 별도입니다.
한 번에 하나의 내보내기를 처리하며 구간이 바뀐 이전 결과는 수용하지 않습니다.
요청 회차·완료 회차·지도 입력 시각을 검증합니다. 서버 계산 시간을 구분해 응답
전송 지연만 결과의 유효 시간에서 차감하며 3초 신선도 한도를 유지합니다.

`refresh=False` 조회는 발행된 불변 지도 사본을 읽으며 최적화를 깨우지 않습니다.
별도 출력 스레드와 요청 슬롯을 사용해 다운로드 대기 중에도 조회를 처리합니다.
조회 파일은 전체 다운로드 PLY와 후보 진단을 덮어쓰지 않습니다. ROS `/snapshot`
조회는 소유자·어댑터·HTTP에서 각각 2초·2.5초·3초로 제한합니다.
2,000점·5,000점·전체 뷰는 이 경로를 사용하고 1,000점 ROS 표본은 유지합니다.
PLY와 응답은 `snapshot_policy current` 또는 `snapshot_policy update`를 구분합니다.

The export contains all eligible points independently of the display sample. PLY scope `reoptimized_sparse_mapping_landmarks` requires
`map_qualified false`, a positive `optimization_revision` and a positive
`optimized_through_ns` no later than `stamp_ns`. A diagnostic
`record_mapping_candidates: true` additionally writes every candidate row,
including nonfinite coordinates encoded as JSON null. Neither that option nor
viewer density changes estimator observations or export admission. Legacy
`persistent_mapping: false` archive scopes remain readable.
Application consumers use `map_snapshot.VIO_MAP_SCOPE` for this current visual
scope; legacy file readers retain the file's declared scope.

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
