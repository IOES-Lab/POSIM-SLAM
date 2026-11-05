# Visual-map information contract / 시각 지도 정보 계약

`posim_slam.map_quality.native_settings()` requests persistent native mapping,
recovered joint information and a 0.1 m world-position three-sigma budget.
The bare worker keeps persistent mapping off; callers opt in and validate the
acknowledgement with `validate_ready()`.

`PersistentMapper` receives immutable, post-commit VIO retirement snapshots.
Shared velocity, bias and the unknown pressure datum are eliminated together
with their cross-information. Basalt NfrMapper recovers approximate nonlinear
pose factors and performs multiview bundle adjustment on retained images.
One nonlinear graph retains its optimized poses and landmarks. Original pixel
identities identify new, extended, merged and conflict-removed tracks. Only
uninitialized tracks are triangulated; existing means remain optimizer variables.
Each update still optimizes the full bounded graph. The original estimated
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
with pending input, or at end of input. `MappingInputStore` recovers each retirement's
nonlinear factors and detects each keyframe's descriptor features once. Immutable
snapshots retain feature-page references, factors and the latest pose seeds;
raw images and marginalization matrices are released after ingestion. SHA-256 of
image dimensions and visible uint16 pixels checks repeated timestamps. Limits are
203 keyframes, 96 MiB of processed-image-equivalent input, 96 MiB of compact logical
payload, 512 retirement records and 100,000 point-information candidates. Logical
payload bytes exclude allocator overhead and are reported separately from process RSS. A bound or malformed input invalidates map output while the navigation
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
비선형 자세 제약을 근사 복원하고 보존한 특징으로 번들 조정을 수행합니다.
하나의 비선형 그래프에서 최적화한 자세와 지도점을 유지합니다. 원본 픽셀 식별자로
새 트랙·연장·병합·충돌 제거를 구분하고, 지도점이 없는 트랙만 삼각측량합니다.
기존 추정값도 최적화 변수로 남으며 갱신마다 한도 내 전체 그래프를 최적화합니다.
추정한 초기 위치·방위 기준은 한 번만 사용합니다.

두 시점 트랙부터 그래프에 참여할 수 있으며 점 채택에는 서로 다른 세 영상의
정상 가중치 관측, 양의 역거리 지지와 자세 오차를 포함한 세계 좌표 불확실성
기준을 요구합니다. 설정한 픽셀 잡음과 Huber 기준을 유지합니다. 기준 카메라
구성은 0.5픽셀 잡음과 1픽셀 Huber 기준을 사용합니다. 보정값 고정, NFR 근사와
추정 단계 간 측정 상관의 미검증을 명시하며 `map_qualified`는 false입니다.

입력 수신과 지도 최적화는 별도 스레드입니다. 입력 큐는 사본 두 건을 보관하고,
새 입력 8건, 남은 입력의 5초 대기 또는 입력 종료 시 최적화합니다. 원래 시각으로
입력 영상을 구분합니다. `MappingInputStore`는 전달 정보별 비선형 제약 복원과
키프레임별 기술자 특징 검출을 한 번만 수행합니다. 불변 사본은 특징 페이지 참조·복원
제약·최신 자세 초기값을 보존하며 원본 영상과 주변화 행렬은 수신 처리 후 놓습니다.
같은 시각의 영상은 크기와 유효 uint16 픽셀의 SHA-256으로 일치를 확인합니다.
한도는 키프레임 203개, 처리 영상 환산 입력 96MiB, 특징·제약 논리 데이터 96MiB,
전달 정보 512건과 점 정보 후보 100,000개입니다. 논리 데이터 크기는 할당기 부가
메모리를 제외하며 프로세스 RSS와 별도로 보고합니다. 초과·입력 오류 시
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

## Sparse covariance computation / 희소 공분산 계산

World-point uncertainty uses Eigen sparse LDLT and selected inverse entries, with exact solves for entries outside that set. Small rank-deficient graphs use a dense spectral calculation that preserves unobserved directions. Larger unresolved rank deficiencies invalidate map output. The keyframe, image and retirement limits still apply.

세계 좌표 점의 불확실성은 Eigen 희소 LDLT와 필요한 공분산 항의 선택 역계산을 사용합니다. 그 집합 밖의 항은 선형계를 직접 풉니다. 작은 특이 그래프는 관측 불가 방향을 보존하는 밀집 고유값 계산으로 처리하며, 큰 그래프의 관측 가능성을 판별할 수 없으면 지도 출력을 무효화합니다. 키프레임·영상·전달 정보 한도는 유지합니다.

The information graph accumulates reduced pose normals without retaining a full pose covariance or every reduced visual Jacobian. Selected inverse entries follow the filled LDLT pattern; missing entries use exact solves, including cross-pose covariance. A positive-definite factorization is required for that path. Rank-deficient graphs within the live joint-column budget use the existing spectral nullspace calculation; larger unresolved ranks fail closed. No gauge prior or damping is added by the covariance backend. Calibration, NFR approximation and sensor-error correlations retain their stated scope.

감소시킨 자세 정보행렬을 누적하며 전체 자세 공분산이나 모든 시각 제약 야코비안을 보존하지 않습니다. LDLT의 채움 구조에서 역행렬 항을 계산하고 누락한 항은 교차 자세 공분산을 포함해 직접 풉니다. 이 경로는 양의 정부호 분해가 필요합니다. 작은 그래프의 관측 불가 방향은 기존 고유값 계산으로 보존하고, 큰 그래프에서 관측 가능성을 판별할 수 없으면 지도 실패로 처리합니다. 공분산 계산부가 새 기준 좌표 제약이나 감쇠항을 더하지 않습니다. 보정·NFR 근사·센서 오차 상관의 한계는 그대로 적용합니다.

[Covariance recovery method](https://dellaert.github.io/files/Kaess09ras.pdf) · [Sparse information implementation](../../native/basalt/sparse_normal_information.hpp) · [Analytic and dense controls](../../native/basalt/sparse_normal_information_probe.cc)


## Linearized submap information / 선형화한 부분 지도 정보

The optional `SubmapInformation` utility stores fixed-linearization marginal rows
and removed-variable conditionals with original measurement ownership. Late
factors recondition the affected chain; joint queries retain cross-point noise,
null directions and exact affine constraints. It reuses the component's existing
QR/SVD operations. Row compression carries the source numerical precision so
round-off directions do not become measurements in a smaller marginal. The
caller supplies one consistent tangent reference and its
original world-reference rows. It is an in-memory numerical interface; the
persistent native mapper retains its existing keyframe/input bounds. See the
[native input and bounds](../../native/basalt/README.md).

선택형 `SubmapInformation`은 고정 선형화의 주변 정보와 제거한 변수의 조건부
정보를 원본 측정 소유권과 함께 보존합니다. 늦게 온 제약은 영향을 받은 연결
정보를 다시 계산하고, 결합 조회는 점 사이 공유 잡음·관측 불가 방향·정확한
아핀 제약을 유지합니다. QR/SVD 압축 뒤에도 원본의 수치 정밀도 기준을 전달해
반올림 잔차를 관측 정보로 취급하지 않습니다. 호출자가 일관된 접선
좌표와 원래 기준 좌표 제약을 제공합니다. 메모리 수치 인터페이스이며 현재
네이티브 지도 워커의 키프레임·입력 한도는 유지합니다.

`SubmapFactorization` retains source-factor groups and their elimination
dependencies. Replacing an archived group recomputes affected branches and
shares unchanged conditionals. Explicit unassignment keeps original keys in
the ledger; the caller retains the raw input. Its frontier can move while old
coordinates remain queryable. This numerical interface does not perform native
feature reassociation, automatic mapper rollover or disk paging.

`SubmapFactorization`은 원본 요인 그룹과 소거 계산의 의존 관계를 보존합니다.
과거 그룹을 교체하면 영향을 받은 부분만 다시 계산하고 나머지 조건부 정보를
재사용합니다. 대응을 해제한 원본 키도 기록에 남기며 실제 입력은 호출자가
보존합니다. 활성 구간이 이동한 뒤에도 과거 좌표를 조회할 수 있습니다.
네이티브 특징 대응 변경·지도 워커 자동 전환·디스크 페이징은 별도로 필요합니다.

[Source-factor ownership](../../native/basalt/submap_factorization.hpp)
· [Information compression](../../native/basalt/submap_information.hpp)


## Original observation graph / 원본 관측 그래프


The mapper identifies observations by original image timestamp, camera and feature index. Rebuilding tracks replaces the complete association graph and rejects duplicate pixel ownership. Point IDs follow each track's earliest original observation rather than internal track numbering.

Published map snapshots retain immutable measurement and optimizer values. A full PLY refresh also saves `worker-map.graph.pmap` in the native output directory. Its `PSIMMAP4` archive preserves calibration, original observations and pose seeds, recovered factors, current poses/landmarks, optimizer controls, segment identity, all detector features and all evaluated image-pair decisions. Native graph restoration retains successful and failed matches, reconstructs retrieval and checks each complete original feature frame. Unmatched pixels remain available for later associations. The next native track ID and export identity ledger continue from the saved graph.

The 96 MiB file limit includes metadata and all pages. Older graph-only archives retain their saved associations for relinearization. Raw images and live VIO/IMU history are separate; automatic graph growth, window rollover and navigation restart remain unavailable. Viewer-only requests do not write the archive. A write failure fails the export request while sensor processing continues.


관측은 원본 영상 시각·카메라·특징 번호로 구분합니다. 트랙을 재구성하면 전체 관측 연결을 교체하고 픽셀의 중복 배정을 거부합니다. 점 ID는 내부 트랙 번호 대신 각 트랙의 가장 이른 원본 관측을 기준으로 유지합니다.

발행된 지도 사본은 측정값과 최적화 값을 불변 데이터로 보관합니다. 전체 PLY 갱신은 네이티브 출력 디렉터리에 `worker-map.graph.pmap`도 저장합니다. `PSIMMAP4` 파일은 보정값·원본 관측과 입력 자세 초기값·복원 제약·현재 자세와 지도점·최적화 설정·구간 식별자·모든 검출 특징과 영상쌍 평가 결과를 보존합니다. 네이티브 그래프를 복원하면 성공·실패한 매칭 결과를 유지하고 검색 색인을 재구성하며 전체 원본 특징을 검사합니다. 아직 트랙에 연결되지 않은 픽셀도 이후 대응에 사용할 수 있습니다. 다음 네이티브 트랙 번호와 내보내기 ID 이력도 이어집니다.

파일의 96MiB 한도에는 메타데이터와 모든 페이지가 포함됩니다. 이전 그래프 전용 파일은 저장된 관측 연결의 재계산에 사용합니다. 원본 영상과 라이브 VIO/IMU 이력은 별도이며 자동 그래프 확장·구간 전환·항법 재시작은 아직 지원하지 않습니다. 화면 조회는 파일을 기록하지 않습니다. 기록 오류는 내보내기 요청을 실패 처리하며 센서 처리는 계속합니다.


The native stream stores complete detector features, recovered factors, pose seeds and retirement identities in a session-owned `worker-map-inputs-*.sqlite` file beside its output. Each retirement commits in one SQLite transaction before publication. Immutable map snapshots retain page references; a read leases one checked feature frame, while optimization retains pixel observations and the bounded pose/factor graph. The SQLite page cache targets 2 MiB, memory mapping is disabled and the database is bounded to 96 MiB. Input limits and admission checks still apply. A storage error invalidates map output; the navigation estimator can continue. Restoring inputs requires matching epoch, calibration, mapper settings and limits; it does not restart navigation.

네이티브 스트림은 모든 검출 특징·복원 제약·자세 초기값·퇴출 식별자를 출력 옆의 세션별 `worker-map-inputs-*.sqlite`에 저장합니다. 전달 정보 한 건을 SQLite 트랜잭션으로 확정한 뒤 지도 생성 쪽에 전달합니다. 불변 지도 사본은 페이지 참조를 보관하고, 읽을 때 검증한 특징 프레임 하나를 임시로 소유합니다. 최적화는 픽셀 관측과 한도 내 자세·제약 그래프를 메모리에 보관합니다. SQLite 페이지 캐시 목표는 2MiB이며 메모리 매핑은 끄고 데이터베이스 크기는 96MiB로 제한합니다. 기존 입력 한도와 점 채택 기준은 유지합니다. 저장 오류는 지도 출력을 무효화하며 항법 추정은 계속할 수 있습니다. 입력 복원에는 같은 구간 식별자·보정값·지도 설정·한도가 필요하며 항법을 재시작하지 않습니다.


### Native matching input memory / 네이티브 매칭 입력 메모리

The native matcher uses the pinned Basalt HashBoW retrieval, descriptor matching
and OpenGV RANSAC. Two tasks lease at most four complete feature frames; the
mapper retains their pixel coordinates for track construction and optimization.
Descriptors, bearings and orientation arrays are released after use. The segment
keeps its retrieval index and previously evaluated image-pair decisions, including
failed attempts. Chronological input queries only new frames; delayed input
refreshes affected later retrieval while reusing old pair results. The all-frame
path retains the upstream Hamming threshold 70
and second-best ratio 1.2. The similarly named configurable fields belong to the
separate stereo path. Original pixel ownership, NFR factors and map admission
remain unchanged. Checkpoints restore matching decisions without repeating old
pair evaluations. Conditional-window growth,
active solver paging and rollover still require separate qualification.

네이티브 매칭은 고정 Basalt의 HashBoW 검색·기술자 대응과 OpenGV RANSAC을
사용합니다. 두 작업자가 전체 특징 프레임을 최대 4개까지 임시로 읽고, 지도는
트랙 구성과 최적화에 필요한 픽셀 좌표를 보관합니다. 기술자·방향 벡터·특징점
방향값은 사용 후 해제하고, 현재 구간의 검색 색인과 영상쌍 평가 결과는 실패한
결과까지 보존합니다. 시간순 입력은 새 영상만 검색하며 지연 입력은 이후 검색을
갱신하면서 기존 영상쌍 결과를 재사용합니다. 전체 프레임 매칭의 Hamming
한도 70과 두 번째 후보 거리 비율 1.2는 원본을 따릅니다. 같은 이름의 설정값은
별도의 스테레오 경로에 적용됩니다. 원본 픽셀 소유권·NFR 제약·점 채택 기준은
유지합니다. 체크포인트에서 매칭 판단을 복원하면 기존 영상쌍을 다시 평가하지
않습니다. 조건부 정보 기반 범위 확장·활성
최적화 범위 제한·구간 전환은 별도 검증이 필요합니다.
