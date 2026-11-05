# Visual-map information contract / 시각 지도 정보 계약

`posim_slam.map_quality.native_settings()` enables persistent mapping and a
**0.1 m** world-position three-sigma budget. Its default `full_graph_v1` backend
uses the full graph, limited to **203 keyframes**. To select the optional
conditional policy, pass `dict(native_settings(), map_backend="conditional_paged_v1")`.
`validate_ready(ready, backend="conditional_paged_v1")` checks the requested policy,
capacities, epoch and snapshot protocol. Without an explicit backend requirement,
it also accepts the legacy full-graph provider contract. The bare native binary
leaves persistent mapping disabled; the bare C++ constructor uses `active_window=0`.

`PersistentMapper` receives immutable VIO retirement snapshots after the marginal
prior commits. Shared velocity, bias and an unknown pressure datum are eliminated
with their cross-information. Basalt NfrMapper recovers approximate nonlinear
pose factors. Mapping detects descriptor features separately from VIO patch
tracking, following [Usenko et al., V-A](https://www.usenko.net/pdf/usenko2020nfr.pdf).
Different feature IDs alone do not establish independent measurement errors.

The optional conditional policy reoptimizes the recent calculation window. Historical factors
retain their original observations, linearization anchors and conditional
correlations; historical poses and landmarks move with the solution. Changed
source groups rebuild their affected elimination dependencies. Weighted minimum
degree orders the affected historical blocks while unchanged branches keep their
numeric pages. An order is adopted when its symbolic work decreases without
widening the largest intermediate block. QR/SVD elimination
and conditional queries produce world-point uncertainty. The original estimated
position/yaw reference enters once. Truth and optimizer damping add no information.

Export eligibility requires **three distinct full-weight exposures**, positive
inverse range and maximum world-coordinate **3-sigma ≤ 0.1 m**. Two-view tracks
can constrain the graph before admission. The reference profile uses **0.5 px**
noise and a **1 px** Huber threshold. Calibration is fixed, NFR is approximate
and cross-layer statistical independence is unverified. `map_qualified` is false.

`native_settings()`는 지속 지도와 세계 좌표 **3시그마 0.1m** 한도를 켭니다.
기본 `full_graph_v1` 백엔드는 전체 그래프를 사용하며 **203키프레임**으로 제한합니다.
조건부 정책은 `dict(native_settings(), map_backend="conditional_paged_v1")`로
명시합니다. `validate_ready(ready, backend="conditional_paged_v1")`는 요청한
정책·용량·구간 식별자·스냅샷 규약을 검사합니다. 백엔드를 지정하지 않으면 기존
전체 그래프 공급 모듈의 규약도 허용합니다. 설정 없는 네이티브 실행 파일은 지속
지도를 끄며 기본 C++ 생성자는 `active_window=0`을 사용합니다.

`PersistentMapper`는 주변화 사전정보가 확정된 뒤 불변 VIO 퇴역 정보를 받습니다.
속도·편향·미지 압력 기준 수심은 교차정보와 함께 소거하고 Basalt NfrMapper가
근사 비선형 자세 인자를 복원합니다. [Usenko 외 V-A절](https://www.usenko.net/pdf/usenko2020nfr.pdf)에
따라 VIO 패치 추적과 별도로 기술자 특징을 검출합니다. 특징 ID가 다르다는
사실만으로 측정 오차의 독립성을 인정하지 않습니다.

선택형 조건부 정책은 최근 계산 구간을 다시 최적화합니다. 과거 인자는 원본 관측·
선형화 기준·조건부 상관관계를 보존하며 과거 자세와 지도점도 해에 따라
갱신됩니다. 원본 관측 묶음이 바뀌면 영향을 받는 소거 관계를 다시 구성합니다.
연결된 좌표 수를 가중치로 삼는 최소 차수 순서로 해당 과거 블록을 소거하며,
변경되지 않은 계산 가지는 기존 행렬을 재사용합니다. 예상 계산량이 줄고 가장
큰 중간 블록이 넓어지지 않는 순서를 적용합니다.
QR/SVD 소거와 조건부 질의로 세계 좌표 점의 불확실성을 구합니다. 원래 추정한
위치·방위 기준은 한 번 반영하며 시뮬레이터 정답이나 최적화 감쇠로 정보를
추가하지 않습니다.

점 채택에는 **완전한 가중치의 서로 다른 노출 3개**, 양의 역거리, 세계 좌표
최대 **3시그마 ≤ 0.1m**가 필요합니다. 두 시점 트랙도 채택 전에 그래프를
구속할 수 있습니다. 기준 설정은 잡음 **0.5픽셀**, Huber 임계값 **1픽셀**입니다.
보정값은 고정이며 NFR은 근사입니다. 단계 간 오차 독립성은 검증되지 않았으므로
`map_qualified`는 false입니다.

## Resource and storage policy / 자원과 저장 정책

The conditional policy has the following limits. Shared input and export bounds
also apply to the default full-graph backend.

조건부 정책에는 다음 한도를 적용합니다. 공통 입력·내보내기 한도는 기본 전체
그래프 백엔드에도 적용합니다.

- **12 recent active keyframes**, **8 MiB** derived-row cache and at most
  **10,922 keyframe addresses** in the history namespace.
- **512 retirement records**, **100,000 candidate points**, **96 MiB** of
  processed-image-equivalent input and **96 MiB** of compact input.
- Session input SQLite and complete `PSIMMAP4` archives each have a **96 MiB**
  file bound. Input SQLite targets a **2 MiB** page cache with memory mapping off.
- Input queue capacity is **two**. Matching uses **two tasks** leasing at most
  **four feature frames**. Mapping refreshes after **eight retirements**, after
  **five seconds** with pending input, or at end of input.

These are separate payload/cache bounds, not a process-RSS guarantee. Container
overhead, concurrent immutable snapshots and temporary solver workspace are
additional. Input quota exhaustion has no automatic continuation policy.

Each retirement commits once to `worker-map-inputs-*.sqlite` beside the output.
The store extracts each new keyframe once and releases raw images and large
marginal matrices. SHA-256 of dimensions and visible uint16 pixels checks image
identity. The conditional policy's derived numeric rows use a disposable sibling `.rows` directory;
existing directories are protected and a successful shutdown releases the cache.
Original input snapshots remain exportable after the derived cache is released.
A storage, capacity or rank failure invalidates map output while navigation can
continue. It cannot certify a truncated or unobserved map.

- **최근 활성 키프레임 12개**, 파생 행렬 캐시 **8MiB**, 이력 주소 상한
  **키프레임 10,922개**를 사용합니다.
- **퇴역 기록 512개·후보점 100,000개·처리 영상 환산 입력 96MiB·압축 입력
  96MiB**를 각각 제한합니다.
- 세션 입력 SQLite와 전체 `PSIMMAP4` 파일도 각각 **96MiB**로 제한합니다.
  입력 SQLite의 페이지 캐시 목표는 **2MiB**이며 메모리 매핑은 사용하지 않습니다.
- 입력 큐는 **2개**이며 대응 작업자 **2개**가 특징 프레임을 최대 **4개** 읽습니다.
  지도는 **퇴역 8회**, 대기 입력이 있는 상태에서 **5초**, 또는 입력 종료 시 갱신합니다.

각 값은 페이로드·캐시 한도이며 전체 프로세스 RSS를 보장하지 않습니다. 컨테이너,
동시 불변 사본, 풀이 임시 공간은 별도입니다. 입력 보관 한도를 넘긴 뒤의 자동
연속 처리 정책은 없습니다.

퇴역 정보는 출력 옆의 `worker-map-inputs-*.sqlite`에 한 번 확정합니다. 새 특징을
한 번 검출한 뒤 원본 영상과 큰 주변화 행렬을 해제합니다. 크기와 유효 uint16
픽셀의 SHA-256으로 같은 영상인지 확인합니다. 조건부 정책의 파생 행렬은 옆의 임시 `.rows`
디렉터리를 사용하며 기존 디렉터리는 보호하고 정상 종료 시 캐시를 해제합니다.
캐시 해제 뒤에도 원본 입력 사본을 내보낼 수 있습니다. 저장·용량·계수 부족은
지도 출력을 무효화하며 항법은 계속할 수 있습니다. 잘리거나 관측되지 않은
지도를 통과한 결과로 표시하지 않습니다.

## Snapshot and download / 스냅샷과 다운로드

`LiveClient.snapshot(id)` pins the accepted input snapshot and requests a complete
refresh while sensor processing continues. Only one export is active. Native,
ROS-owner and browser-service deadlines are **10/11/12 seconds**. Segment changes
cancel admission. `snapshot(id, refresh=False)` reads the published immutable map
without waking optimization, through a separate capacity-one writer. It produces
`worker-view.ply`; the ROS view service uses **2/2.5/3-second** owner/adapter/client
bounds. A viewer request does not replace the full export or candidate diagnostics.
The current update finishes before the pinned request runs; later input remains
available for subsequent background updates without pausing ingestion.

PLY and replies carry the requested/optimized revision, `optimized_through_ns`
and `snapshot_policy`. The optimized revision covers the request, and its input
time cannot exceed the paired tracking time. The **three-second** completed-map
freshness budget remains separate from computation time. `validate_geometry()`
checks policy, provisional status, counts and time consistency. The VIO landmark
window and persistent candidate count are separate values.

The normal ROS cloud contains at most **1,000** points; larger viewer selections
read the separate view snapshot. Full PLY contains every eligible retained point,
independent of the viewer limit, with scope `reoptimized_sparse_mapping_landmarks`.
Optional candidate diagnostics retain all candidates, encoding nonfinite values
as JSON null. Dense surface reconstruction and qualified loop closure are absent.

`snapshot(id)`는 수신 완료된 입력 스냅샷을 고정하고 전체 지도를 갱신하며 센서
처리는 계속합니다. 내보내기는 한 번에 한 건이며 네이티브·ROS 소유자·브라우저
서비스의 한도는 **10/11/12초**입니다. 구간 변경 시 채택을 취소합니다.
`snapshot(id, refresh=False)`는 최적화를 깨우지 않고 발행된 지도를 읽습니다.
별도 용량 1의 출력 작업자가 `worker-view.ply`를 만들며 ROS 조회의 소유자·
어댑터·클라이언트 한도는 **2/2.5/3초**입니다. 조회는 전체 내보내기나 후보 기록을
교체하지 않습니다.
진행 중 갱신이 끝나면 고정한 요청을 먼저 계산합니다. 그 뒤에 도착한 입력도
계속 수신·보관하며 후속 배경 갱신에서 계산합니다.

PLY와 응답은 요청·최적화 회차, `optimized_through_ns`, `snapshot_policy`를
담습니다. 결과는 요청 회차를 반영해야 하며 입력 시각은 함께 발행한 추적 시각을
넘을 수 없습니다. 완료된 지도의 **3초** 신선도 한도는 계산 시간과 별도입니다.
`validate_geometry()`는 정책·잠정 상태·점 수·시각 일치를 검사합니다. VIO 현재
구간의 랜드마크 수와 지속 지도의 후보점 수는 구분합니다.

일반 ROS 점군은 최대 **1,000점**이며 더 큰 뷰어 선택은 별도 지도 조회를 사용합니다.
전체 PLY는 표시 한도와 관계없이 채택된 보존 점을 모두 담고 범위는
`reoptimized_sparse_mapping_landmarks`입니다. 선택형 후보 기록은 비유한 값을
JSON null로 표시하며 모든 후보를 보존합니다. 조밀한 표면 복원과 검증된 루프
폐쇄는 제공하지 않습니다.

## Original observation graph / 원본 관측 그래프

`PSIMMAP4` retains all detector features, original pixels/factors, pose seeds,
current poses/landmarks, source anchors, optimizer controls, epoch/calibration,
evaluated image pairs and the next track/export IDs. Restoring a native graph
checks these identities, reconstructs retrieval and preserves successful and
failed matches. New input can extend its selected calculation window within the
retained-input quota. Navigation state is not restored. Older graph-only archives
retain their saved observation associations for relinearization.

The archive streams each checked feature frame once and uses the same encoded
bytes for matching identity. A sibling temporary file and a **1 MiB** transfer
buffer preserve the indexed format; the completed file replaces its destination
atomically. Source-integrity failures leave the published archive unchanged.

`PSIMMAP4`는 모든 검출 특징·원본 픽셀과 인자·자세 초기값·현재 자세와 지도점·
선형화 기준·최적화 설정·구간 식별자와 보정값·평가한 영상쌍·다음 트랙 및 출력 ID를
보존합니다. 복원은 식별자를 검사하고 검색 색인을 재구성하며 성공·실패한 대응을
유지합니다. 입력 보관 한도 안에서 새 입력으로 선택한 계산 구간을 확장할 수
있습니다. 항법 상태는 복원하지 않습니다. 이전 그래프 전용 파일은 저장된 관측
연결의 재선형화를 지원합니다.

검사한 특징 프레임은 한 번 읽어 저장하고 같은 인코딩 바이트로 대응 관계의
식별자를 만듭니다. 같은 폴더의 임시 파일과 **1MiB** 전송 버퍼로 색인 형식을
유지하며, 완성된 파일을 원자적으로 교체합니다. 원본 무결성 검사에 실패하면
발행된 기존 파일은 유지됩니다.

## Native matching input memory / 네이티브 매칭 입력 메모리

HashBoW retrieval and pinned OpenGV matching evaluate new chronological queries;
late frames update retrieval from their timestamp. Previously evaluated pairs,
including failures, keep their decisions. New pairs derive their RANSAC seed from
ordered timestamps and image digests. The compact-input quota also bounds retained
matching numeric payload. Transient descriptors are released after each lease.

HashBoW 검색과 고정 OpenGV 대응은 시간순 새 질의를 처리하며 늦은 프레임은 해당
시각부터 검색을 갱신합니다. 평가한 영상쌍은 실패까지 유지합니다. 새 쌍의 RANSAC
시드는 정렬된 시각과 영상 해시에서 구합니다. 대응 수치 데이터에도 압축 입력
한도를 적용하며 임시 기술자는 읽기를 마치면 해제합니다.

## Sparse covariance computation / 희소 공분산 계산

The full-graph backend uses Eigen sparse LDLT with selected inverse entries and
exact solves for other queries. The conditional backend uses the retained QR/SVD
factorization described above. Both include shared pose uncertainty and preserve
unobserved directions; neither changes the admission threshold.

전체 그래프 백엔드는 Eigen 희소 LDLT, 선택 역행렬 원소와 나머지 질의의 직접
풀이를 사용합니다. 조건부 백엔드는 위의 QR/SVD 분해 정보를 사용합니다. 두 방식
모두 공유 자세 불확실성과 미관측 방향을 보존하며 채택 임계값을 변경하지 않습니다.

## Linearized submap information / 선형화한 부분 지도 정보

`SubmapInformation` and `SubmapFactorization` retain original measurement
ownership, exact affine constraints, conditional shared noise and null directions.
Replacement rebuilds affected dependencies from source rows. An unchanged,
already compacted row block can be reused after coordinate permutation;
newly stacked or restored rows undergo the rank-aware calculation. This preserves
the source-scale precision rule and supplies no prior or damping.

`SubmapInformation`과 `SubmapFactorization`은 원본 측정의 소유 관계, 정확한
아핀 제약, 조건부 공유 잡음과 미관측 방향을 보존합니다. 교체 시 원본 행으로
영향받는 관계를 재구성합니다. 이미 압축한 불변 행 묶음은 좌표 순서를 바꾼 뒤에도
재사용할 수 있고, 새로 쌓거나 복원한 행은 계수를 확인하며 계산합니다. 원본 크기에
따른 수치 정밀도 규칙을 유지하고 사전정보나 감쇠를 추가하지 않습니다.

Implementation: [selected sources](../../native/basalt/sources.json),
[persistent mapper](../../native/basalt/persistent_mapper.cc),
[mapping window](../../native/basalt/native_mapping_window.hpp),
[information](../../native/basalt/submap_information.hpp),
[factorization](../../native/basalt/submap_factorization.hpp).
