# Joint sonar planes / 소나 면과 자세의 결합 인자

`sonar_plane_joint.linearize_plane_patch` supplies experimental joint rows for
explicitly associated returns on an arbitrary plane, including vertical faces.
A frame supplies its exact timestamp, unique source IDs, measured ranges and the
actual unit rays across each vertical fan. The caller supplies estimated IMU
poses, sonar extrinsics, a unit plane normal and signed plane distance.

The function evaluates the earliest valid ray/plane intersection. Tied minima
are rejected because the minimum can have different derivatives on either side
of a branch switch. Residuals are predicted minus measured range. Jacobian
columns are each frame's map translation and right IMU rotation, followed by two
additive unit-normal tangent coordinates and signed distance. A full range
covariance whitens all rows together, retaining spatial and temporal correlations.
The plane is a shared estimated variable; its fitted normal is not a new prior.

Optional `support_points` and `max_support_distance` require predicted hits inside
the observed projected convex hull and close to actual samples. This avoids
unbounded extrapolation and large unsampled interior holes under a local-plane
assumption. The observed hull is not a physical object boundary. The caller must
preserve correlations between support points, poses and plane parameters.

`marginal_plane_information` integrates an explicitly supplied full pose prior:
for a whitened Jacobian `[A B]`, the plane information is
`Bᵀ (I + A P Aᵀ)⁻¹ B`. `P` includes inter-frame pose correlations. Zero `P` gives
the fixed-pose conditional diagnostic. The result is local and omits association,
beam-switch and acoustic-model uncertainty. This formula assumes the pose prior
is independent of these sonar rows. A posterior already containing the same
measurements must instead remain in the joint optimizer to avoid double counting.
No pose covariance is invented or estimated by this helper.

These APIs do not extract correspondences, recover an object boundary, update a
live estimator or qualify a map. Tests cover numerical derivatives with a rotated
lever arm, world-frame gauge freedoms after eliminating the unknown plane, full
correlated marginal covariance versus the dense joint inverse, finite-support
rejection, ambiguous rays and invalid identities/covariances.

---

`sonar_plane_joint.linearize_plane_patch`는 대응이 명시된 반사점들을 임의 방향
평면과 자세에 함께 연결하는 실험용 인자입니다. 수직 물체 면도 표현합니다.
프레임마다 정확한 시각, 고유한 소스 ID, 측정 거리, 수직 팬의 실제 단위 광선을
입력합니다. 호출자는 추정 IMU 자세, 소나 장착 변환, 단위 법선과 평면의 부호 있는
거리를 제공합니다.

유효한 광선·평면 교차 중 첫 거리를 사용합니다. 여러 광선의 최소 거리가 같으면
가지 전환 양쪽의 미분이 다를 수 있으므로 거부합니다. 잔차는 예측 거리에서 측정
거리를 뺀 값입니다. 야코비안 열은 프레임별 지도 좌표 이동과 IMU 우측 회전,
이어서 단위 법선에 더하는 접선 좌표 두 개와 평면 거리입니다. 전체 거리 공분산으로
행들을 함께 백색화하여 공간·시간 상관을 유지합니다. 평면은 공유 추정 변수이며,
맞춘 법선을 별도의 사전 정보로 다시 넣지 않습니다.

선택 입력인 `support_points`와 `max_support_distance`는 예측 교차점이 관측
점들의 투영 볼록 껍질 안에 있고 실제 표본과 가까운지 검사합니다. 국소 평면 가정
아래 무한한 외삽과 내부의 큰 미관측 구멍을 거부합니다. 관측 껍질은 실제 물체
경계가 아닙니다. 지지점·자세·평면 사이의 상관은 호출자가 유지해야 합니다.

`marginal_plane_information`은 명시적으로 입력한 전체 자세 사전 공분산을
적분합니다. 백색화한 야코비안이 `[A B]`이면 평면 정보는
`Bᵀ (I + A P Aᵀ)⁻¹ B`입니다. `P`에는 프레임 간 자세 상관도 포함합니다.
`P`가 0이면 자세 고정 조건부 진단이 됩니다. 결과는 국소 미분에 한정되며 대응,
광선 가지 전환, 음향 모델 불확실성은 포함하지 않습니다. 자세 사전 정보는 이
소나 관측과 독립이어야 합니다. 같은 관측을 이미 포함한 사후 분포는 중복 계산을
피하도록 결합 최적화 안에서 다뤄야 합니다. 이 함수가 자세 공분산을 만들어 내거나
추정하지는 않습니다.

이 API는 대응 추출, 실제 물체 경계 복원, 실시간 추정기 갱신, 지도 품질 판정을
수행하지 않습니다. 회전된 장착 위치에서의 수치 미분, 미지 평면 제거 후 세계 좌표
게이지 자유도, 전체 결합 행렬 역행렬과 상관 공분산의 일치, 유한 지지 영역 거부,
동률 광선, 잘못된 식별자와 공분산을 테스트합니다.

## Native grouped optimizer / 네이티브 그룹 최적화

`native/basalt/sonar_plane_group.hpp::makeOwnedSonarPlanes` connects explicitly
associated raw returns to the existing Basalt joint optimizer. One acoustic group
can contain several plane variables, with one covariance matrix spanning all
frames, sources and surfaces. Every frame/source pair has one owner; a source's
plane membership remains fixed during the group. This prevents treating correlated
surfaces as independent factors or counting one frame in multiple acoustic groups.
The existing global budget remains eight owned planes. A group has at most
16 frames, 512 range rows and 512 vertical rays per row.

Native pose columns use Basalt's map translation and map-left rotation. Plane
increments use the owner's fixed-reference rotation coordinates. Accumulated
pose/plane increments are differentiated at their current values. Every plane
is back-substituted in the existing whole-trial transaction; retirement eliminates
all group planes into the same state prior and removes them atomically. Full
range covariance is whitened before elimination. Tied minimum rays reject the
whole correlated group. Legacy one-plane visual/Ping groups use the same path.

The native probe covers two surfaces at shared timestamps, NumPy agreement,
nonzero accumulated increments, joint optimization, correlated marginalization,
atomic prior installation and replay rejection. Frozen recorded sonar groups
exercise the geometry path without a synthetic pose prior or truth. This does not
yet install an online source-association owner or a ROS packet handler, qualify
finite object extent, estimate real sensor noise, or validate trajectory/map
accuracy. Native finite-support admission remains the caller's responsibility.

`native/basalt/sonar_plane_group.hpp::makeOwnedSonarPlanes`는 대응이 명시된 원시
거리 관측을 기존 Basalt 결합 최적화에 연결합니다. 하나의 음향 그룹에 여러 평면
변수를 담고, 모든 프레임·소스·면을 포괄하는 공분산 행렬 하나를 사용합니다.
프레임·소스 쌍의 소유자는 하나이며, 그룹 안에서 소스의 평면 소속은 고정됩니다.
상관된 면을 독립 인자로 취급하거나 같은 프레임을 여러 음향 그룹에 중복 반영하지
않습니다. 전체 소유 평면 예산은 기존의 8개를 유지합니다. 그룹당 최대 프레임은
16개, 거리 관측은 512행, 행마다 수직 광선은 512개입니다.

네이티브 자세 열은 Basalt의 지도 좌표 이동과 지도 좌표 좌측 회전을 사용합니다.
평면 증분은 소유자의 고정 기준 회전 좌표를 사용합니다. 누적된 자세·평면 증분의
현재 값에서 미분합니다. 모든 평면을 기존의 전체 시도 트랜잭션 안에서 역대입하고,
그룹이 퇴역하면 모든 평면을 같은 자세 사전 정보로 주변화한 뒤 원자적으로 제거합니다.
전체 거리 공분산은 주변화 전에 백색화합니다. 최소 거리 광선이 동률이면 상관 그룹
전체를 거부합니다. 기존 단일 평면 시각/Ping 그룹도 같은 경로를 사용합니다.

네이티브 검사기는 같은 시각에 관측한 두 면, NumPy와의 일치, 0이 아닌 누적 증분,
결합 최적화, 상관 주변화, 사전 정보의 원자적 설치와 재적용 거부를 확인합니다.
고정된 실제 소나 관측 그룹으로 가상의 자세 사전 정보나 참값 없이 기하 경로를
검사합니다. 온라인 소스 대응 소유자와 ROS 패킷 처리는 아직 연결하지 않았습니다.
유한한 물체 범위, 실제 센서 잡음, 궤적·지도 정확도도 검증하지 않았습니다.
네이티브의 유한 지지 영역 채택은 호출자가 책임져야 합니다.
