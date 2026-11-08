# Map/local mean coordinates

The persistent native mapper publishes an optional
`map_alignment_transport: source_pinned_map_local_mean_v1` acknowledgement.
Each completed map carries `mapping_alignment` with its epoch, input revision,
latest anchor timestamp and a 4 × 4 `map_from_local` rigid transform. Pending or
failed maps provide no transform.

The anchor's optimized pose and source VIO pose come from the same pinned input
revision. The mapper applies its existing initial-position/yaw gauge to the
optimized pose, then computes
`T_map_local = T_map_anchor * inverse(T_local_anchor)`.
This follows the separation of local odometry and map localization described
in [ROS REP 105](https://github.com/ros-infrastructure/rep/blob/master/rep-0105.rst).
It does not add a measurement to the estimator.

Call `LiveClient.map_aligned_pose(state, map_packet)` on the output owner thread
for each matched state/map pair, in timestamp order. It returns the IMU pose
mean in map coordinates, or `None` when the worker has not acknowledged this
contract or the map has no alignment. Each new revision first applies to its
delivered pair; previously emitted states are unchanged. Repeated revisions
must retain the same anchor and transform. Cross-epoch, out-of-order, future,
non-rigid and inconsistent revisions are rejected. Create a new client after a
worker restart. No TF tree or application dependency is introduced.

The local VIO state, observations, map points and original uncertainty are
unchanged. Shared map/VIO correlations are not propagated into a corrected-pose
covariance. The result therefore reports `uncertainty_available: false` and
`navigation_ready: false`. Mean alignment does not restore information lost
during unobservable motion. Source API checks, native worker build checks and
physical accuracy qualification are separate requirements.

## 지도와 로컬 평균 좌표

지속 네이티브 매퍼는 선택 기능으로
`map_alignment_transport: source_pinned_map_local_mean_v1`을 알립니다.
완료된 지도에는 구간 식별자·입력 리비전·최신 기준 자세의 시각과
4 × 4 강체 변환 `map_from_local`을 담은 `mapping_alignment`가 있습니다.
준비 중이거나 실패한 지도는 변환을 제공하지 않습니다.

기준 시각의 최적화 자세와 원본 VIO 자세는 같은 고정 입력 리비전에서 가져옵니다.
최적화 자세에 기존 초기 위치·방위 기준을 적용한 뒤
`T_map_local = T_map_anchor * inverse(T_local_anchor)`를 계산합니다.
이는 [ROS REP 105](https://github.com/ros-infrastructure/rep/blob/master/rep-0105.rst)의
로컬 오도메트리와 지도 위치 추정 분리 방식에 따릅니다. 추정기에 새 측정값을
추가하지 않습니다.

출력 소유 스레드에서 같은 시각의 상태·지도 쌍마다
`LiveClient.map_aligned_pose(state, map_packet)`를 시각 순으로 호출합니다.
지도 좌표계의 IMU 평균 자세를 반환하며, 작업자의 기능 확인이나 지도 변환이
없으면 `None`을 반환합니다. 새 리비전은 처음 전달된 쌍부터 적용하고 이미
발행한 상태는 바꾸지 않습니다. 같은 리비전의 기준 시각·변환은 불변입니다.
다른 구간, 역순, 미래 시각, 강체가 아닌 변환과 일관되지 않은 리비전은 거부합니다.
작업자를 재시작하면 새 클라이언트를 만드세요. TF 트리나 앱 의존성은 추가하지
않습니다.

원래 VIO 상태·관측·지도점·불확실성은 유지합니다. 지도와 VIO가 공유하는
상관관계를 보정 자세의 공분산으로 전파하지 않으므로 결과는
`uncertainty_available: false`, `navigation_ready: false`입니다.
평균 좌표 보정으로 관측 불가능한 운동의 정보가 복원되지는 않습니다.
소스 API 검사, 네이티브 작업자 빌드 검사와 물리 정확도 검증은 각각 필요합니다.
