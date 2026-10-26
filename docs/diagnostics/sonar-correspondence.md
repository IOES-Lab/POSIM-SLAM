# Pose-conditioned sonar search geometry / 자세 기반 소나 탐색 기하

`sonar_correspondence.project_return_arc` projects a measured range/azimuth's
sampled elevation arc through two IMU poses and the sonar extrinsics. It retains
both elevation branches and reports target sensor support explicitly. Stationary
views preserve the full elevation ambiguity. Rotating lever arms are included.
It uses NumPy and existing geometry validation; no new dependency or license is
introduced. This is an experimental independent-library function, not enabled
in the live frontend.

`sonar_correspondence.project_return_arc`는 측정된 거리·방위각에 가능한 수직각을
샘플링하고, 두 IMU 자세와 소나 장착 변환으로 다음 관측 위치를 계산합니다.
수직각의 양쪽 방향을 유지하고 다음 시야각 안에 있는지 표시합니다. 정지한
관측에서는 수직각의 모호함이 그대로 남으며, 회전하는 센서 장착 위치도
반영합니다. NumPy와 기존 기하 검증을 사용하므로 추가 의존성·라이선스는
없습니다. 독립 라이브러리의 실험 함수이며 라이브 추적에는 적용하지 않았습니다.

The output is a **nominal search locus**, not a measured landmark or calibrated
uncertainty gate. Caller poses must share a metric map frame. Pose covariance,
range/azimuth uncertainty, continuous-arc bounds and physical scattering identity
remain caller responsibilities. Do not reject observations simply because they
lie outside the sampled curve. It neither matches image patches nor returns a
resolved elevation. Future matching must preserve ambiguity and avoid counting
the motion prior twice in joint optimization.

출력은 **추정 자세에 따른 탐색 곡선**이며, 측정된 랜드마크나 보정된 불확실성
기준이 아닙니다. 입력 자세는 동일한 미터 단위 지도 좌표계를 사용해야 합니다.
자세·거리·방위각 불확실성, 샘플 사이의 곡선 범위, 실제 반사점의 동일성은
별도로 처리해야 합니다. 샘플 곡선 밖이라는 이유만으로 관측을 거부하면 안
됩니다. 영상 패치 대응이나 수직각 결정은 수행하지 않습니다. 후속 대응 탐색은
모호함을 유지하고 결합 최적화에서 이동 정보를 중복 계산하지 않아야 합니다.

Four analytic tests cover stationary ambiguity, vertical translation, rotating
lever arms/global frame invariance, and invalid geometry/sampling budgets.
Recorded-output-pose diagnostics show substantial image-flow departures from
the nominal curve; they motivate search-guided correspondence experiments, not
an acceptance claim. No simulator truth enters this function.

해석적으로 계산 가능한 테스트 4개로 정지 시 모호함, 수직 이동, 회전하는
장착 위치와 전역 좌표 변환 불변성, 잘못된 기하 및 샘플 수를 검증했습니다.
기록된 출력 자세를 사용한 진단에서는 영상 추적이 예측 곡선에서 벗어나는
경우가 확인됐습니다. 이는 탐색 보조 대응 실험의 근거이며 성능 합격을 뜻하지
않습니다. 이 함수는 시뮬레이터 참값을 입력받지 않습니다.

`rank_return_patches` ranks distinct patch locations around a supplied locus by
zero-mean normalized correlation. It returns multiple modes and their scores;
it does not accept an association. Search size, patch radius and mode count are
bounded. Positive gain/offset, repeated patterns and unavailable patches are
covered by three additional tests. A single returned mode means only that the
bounded search found one mode, not that the physical match is unique. Live
tracking integration, reverse consistency and unused-view validation are pending.

`rank_return_patches`는 입력 탐색 곡선 주변의 패치 위치를 평균 제거 정규화
상관도로 정렬합니다. 여러 후보와 점수를 반환하며 대응을 확정하지 않습니다.
탐색 크기·패치 반경·후보 수에 상한을 둡니다. 양의 밝기 배율과 오프셋,
반복 무늬, 사용할 수 없는 패치를 추가 테스트 3개로 검증했습니다. 후보가
하나라는 것은 제한된 탐색에서 하나를 찾았다는 뜻이며 실제 대응의 유일성을
보장하지 않습니다. 라이브 추적 연결, 역방향 일관성과 후속 관측 검증은 남아 있습니다.
