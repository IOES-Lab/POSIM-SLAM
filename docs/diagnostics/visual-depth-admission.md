# Visual-map depth support / 시각 지도의 깊이 지지

![Sparse cloud and same-pose RGB](../images/visual-depth-admission.png)

[Complete candidate local-frame PLY](../data/visual-depth-admission.ply).

The native archive uses optimized-window full-weight camera observations and an SVD of pixel-noise-whitened point Jacobians. It requires rank three and a positive three-sigma inverse-range interval, then three distinct eligible inlier exposures. Active points that lose support are withdrawn and require fresh admission. Retired qualified points retain their last coordinates. Streaming and recorded Ping-patch inspection share this rule. Camera poses are fixed in this conditional test, so pose uncertainty is excluded; it cannot certify map accuracy. There is no metric distance cap or truth-based trimming.

Separate 1x ROS camera/IMU/DVL runs on the same raw recording export 1,412 control and 1,418 candidate points. Maximum physical-surface distance falls from 33.938 m to 1.784 m; median/p90 changes from 0.00869/0.08143 m to 0.00943/0.08427 m. Both retain 122 points outside 10 cm. Position RMSE is 0.00898/0.01414 m using initial yaw/translation only and scale one. Asynchronous initialization differs, so this is not an identical-state ablation. Both pass DVL-silence clearing/new-frame recovery and shutdown; new SDK/installed-GUI acceptance is separate.

The two extreme control IDs have under 0.081-degree parallax and relative depth sigma above one. Exact snapshot sidecars establish their IDs without inferring from a historical union. The candidate's worst remaining ID has 13 observations and 1.576-degree parallax but lies 1.784 m from collision geometry. Depth conditioning does not prove a static physical feature. Dynamic particles, caustics and false associations need investigation. The map remains unqualified, with poor target/wall coverage.

The figure uses the same actual isometric camera pose/perspective, Turbo -5 to 0 m and 64 pt² markers. RGB includes attenuation, suspended particles and seabed caustics. All exported points, including offscreen ones, enter surface scoring. Analytical covariance/degeneracy and rotation/scale invariance probes, archive withdrawal/readmission tests and 48 affected native tests pass. No new third-party dependency is added; existing Basalt/Eigen notices apply.

## 한국어

네이티브 지도는 현재 최적화 창의 정상 가중치 관측으로 픽셀 잡음 백색화 점 Jacobian을 만들고 SVD로 완전 계수와 양의 역거리 3시그마 조건을 확인합니다. 조건을 통과하는 서로 다른 관측 세 장이 필요하며 활성 점이 깊이 지지를 잃으면 제거하고 다시 검사합니다. 추정 창에서 빠진 적합 점은 마지막 좌표를 유지합니다. 스트리밍과 기록 Ping 패치 검사의 기준은 같습니다. 카메라 자세를 고정한 조건부 검사여서 자세 불확실성은 포함하지 않으며 지도 정확도를 보증하지 않습니다. 거리 상한이나 정답 형상을 이용한 제거는 없습니다.

동일 원본의 별도 1배속 카메라·IMU·DVL ROS 실행에서 점 수는 1,412/1,418개, 최대 표면 거리는 33.938/1.784 m입니다. 중앙값·90백분위 거리는 0.00869/0.08143 m와 0.00943/0.08427 m이며 두 실행 모두 10 cm 밖에 122점이 남습니다. 위치 RMSE는 0.00898/0.01414 m입니다. 초기 방위각·위치만 정렬하고 축척은 1입니다. 비동기 초기화 시각이 달라 동일 상태의 엄밀한 대조 실험으로 해석하지 않습니다. DVL 통신 중단의 지도 비우기·새 프레임 복구와 종료는 통과했으며 새 SDK·설치형 GUI 검증은 별도입니다.

대조군의 큰 이상점 두 개는 시차가 0.081° 미만이고 상대 깊이 표준편차가 1을 넘습니다. 정확한 스냅샷 ID 기록으로 대응 관계를 확인했습니다. 수정 후 최대 이상점은 관측 13개·시차 1.576°에도 실제 충돌 표면에서 1.784 m 떨어집니다. 깊이 추정 조건만으로 정적 물체의 대응점을 증명할 수 없습니다. 입자·집광무늬와 잘못된 대응점 가능성을 조사해야 합니다. 물체·벽 복원은 약하며 지도는 여전히 미통과입니다.

점군과 RGB는 실제 같은 등각 방향 카메라 원근을 사용하며 Turbo -5~0 m와 점 면적 64 pt²를 사용합니다. RGB에는 수중 감쇠·입자·해저 집광무늬가 포함됩니다. 화면 밖 점도 표면 평가와 PLY에 보존합니다. 해석적 공분산·퇴화·회전/축척 불변성, 지도 제거·재채택 검사와 관련 네이티브 시험 48개를 통과했습니다. 새 의존성은 없으며 기존 Basalt·Eigen 고지를 유지합니다.
