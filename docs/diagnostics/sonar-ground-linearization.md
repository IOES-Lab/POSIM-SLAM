# Ground-factor linearization / 해저 제약 선형화

The experimental raw-ground factor differentiates the current sensor pose and
unknown plane. The plane's initial normal defines its rotation coordinates;
it is not fixed surface geometry. The full correlated range covariance is
whitened before eliminating the three plane variables.

실험용 원본 해저 제약은 현재 센서 자세와 미지 평면에서 미분합니다. 평면의
초기 법선은 회전 좌표의 기준이며 고정된 표면 형상이 아닙니다. 상관된 거리
공분산으로 백색화한 뒤 평면 변수 세 개를 소거합니다.

The stored normal is `n = Exp(B_ref * delta) * n_ref`. Its differential uses
the SO(3) left Jacobian at `B_ref * delta`. A retained pose similarly stores
an accumulated rotation vector, so its current geometric derivative is mapped
through that vector's left Jacobian. Fresh poses use an identity mapping.
When a group retires, the whole plane is eliminated and the retained prior is
rebased in the existing pose coordinates. No known plane or pose is supplied.

저장한 법선은 `n = Exp(B_ref * delta) * n_ref`입니다. 미분에는
`B_ref * delta`에서의 SO(3) 왼쪽 야코비안을 사용합니다. 보유 자세의 누적
회전 벡터에도 같은 좌표 변환을 적용하고 새 자세에는 단위 변환을 사용합니다.
그룹을 제거할 때 평면 전체를 소거하고 보유 자세 좌표에 맞춰 사전 제약의
상수항을 변환합니다. 알려진 평면이나 자세를 관측값으로 제공하지 않습니다.

## Numerical checks / 수치 검사

A synthetic test starts with a tilted plane reference and updates the normal
to horizontal. Frozen initial geometry produces nonzero horizontal/yaw
information even though numerical differentiation gives zero. Current-geometry
rows reduce the maximum offending information entry from 4,268 to
`2.8e-13`; maximum whitened derivative error is `2.9e-8`.

합성 검사는 경사진 평면 기준에서 시작해 법선을 수평으로 갱신합니다.
초기 형상을 고정하면 수치 미분이 0인 수평 이동·방위 방향에도 정보가 남습니다.
현재 형상으로 계산하면 해당 정보 행렬 성분의 최대 절댓값이 4,268에서
`2.8e-13`으로 줄며, 백색화 미분의 최대 오차는 `2.9e-8`입니다.

Further checks cover nonzero accumulated pose/plane rotations, six rigid-world
gauge directions, and joint marginalization with a nonzero retained delta.
The marginal matrix and rebased vector agree with independent dense elimination.
Native IMU/DVL/pressure tests also exercise repeated state retirement.

추가 검사는 자세·평면의 누적 회전이 0이 아닌 경우, 세계 좌표 전체의 강체
변환에 해당하는 여섯 자유도, 보유 상태 변화량이 0이 아닌 공동 주변화를
포함합니다. 주변화 행렬과 변환한 벡터를 독립적인 밀집 소거와 대조합니다.
네이티브 IMU·DVL·수압 검사에서는 반복 상태 제거도 수행합니다.

## Accuracy limit / 정확도 한계

This numerical correction does not qualify terrain SLAM. In recorded case C,
online position RMSE with actual ground entries is 1.389 m versus 0.016 m
without ground factors. The synthetic horizontal-plane ablation is 0.086 m;
its ranges are generated from reference poses, so it is only a model diagnosis.
Four updated recorded comparisons and matched RGB/point clouds are in
[native ground fusion](sonar-ground-live.md).

이 수치 수정만으로 지형 SLAM을 검증하지 않습니다. C 기록의 실제 해저 관측을
사용한 온라인 위치 RMSE는 1.389 m이며 해저 제약 없는 경우는 0.016 m입니다.
합성 수평면 진단은 0.086 m지만 정답 자세로 거리를 생성하므로 모델 진단에만
사용합니다. 네 기록의 최신 비교와 같은 시점 RGB·점군은
[네이티브 해저 융합](sonar-ground-live.md)에 있습니다.

Surface association and acoustic onset uncertainty still need validation.
These checks do not establish statistical consistency of the complete
fixed-lag estimator or camera-free navigation without DVL.

표면 연결과 음향 반사 시작점의 불확실성 검증이 남아 있습니다. 이 검사로
전체 고정 구간 추정기의 통계적 일관성이나 DVL 없는 카메라 미사용 항법을
입증하지 않습니다.

Sources / 소스:
[factor](../../native/basalt/sonar_ground_group.hpp),
[numerical probe](../../native/basalt/sonar_ground_probe.cc),
[regression tests](../../tests/test_sonar_ground_joint.py).
