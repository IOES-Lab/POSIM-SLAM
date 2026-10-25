# Ground association / 해저 관측 연결

The experimental ground store tests whether a new frame is consistent with an
existing unknown plane. A failed connection starts a new plane when capacity is
available; the worker reports registration failures separately. It does not
silently turn rejected connections into missing sensor measurements.

실험용 해저 저장소는 새 프레임이 기존 미지 평면과 일치하는지 검사합니다.
연결이 실패하면 여유가 있을 때 새 평면을 만들고, 등록 자체가 실패한 경우는
별도로 보고합니다. 연결 실패를 관측 누락으로 조용히 처리하지 않습니다.

For fixed current poses, it profiles the three free plane coordinates in the old
system and in the system with the new frame. Both use the same lower-Cholesky
whitening and full correlated range covariance. The increase in minimum squared
residual is a conditional predictive innovation. It includes plane-fit uncertainty
and noise shared across frames. No fitted normal becomes an independent prior.

현재 자세를 조건으로 기존 관측과 새 프레임을 합친 관측에서 각각 평면의 자유
좌표 세 개를 최소제곱으로 소거합니다. 같은 하삼각 Cholesky 백색화와 상관된
거리 공분산을 사용합니다. 최소 잔차 제곱합의 증가는 조건부 예측 오차이며,
평면 적합의 불확실성과 프레임 사이의 공통 잡음을 포함합니다. 적합한 법선을
독립된 사전 관측으로 추가하지 않습니다.

With `n` new range rows and tail parameter `k`, the limit is
`n + k sqrt(2n) + k²`. This is the
[Laurent–Massart Gaussian chi-square bound](https://projecteuclid.org/journals/annals-of-statistics/volume-28/issue-5/Adaptive-estimation-of-a-quadratic-functional-by-model-selection/10.1214/aos/1015957395.pdf)
with `x = k²/2`. The conditional linear Gaussian upper-tail probability is bounded
by `exp(-k²/2)`. With `n=50`, `k=5`, the limit is 125, rather than the 1,250 implied
by a per-row five-sigma RMS rule. This probability is not a calibrated real-sonar
false-rejection rate: pose uncertainty, nonlinear surface changes and acoustic
model error are not included.

새 거리 관측 `n`개와 꼬리 계수 `k`에 대해 문턱은 `n + k sqrt(2n) + k²`입니다.
조건부 선형 가우시안 모델에서 상단 꼬리 확률의 상계는 `exp(-k²/2)`입니다.
`n=50`, `k=5`이면 문턱은 125이며, 관측별 5시그마 RMS 규칙의 1,250과 다릅니다.
자세 불확실성·비선형 표면 변화·음향 모델 오차를 포함하지 않으므로 실제 소나의
오거절 확률을 보정한 값으로 해석할 수 없습니다.

Native tests compare the score with independently formed dense predictive
covariance, check beam-order and plane-coordinate invariance, retain uncertainty
from weak old observations, and reject distributed disagreement accepted by the
RMS rule. Geometry/marginalization and native sensor regressions also pass.

네이티브 검사는 독립적으로 만든 밀집 예측 공분산과의 일치, 빔 순서·평면 좌표
변경에 대한 불변성, 약한 기존 관측의 불확실성 유지, RMS 규칙이 허용한 분산된
불일치의 거절을 확인합니다. 기하·주변화와 네이티브 센서 회귀 검사도 통과합니다.

In the [four recorded comparisons](sonar-ground-live.md), the gate accepts all
52 tested connections. The four ground and four no-ground state sequences remain
byte-identical to the preceding implementation. This change does **not** improve
the recorded trajectory or qualify terrain-aided SLAM. Surface-model error and
spatially correlated acoustic uncertainty remain unresolved.

[네 기록 비교](sonar-ground-live.md)에서는 검사한 연결 52개가 모두 통과했습니다.
해저 융합 네 실행과 대조군 네 실행의 상태열은 이전 구현과 바이트 단위로 같습니다.
이 변경으로 기록의 궤적이 개선되거나 지형 보조 SLAM이 검증된 것은 아닙니다.
표면 모델 오차와 공간적으로 상관된 음향 불확실성의 처리가 남아 있습니다.

Sources / 소스: [predictive statistic](../../native/basalt/ground_predictive.hpp),
[owner store](../../native/basalt/sonar_ground_store.hpp),
[independent checks](../../tests/test_sonar_ground_joint.py).
