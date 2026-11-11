# Joint uncertainty for selected recorded poses

Four original-time sonar poses share one native IMU/DVL/pressure posterior.
`NativeSurfaceSnapshot.joint_marginal` propagates query coefficients backwards
through retirement conditionals. It accumulates shared noise once per source
and includes the current native root, exact constraints and free directions.
The default recovery budget remains **32 MiB**.

On the captured 85.01 s snapshot, the original query rejects three/four selected
poses for workspace capacity. The query-adjoint calculation recovers all four
using a **10,915,968-byte numeric workspace bound**. This is an allocation bound,
not process RSS or a measured speedup. For the original supported one/two-pose
queries, covariance differs by at most **6.22e-15**. A 250-node analytic chain,
dense systems, free gauges and exact-support tests cover the same operation.

The diagnostic snapshot option preserves all **271** native states. The maximum
position/quaternion/velocity/bias component change is **2.13e-13**. The same raw
IMU, bottom-locked DVL and pressure are used; camera, Ping and sonar point factors
do not enter this navigation control. Native client close terminates the worker
with SIGTERM (-15); the driver completes with 0. This is not zero-exit lifecycle
qualification.

## Remaining physical uncertainty

The unknown-bias control has 2.67 cm trajectory position RMSE and 0.137 degree
rotation RMSE, after initial yaw/translation gauge alignment only. Its local
observable pose 3-sigma values remain large: position **3.13–10.95 m**, rotation
**42.6–155.9 degrees**. Maximum between-time correlation is **0.999917**. These
are frozen-weight Gauss–Newton diagnostics; such large angular uncertainty is
not calibrated global confidence. They do not qualify a map or readiness.

The same complete source scene at exact renderer poses gives power residual RMS
0.963–1.005. Using native pose means gives 14.2–73.4 despite small pose errors.
The retained 80 old-record and 20 exact-record background cases also show why a
fixed, unmeasured flat seafloor cannot represent the whole image. At 85.01 s,
replacing that background changes RMS from 58.78 to 1.005. Other views still
require missing/incorrect reflectors to be resolved. Reference geometry/poses
enter these assessments only. Every original case remains; no map is accepted.

## Reproduction and scope

With the public library source and NumPy available:

```sh
python docs/diagnostics/native-joint-query-20261101/verify.py
```

The verifier replays the complete recorded native snapshot, compares the earlier
supported queries, preserves the earlier capacity failures and checks all 271
retained state times/poses. It validates the retained 100 background-case
contracts and hashes; it does not reacquire the world or recompute those 100
power images. Original raw fields and their independent source checks are in
the [renderer-sampling record](../sonar-renderer-sampling-20261101/README.md).
`original_query.py` preserves the public pre-change implementation. Source and
third-party notices remain applicable; no new dependency is introduced.

## 한국어

네 소나 시점은 하나의 IMU·DVL·수압 추정 결과를 공유합니다. 조회할 변수의
계수만 과거 상태의 조건부 관계에 역전파해 공분산을 계산합니다. 현재 상태의
불확실성·공유 잡음·정확한 제약·자유 방향을 유지하며 복원 예산은 **32 MiB**입니다.
기존 계산이 메모리 한도로 거부한 네 시점을 계산 배열 상한 **10,915,968바이트**로
복원합니다. 실제 프로세스 메모리나 실행 속도 측정값은 아닙니다.

원본에서 가능했던 한두 시점 조회의 공분산 차이는 최대 **6.22e-15**이고,
진단 사본 생성 전후 **271개** 상태의 위치·자세·속도·바이어스 성분 차이는 최대
**2.13e-13**입니다. 같은 원본 IMU·바닥 추적 DVL·수압을 사용하며 카메라·Ping·
소나 점 제약은 항법 입력에 포함하지 않습니다. 클라이언트 종료는 워커에
SIGTERM(-15)을 사용하고 실행 스크립트는 0으로 끝납니다.

바이어스를 모르는 대조군의 위치 RMSE는 2.67 cm, 자세 RMSE는 0.137°이지만
국소 3시그마는 위치 **3.13–10.95 m**, 자세 **42.6–155.9°**로 남습니다.
시점 간 최대 상관은 **0.999917**입니다. 큰 각도에서 이 국소 근사를 보정된
전역 신뢰도로 해석하지 않으며 지도나 준비 완료를 인정하지 않습니다.

실제 촬영 자세에서 전체 장면의 전력 잔차 RMS는 0.963–1.005이고, 네이티브
평균 자세에서는 14.2–73.4입니다. 보존한 80개 기존 기록·20개 정확한 촬영 기록의
배경 대조도 함께 남깁니다. 85.01초에서는 미측정 평면 해저를 실제 해저로
대조하면 RMS가 58.78에서 1.005로 바뀝니다. 다른 시점의 누락·부정확 반사체와
공유 자세 불확실성은 별도 해결이 필요합니다. 정답은 평가에만 사용합니다.

검증기는 실제 네이티브 사본의 공분산 조회와 기존 용량 실패, 271개 상태를
재검사합니다. 배경 대조 100개의 계약·해시는 확인하지만 100개 전력 영상을
다시 계산하거나 현장을 재촬영하지는 않습니다. 원본 음선·전력 검사는 위
렌더러 샘플링 기록에 있습니다. 기존 실험·기준·잡음·지도점 수용 조건은 유지합니다.
