# Native first-state IMU bias calibration

This paired control reuses the same 183 recorded sonar frames, raw IMU,
90 bottom-lock DVL and 90 pressure factors. Camera, Ping, acoustic factors and
reference geometry do not enter navigation. All 271 state times, native worker,
raw frontend output, stochastic noise and gravity remain unchanged.

The captured SDF declares a noiseless IMU. Its explicit model certificate
supplies a zero bias mean and covariance for the **first state only** through
`InitialImuBiasCalibration`. The native owner replaces the generic initial-bias
prior and retains subsequent stochastic evolution. Zero ROS covariance or low
motion is never used as a calibration certificate. A real or noisy sensor needs
an externally calibrated mean and full covariance.

| Original time (s) | Unknown-bias orientation 3σ (°) | Certified orientation 3σ (°) | Certified position 3σ (m) |
| --- | --- | --- | --- |
| 25.01 | 42.55 | 1.32 | 0.132 |
| 45.01 | 80.34 | 2.95 | 0.166 |
| 65.01 | 118.11 | 5.05 | 0.208 |
| 85.01 | 155.86 | 7.55 | 0.639 |

These are local frozen-weight Gauss–Newton approximations. The full 24D query
retains cross-time correlation (maximum 0.968), current-root uncertainty and the
unchanged 32 MiB query limit. The [unknown-bias record](../native-joint-query-20261101/README.md)
is retained. Assessment against simulator truth changes position RMSE from
2.67 to 2.72 cm and orientation RMSE from 0.137° to 0.101°; the initial gauge is
the only trajectory alignment. No map or readiness is accepted.

Using the same assessment-only complete scene at the new native pose means
still gives whole-image power residual RMS 20.09, 2.21, 42.05 and 74.30. Exact
renderer poses give about one. A mean pose cannot replace the shared uncertain
navigation state when inferring acoustic geometry.

Run `PYTHONPATH=src python docs/diagnostics/native-initial-bias-20261101/verify.py`
from POSIM-SLAM. The verifier checks the SDF certificate, identical remaining
configuration/clocks, native acknowledgement, the recorded joint posterior and
271 assessment rows. It recomputes the 24D marginal without SDK/application
imports. It does not restart the native worker or recompute acoustic images.
The bounded snapshot and derived records total about 1.7 MB; raw recordings,
SDKs and private WWW-POSIM implementation are not copied.

## 초기 IMU 바이어스 보정 대조

같은 소나 원본 183장, 원시 IMU, 해저 추적이 유효한 DVL 90개와 수압 제약
90개를 사용합니다. 카메라·Ping·음향 제약·장면 정답은 항법에 넣지 않습니다.
271개 상태의 시각, 네이티브 실행 파일, 원본 전처리 결과와 잡음은 같습니다.

기록된 SDF의 무잡음 IMU 인증값을 `InitialImuBiasCalibration`으로 전달합니다.
영 바이어스 평균·공분산은 **첫 상태에만** 적용하며 이후 확률적 변화는
유지합니다. ROS 공분산 0이나 작은 움직임을 보정 정보로 해석하지 않습니다.
실물이나 잡음이 있는 센서는 외부에서 보정한 평균과 전체 공분산이 필요합니다.

위 표의 값은 국소 Gauss–Newton 근사입니다. 시점 간 상관관계와 현재 상태의
불확실성을 보존하며 복원 예산은 32 MiB입니다. 위치 RMSE는 2.67→2.72 cm,
자세 RMSE는 0.137→0.101°입니다. 지도와 준비 완료는 인정하지 않습니다.
같은 전체 장면을 평가용으로만 주어도 평균 자세에서 음향 잔차가 남으므로,
음향 형상 추정에는 시점 간 상관관계를 포함한 자세 불확실성이 필요합니다.

위 명령은 공개 라이브러리와 NumPy로 보정 인증, 설정·시각, 네이티브 응답과
24차원 공분산을 다시 검사합니다. 네이티브 실행이나 음향 영상을 재계산하는
검사는 아닙니다. 원본·SDK·WWW-POSIM 비공개 코드는 포함하지 않습니다.
