# Recorded-map accuracy review / 기록 지도 정확도 재평가

## English

The 6,213-point camera/raw-IMU/single-Ping map is acceptable for the geometry of this recorded demo. Three points beyond 10 cm do not by themselves fail the complete map. This revises the engineering judgment in the [earlier comparison](../orb-redetection-control-20261028/README.md); that dated report and every original vertex remain unchanged. Initialization, live loss/recovery, source-bound runtime profiles and other sensor combinations need their own evidence.

| Complete-map metric | Measured value |
| --- | ---: |
| Surface-distance RMSE | 9.50 mm |
| Median | 5.57 mm |
| 95th percentile | 1.55 cm |
| 99th percentile | 2.78 cm |
| 99.9th percentile | 6.79 cm |
| Beyond 10 cm | 3 / 6,213 = 0.0483% |
| Within 10 cm | 99.9517% |
| Maximum | 16.83 cm |
| Trajectory position RMSE | 2.42 cm |

These are post-estimation nearest distances to static physical triangles, after initial yaw and translation alignment at scale 1. They are not ground-truth physical feature identities, signed vertical depth errors or a hydrographic uncertainty certificate. The map has nearby points for all three test objects and both walls. Such object support is useful alongside the errors, but is not an area-completeness measurement. A lower maximum from the 183-point candidate does not compensate for its missing objects.

Marine SLAM evaluation uses trajectory metrics such as absolute trajectory error, relative pose error and tracking/initialization behavior. For example, [Suresh et al., through-water stereo SLAM](https://www.cs.cmu.edu/~kaess/pub/Suresh19ral.pdf) compares ATE and RPE, while [YOLO-NeRFSLAM, evaluation metrics](https://www.frontiersin.org/journals/marine-science/articles/10.3389/fmars.2025.1582126/full) reports ATE, RPE and initialization efficiency. These are evaluation approaches, not a universal centimetre-level pass/fail standard.

[IHO S-44 Edition 6.2.0](https://iho.int/uploads/user/pubs/standards/s-44/S-44_Edition_6.2.0_adopted.pdf), sections 2.6–2.7 and chapter 7, specifies purpose-dependent horizontal/vertical uncertainty at a 95% confidence level, feature detection and coverage for hydrographic surveys. It does not establish a point-cloud rule that every vertex must be within 10 cm. Its uncertainty requirements cannot be certified from this nearest-surface statistic or from an outlier percentage alone.

For this demo, report RMSE, quantiles, the error tail, spatial distribution and scene support together. Keep full points and disclose the remaining large errors. Do not keep modifying a working mapper solely to obtain zero threshold exceedances. The estimator's 0.1 m predicted three-sigma admission budget remains unchanged: model uncertainty is a different quantity from an empirical all-vertices maximum. This review adds no universal numerical acceptance threshold and is a retrospective assessment of this record, not a held-out validation result.

`python summarize.py` reproduces [metrics.json](metrics.json) from the public complete-map assessment. Input hashes bind the assessment, PLY and trajectory comparison. No new sensor recording, SDK copy, feature gate, covariance change or point deletion is introduced.

## 한국어

카메라·원시 IMU·단일 Ping의 6,213점 지도는 **이 기록의 데모 기하 품질을 수용**합니다. 3점이 10cm를 넘었다는 이유만으로 전체 지도를 불합격 처리하지 않습니다. 위 이전 비교의 공학적 판단을 정정하며 당시 보고서와 모든 원본 점은 그대로 보존합니다. 초기화·실제 손실과 복구·소스에 결합된 운영 프로필·다른 센서 조합은 각자 검증해야 합니다.

표면 거리 RMSE는 9.50mm, 중앙값 5.57mm, 95백분위 1.55cm, 99백분위 2.78cm입니다. 99.9517%가 10cm 이내이고 최대 거리는 16.83cm입니다. 궤적 위치 RMSE는 2.42cm입니다. 첫 yaw·평행이동만 정렬하고 배율 1을 유지하여 정적인 물리 삼각형까지의 최근접 거리를 추정 뒤 계산했습니다. 실제 특징점 ID·수직 수심 오차·수로측량 불확실성 인증과는 다른 값입니다.

세 시험 물체와 두 벽 근처에 모두 지도 점이 있습니다. 이는 오차와 함께 볼 유용한 근거지만 면적 재구성을 측정한 값은 아닙니다. 183점 후보에서 최대 오차가 작아도 빠진 물체들을 대신하지는 못합니다.

해양 SLAM 연구는 보통 절대 궤적 오차·상대 자세 오차와 추적·초기화 동작을 함께 평가합니다. 위 원 논문들은 이러한 지표를 사용하는 예이며, 모든 시스템에 적용할 cm 단위의 합격선은 아닙니다. IHO S-44는 수로측량 용도에 따른 95% 신뢰수준의 수평·수직 불확실성, 물체 탐지와 측정 범위를 규정합니다. 모든 점이 10cm 이내여야 한다는 점군 규칙이 아니며, 최근접 표면 거리나 이상점 비율만으로 이 표준의 적합성을 판정할 수 없습니다.

데모에서는 RMSE·백분위·큰 오차의 비율과 위치·장면 지지를 함께 보고합니다. 전체 점과 남은 오차를 보존하며 임계값 초과를 0개로 만드는 것만을 목표로 추적기를 계속 변경하지 않습니다. 추정기의 예측 3시그마 채택 한도 0.1m는 유지합니다. 모델 불확실성과 모든 점의 실제 최대 오차는 서로 다른 양입니다. 이번 판단은 이 기록에 대한 사후 검토이며 새 보편 수치 기준이나 독립 검증 결과를 주장하지 않습니다.

위 명령은 공개된 전체 점 평가에서 수치를 재현하고 입력 해시를 기록합니다. 새 원본 기록·SDK 복제·특징 필터·공분산 변경·점 삭제는 없습니다.
