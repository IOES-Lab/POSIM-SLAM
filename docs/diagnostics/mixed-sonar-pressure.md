# Mixed visual/sonar pressure input

The experimental ROS owner combines camera, raw IMU, single Ping, imaging sonar and hydrostatic pressure depth. Pressure factors use calibrated variance, sensor mounting offset and one shared unknown height datum. Measurements retain their original timestamps. Validated arrivals advance input ordering independently of the 1 Hz pressure factor limit. Both map outputs and exports require recent acknowledged pressure optimization on both sensor and wall clocks.

![Visual map and matched RGB](../images/mixed-pressure-visual.png)

![Acoustic hypotheses and matched RGB](../images/mixed-pressure-sonar.png)

The recorded ROS/native example exports 1,055 visual points and 61 acoustic hypotheses at a 5 Hz camera cap and 1x playback. Initial yaw/translation alignment keeps scale one. Position RMSE is 0.026 m over the 29.6-second comparison segment. Only 33 acoustic hypotheses lie within 10 cm of a physical surface; surface-distance p90 is 0.252 m and maximum is 0.671 m. The sparse acoustic map does not reconstruct the terrain and remains unqualified.

Each pair uses the same isometric perspective, fixed Turbo height limits and 4× marker diameter. Field RGB includes underwater attenuation, suspended particles, seabed caustics and sea surface. This RGB overview and simulator geometry are evaluation-only. Estimation uses the recorded onboard camera, raw IMU, Ping, sonar and pressure observations.

## Integration and processing limits

The 1x sonar interruption/recovery check publishes 208 poses. Loss clears both maps and rejects both exports; a fresh segment recovers 448 visual points, seven acoustic hypotheses and 11 optimized pressure states. The shorter 1x pressure interruption check also passes. Camera/IMU/Ping at a 10 Hz camera cap passes at 0.6x playback with 338 poses and 1,357 exported points. These recorded-input ROS/native checks do not qualify a new Gazebo mission or an installed application.

A continuous 1x mixed playback trial fills the 16-event queue after approximately 40 seconds, before the planned sensor interruption. A trace-enabled repeat reaches the planned interruption without that overflow but rejects the recovered sonar export because no map is available. Plain camera/IMU/Ping at 1x also encounters worker input backlog. The short recovery checks reset estimator state during the run and cannot establish sustained real-time performance. Default panel activation remains gated on processing and map quality.

Shared nuisance elimination uses thin SVD for the same rank test and minimum-norm back-substitution, followed by implicit Householder transforms of the retained Jacobian and residual. This preserves reduced information, residual cost and state correlations without allocating the full left singular basis. Independent NumPy tests cover tall, wide, rank-deficient, zero and damped systems. Four fixed synthetic systems measure 1.02–13.12× kernel speedup; this is not an end-to-end speedup. Exact camera/Ping pairing also releases a missing match once a strictly later Ping timestamp proves it cannot arrive, retaining the existing queue and timeout limits.

The earlier 2.7-second `processing_ms` observation measures owner submission to paired pose/map acknowledgement, including scheduling and queues. It is not an isolated native solve time. Native profiling identifies full-basis elimination as a cost, but the remaining continuous-run bottleneck needs separate measurement. These are standard numerical and stream-ordering improvements, without a research novelty claim.

## 한국어

실험용 ROS 경로는 카메라·원본 IMU·단일 Ping·영상 소나·정수압 수심을 결합합니다. 압력 제약은 보정된 분산, 센서 장착 위치와 공유 미지 높이 기준값을 사용합니다. 원래 측정 시각을 유지하며, 검증된 수신 시각은 1 Hz 제약 선택과 독립적으로 입력 정렬을 갱신합니다. 두 지도 모두 센서·실시간 시계에서 최근 압력의 실제 최적화 반영을 확인해야 표시·내보내기를 허용합니다.

위 비교는 카메라 한도 5 Hz, 1배속 재생에서 얻은 시각 점 1,055개와 소나 가설점 61개입니다. 초기 방위각·위치만 정렬하며 축척은 1입니다. 29.6초 비교 구간의 위치 RMSE는 0.026 m이고 소나 점 33개만 실제 표면 10 cm 이내입니다. 표면 거리 오차 p90은 0.252 m, 최대는 0.671 m여서 지형 복원 합격으로 인정하지 않습니다.

두 비교는 같은 등각 방향 원근 투영, 고정 Turbo 높이 범위와 지름 4배의 점을 사용합니다. RGB에는 수중 감쇠·부유 입자·해저 집광무늬·수면이 포함됩니다. 이 현장 RGB와 지형은 평가용이고, 추정 입력은 기록된 차량 카메라·IMU·Ping·소나·수압입니다.

1배속 소나 중단·복구 시험은 자세 208개를 발행합니다. 중단 시 두 지도를 비우고 내보내기를 거부하며, 새 구간에서 시각 점 448개·소나 가설점 7개·압력 최적화 상태 11개로 복구합니다. 짧은 1배속 압력 중단 시험도 통과합니다. 카메라 한도 10 Hz의 카메라·IMU·Ping 대조 실행은 0.6배속에서 자세 338개와 점 1,357개로 통과합니다. 기록 기반 ROS/네이티브 연동 결과이며 새 Gazebo 주행이나 설치형 앱 합격은 아닙니다.

혼합 센서의 연속 1배속 시험에서는 약 40초 뒤, 예정된 센서 중단 전에 16개 입력 큐가 가득 찹니다. 진단 로그를 켠 반복 실행은 큐 초과 없이 예정된 중단까지 진행했지만 복구 후 소나 지도가 없어 내보내기를 거부합니다. 기존 카메라·IMU·Ping의 1배속 실행도 작업자 입력 적체가 발생합니다. 짧은 복구 시험은 중간에 추정 상태를 초기화하므로 지속적인 실시간 성능을 증명하지 못합니다. 패널 기본 활성화에는 처리 성능과 지도 품질 검증이 더 필요합니다.

공통 보조 변수 제거는 thin SVD로 같은 랭크 판정과 최소 노름 역대입을 수행하고, Householder 변환을 암시적으로 적용합니다. 전체 좌특이벡터 행렬을 만들지 않으면서 축약 정보·잔차 비용·상태 간 상관을 유지합니다. 독립 NumPy 검증은 긴 행렬·넓은 행렬·랭크 부족·영행렬·감쇠 조건을 포함합니다. 고정 합성 행렬 네 개의 해당 계산은 1.02~13.12배 빨라지지만 전체 실행 속도 개선율은 아닙니다. 카메라/Ping의 정확한 시각 연결은 더 늦은 Ping 수신으로 누락이 확정된 영상을 즉시 해제합니다. 큐와 대기 한도는 그대로입니다.

앞선 2.7초 `processing_ms`는 입력 전달부터 자세·지도 응답까지의 시간으로 스케줄링과 큐 대기를 포함합니다. 순수 네이티브 최적화 시간은 아닙니다. 프로파일에서 전체 기저 제거 계산의 비용을 확인했지만 연속 실행의 남은 병목은 별도 측정이 필요합니다. 이번 수정은 알려진 수치 계산·입력 정렬 기법이며 연구 신규성을 주장하지 않습니다.
