# Mixed visual/sonar pressure input

The experimental ROS owner combines camera, raw IMU, single Ping, imaging sonar and hydrostatic pressure depth. Pressure factors use calibrated variance, sensor mounting offset and one shared unknown height datum. Measurements retain their original timestamps. Validated arrivals advance input ordering independently of the 1 Hz pressure factor limit. Both map outputs and exports require recent acknowledged pressure optimization on both sensor and wall clocks.

![Visual map and matched RGB](../images/mixed-pressure-visual.png)

![Acoustic hypotheses and matched RGB](../images/mixed-pressure-sonar.png)

The recorded ROS/native example exports 1,063 visual points and 67 acoustic hypotheses at a 5 Hz camera cap and 1x playback. Initial yaw/translation alignment keeps scale one. Position RMSE is 0.027 m over the 30.2-second comparison segment. Only 40 acoustic hypotheses lie within 10 cm of a physical surface; surface-distance p90 is 0.313 m and maximum is 0.664 m. The sparse acoustic map does not reconstruct the terrain and remains unqualified.

Each pair uses the same isometric perspective, fixed Turbo height limits and 4× marker diameter. Field RGB includes underwater attenuation, suspended particles, seabed caustics and sea surface. This RGB overview and simulator geometry are evaluation-only. Estimation uses the recorded onboard camera, raw IMU, Ping, sonar and pressure observations.

## Integration and processing limits

The 1x sonar interruption/recovery check publishes 209 poses. Loss clears both maps and rejects both exports; a fresh segment recovers 458 visual points, seven acoustic hypotheses and 11 optimized pressure states. Camera/IMU/Ping at a 10 Hz camera cap passes a 35-second input replay at 1x, with 337 poses and 1,415 exported points. These bounded recorded-input checks do not qualify sustained hardware deadlines or an installed application.

Two longer mixed-input trials reach the planned pressure interruption without input overflow. Each exports 1,963 visual points and 73 acoustic hypotheses before loss, but neither produces an acoustic map after recovery. The recovery interval is near the surface after the mission: independent truth spans only 3.4 cm and 4.9 cm horizontally, and 11.5 cm vertically. All 336 submitted acoustic track updates are rejected: 169 for insufficient elevation precision, 40 for ambiguous elevation and 127 for inconsistent or unobservable geometry. These counts describe repeated updates, not unique landmarks. This supports insufficient useful viewpoint diversity; it does not rule out association or pose errors. A new sustained maneuver after recovery is needed. Geometry thresholds and empty-map export rejection remain enforced.

Shared nuisance elimination uses thin SVD with the original rank test and minimum-norm back-substitution, then implicit Householder transforms. Each optimizer trial reduces the raw group once with its damping. The no-argument marginalization path remains undamped. Independent numerical tests compare reduced information, gradient, residual cost and back-substitution; dense Schur checks cover coupled landmarks and planes at three damping values. The thin-SVD kernel's four synthetic timings are 1.02–13.12× faster than its full-basis predecessor; they do not measure the whole runtime or the removal of duplicate reduction.

Exact camera/Ping pairing releases a missing match when a strictly later Ping proves it cannot arrive. Queue, timeout, noise and geometry thresholds are unchanged. `processing_ms` measures owner submission to paired pose/map acknowledgement, including scheduling and queues. The short mixed trial's maximum is 879 ms. It is not isolated native solve time or a worst-case guarantee. Default mixed-mode activation still requires useful map geometry and broader runtime qualification. These are standard numerical and stream-ordering changes, without a research novelty claim.

## 한국어

실험용 ROS 경로는 카메라·원본 IMU·단일 Ping·영상 소나·정수압 수심을 결합합니다. 압력 제약은 보정된 분산, 센서 장착 위치와 공유 미지 높이 기준값을 사용합니다. 원래 측정 시각을 유지하며, 검증된 수신 시각은 1 Hz 제약 선택과 독립적으로 입력 정렬을 갱신합니다. 두 지도 모두 센서·실시간 시계에서 최근 압력의 실제 최적화 반영을 확인해야 표시·내보내기를 허용합니다.

위 비교는 카메라 한도 5 Hz, 1배속 재생에서 얻은 시각 점 1,063개와 소나 가설점 67개입니다. 초기 방위각·위치만 정렬하며 축척은 1입니다. 30.2초 비교 구간의 위치 RMSE는 0.027 m이고 소나 점 40개만 실제 표면 10 cm 이내입니다. 표면 거리 오차 p90은 0.313 m, 최대는 0.664 m여서 지형 복원 합격으로 인정하지 않습니다.

두 비교는 같은 등각 방향 원근 투영, 고정 Turbo 높이 범위와 지름 4배의 점을 사용합니다. RGB에는 수중 감쇠·부유 입자·해저 집광무늬·수면이 포함됩니다. 이 현장 RGB와 지형은 평가용이고, 추정 입력은 기록된 차량 카메라·IMU·Ping·소나·수압입니다.

1배속 소나 중단·복구 시험은 자세 209개를 발행합니다. 중단 시 두 지도를 비우고 내보내기를 거부하며, 새 구간에서 시각 점 458개·소나 가설점 7개·압력 최적화 상태 11개로 복구합니다. 카메라 한도 10 Hz의 카메라·IMU·Ping 대조 시험은 35초 입력을 1배속으로 재생하여 자세 337개와 점 1,415개로 통과합니다. 제한된 기록 기반 검증이며, 지속적인 하드웨어 처리 시간이나 설치형 앱 합격을 뜻하지 않습니다.

더 긴 혼합 입력 시험 두 번은 입력 큐 초과 없이 예정된 압력 중단까지 진행합니다. 각각 중단 전 시각 점 1,963개·소나 가설점 73개를 내보내지만 복구 후 소나 지도는 만들지 못합니다. 복구 구간은 임무가 끝난 뒤 수면 근처이며, 독립 참값의 위치 변화 폭은 수평 3.4 cm·4.9 cm, 수직 11.5 cm뿐입니다. 제출된 소나 트랙 갱신 336건 중 169건은 높이 정밀도 부족, 40건은 높이 해의 모호함, 127건은 기하 불일치 또는 관측 불가능으로 거부됩니다. 고유점 개수가 아닌 반복 갱신 횟수입니다. 유효한 관측 시점 다양성 부족을 뒷받침하지만 대응점이나 자세 오차도 배제할 수 없습니다. 복구 뒤에도 기동이 이어지는 새 시험이 필요합니다. 기하 기준과 빈 지도 내보내기 거부는 유지합니다.

공통 보조 변수 제거는 thin SVD로 같은 랭크 판정과 최소 노름 역대입을 수행하고, Householder 변환을 암시적으로 적용합니다. 최적화 시도마다 감쇠를 적용해 원래 그룹을 한 번만 축약합니다. 인자 없는 지도 주변화 경로에는 감쇠를 넣지 않습니다. 독립 수치 검증은 축약 정보·기울기·잔차 비용·역대입을 비교하며, 세 감쇠값에서 랜드마크·평면 결합을 밀집 Schur 계산과 대조합니다. thin SVD의 합성 행렬 네 개는 전체 기저 방식보다 1.02~13.12배 빠르지만 전체 실행이나 중복 제거의 속도 개선율은 아닙니다.

카메라/Ping의 정확한 시각 연결은 더 늦은 Ping 수신으로 누락이 확정된 영상을 해제합니다. 큐·대기·잡음·기하 기준은 그대로입니다. `processing_ms`는 입력 전달부터 자세·지도 응답까지로 스케줄링과 큐 대기를 포함합니다. 짧은 혼합 시험의 최댓값은 879 ms이며 순수 최적화 시간이나 최악 조건 보장은 아닙니다. 혼합 모드 기본 활성화에는 유용한 지도와 더 넓은 실행 검증이 필요합니다. 알려진 수치 계산·입력 정렬 기법이며 연구 신규성을 주장하지 않습니다.
