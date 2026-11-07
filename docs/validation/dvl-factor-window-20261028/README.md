# DVL factor lifetime and recovery evidence

This dated engineering record retains a computation control and seven source-bound ROS cases. It supplements the [native VIO lifecycle record](../native-vio-lifecycle-20261028/README.md), which retains the failed sixteen-state DVL trial. It does not qualify installed packages or general visual initialization uncertainty.

The application requests three recent full states for visual/DVL operation, using the native default. A DVL velocity factor observes one time and retires into the shared marginal prior. The extended sixteen-state window remains specific to acoustic multiview tracks. The native binary, input rates, sensor noise, admission limits, event-queue bounds and navigation timeout remain unchanged.

## Native computation control

The same measured 25-second input, submitted seed and 287 camera/IMU/DVL/pressure events run with retained windows of 16, 3, 3 and 16 states. Elapsed times are 29.13, 6.38, 6.33 and 29.48 seconds. Each case includes its complete native map, calibration, event stamps and latency observations. Window length changes estimates and map size; these timings do not establish accuracy equivalence. The control begins from a frozen measured seed and does not replay the preceding optical warmup.

The native camera/DVL retirement check covers three and sixteen states at 32×32 and 320×240 pixels. All four subcases pass. This synthetic constant-velocity check verifies factor lifetime and retirement, not field geometry.

## Original-time ROS cases

Each case uses the same 120.06-second record at original time. The seven cases cover DVL bottom-lock loss with and without Ping factors, DVL communication loss with and without Ping factors, and pressure loss in Ping-fused, Ping-reception-only and absent-Ping-input profiles. All use pressure depth. All seven cases pass their explicit loss/recovery contracts and stop their workers and status writer.

Bottom-lock loss retains the visual map and frame, rejects export during invalid lock, and resumes valid DVL velocity fusion without a segment reset. Communication loss produces exactly one `dvl_stale` reset, clears the old map and recovers in a new frame. Pressure loss produces exactly one `pressure_stale` reset. Invalid bottom lock and absent communication are separate cases. Reception-only and absent-input profiles optimize zero Ping factors.

The fourteen complete PLY maps contain 994–3,014 vertices each. Every emitted vertex is assessed against the known test surfaces after estimation; no coordinate filter or trimming is applied. Surface RMSE spans 1.14–1.94 cm, maximum distance 12.23–23.82 cm, and 1–7 vertices per map exceed 10 cm. These figures describe this recorded scene, not a general marine SLAM acceptance threshold. `map-summary.json` retains each map's metrics. Each phase includes an RGB/cloud isometric comparison with 64 pt² markers and a shared −4 to −1.5 m Turbo height scale. Surface associations are geometric evaluation labels, not semantic recognition.

## Verify retained artifacts

Run `python3 summarize.py` from this directory. The standard-library verifier checks hashes, control settings, recovery semantics, worker cleanup, all PLY vertices, all per-point assessment indices, and comparison settings. It verifies retained evidence; it does not rerun ROS or the simulator. Large raw sensor records and the private application probe stay outside this kit. The public kit contains no private application source.

## 한국어

이 기록은 계산량 비교 실험과 소스가 고정된 ROS 사례 일곱 개를 보존합니다. 실패한 16개 상태 DVL 사례는 [네이티브 VIO 수명주기 기록](../native-vio-lifecycle-20261028/README.md)에 있습니다. 설치형 패키지와 일반적인 시각 초기화 불확실성의 검증은 별도로 남습니다.

시각/DVL 조합은 네이티브 기본값인 최근 전체 상태 3개를 사용합니다. DVL 속도 제약은 한 시각을 관측하고 공통 주변화 사전정보로 이전됩니다. 16개 상태 창은 여러 시점의 음향 특징점에 사용합니다. 네이티브 실행 파일, 입력 주기, 센서 잡음, 지도점 채택 기준, 이벤트 큐 한도와 항법 제한 시간은 같습니다.

동일한 실측 시드와 25초 입력 287개 이벤트에서 상태 창을 16/3/3/16개로 바꾸면 처리 시간은 29.13/6.38/6.33/29.48초입니다. 각 실험의 전체 지도, 보정값, 이벤트 시각과 지연 기록을 보존합니다. 상태 창은 추정값과 점 수에도 영향을 주므로 이 결과는 정확도 동등성을 입증하지 않습니다. 시드 이전의 광학 준비 구간은 재생하지 않습니다. 네이티브 제약 수명 검사는 상태 3/16개와 영상 32×32/320×240의 네 조합을 통과합니다.

동일한 120.06초 기록을 원래 속도로 재생하여 DVL 바닥 추적 상실 두 사례, DVL 통신 단절 두 사례, 수압 입력 단절 세 사례를 검사합니다. Ping 거리 융합·수신만·입력 없음의 구분도 확인합니다. 바닥 추적 상실은 지도와 좌표계를 유지하고 유효한 속도 융합으로 복귀합니다. 통신 단절은 `dvl_stale`, 수압 단절은 `pressure_stale`로 각각 한 번 구간을 초기화하고 새 좌표계에서 복귀합니다. 모든 작업자와 상태 기록기가 종료됩니다.

전체 PLY 14개는 각각 994–3,014점입니다. 모든 출력점을 추정 완료 후 알려진 시험 표면과 비교하며, 좌표 필터나 이상점 제거를 적용하지 않습니다. 표면 RMSE는 1.14–1.94 cm, 최대 거리는 12.23–23.82 cm이고 지도마다 1–7점이 10 cm를 넘습니다. 이 값은 해당 기록의 측정 결과이며 해양 SLAM 전반의 합격 기준이 아닙니다. 각 단계의 등각 RGB/점군 비교는 64 pt² 점과 동일한 −4~−1.5 m Turbo 높이 범위를 사용합니다. 표면 이름은 사후 기하 비교용 분류입니다.

`python3 summarize.py`는 해시, 상태 창, 복귀 동작, 작업자 종료, 전체 PLY와 점별 평가 및 비교 그림 설정을 확인합니다. 시뮬레이터 재실행과 구분되는 보존 자료 검사입니다. 대용량 센서 원본과 비공개 앱 검사 코드는 포함하지 않습니다.
