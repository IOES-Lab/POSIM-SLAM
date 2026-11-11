# Native VIO duration and mode lifecycle

These cases exercise the development native simulator with the same camera/raw-IMU, pressure and single-Ping source profile as the [startup-scoring control](../seed-scoring-20261028/README.md). [Source identity](source-identity.json) records component/application revisions, provider hashes, engine revision, assumed noise and the unchanged one-second vehicle-navigation freshness limit. SDK/visual shader overrides remain development artifacts; installed-package qualification is separate.

## Continuous operation

[The trial](nav-06/trial-result.json) completes 21.32 minutes after visual/IMU initialization and eleven checked physical surveys. Each survey covers a 7 × 4 m strip at 0.6 and 1.2 m depth, twice, with fixed heading. Twelve complete PLY exports remain. The [last export](nav-06/mission-11-map.ply) has 6,312 points. There is no operating SLAM reset, and the maximum serialized owner tick is 356 ms.

[The final status](nav-06/final-status.json.gz), all eleven mission records and [full observation timeline](nav-06/observations.jsonl.gz) preserve the boundaries before explicit shutdown. [Worker cleanup](nav-06/worker-cleanup.json), [normal process exit](nav-06/lifecycle.json) and [owned-group inventory](nav-06/owned-process-check.json) show that the owned services stop.

A separately scheduled, read-only ROS probe records raw FCU position messages and MAVROS local pose. Its bounded writer retains all 70,213 rows with no fault or pending data. The [direct receipt timeline](nav-06/direct-receipts.csv.gz), [application timeline](nav-06/api-navigation.jsonl.gz) and [analysis](nav-06/navigation-analysis.json) distinguish upstream message delivery from application receipt. Maximum direct local-pose gap is 211 ms; maximum application pose age is 124 ms. Comparing identical pose headers gives a maximum inferred application-minus-probe receipt delay of 17 ms; HTTP processing can contribute to that estimate. A 200 ms diagnostic marker does not change the controller timeout.

The earlier [navigation-loss trial](../seed-scoring-20261028/cold-05-long/trial-result.json) remains a failure: three surveys complete, then the fourth stops at `navigation_lost`. Its last complete export has 5,426 points. A later nominal run passing does not establish the cause of that failure or demonstrate its repair. Both outcomes remain available.

## Sensor-combination round trips

[The mode trial](modes-07/trial-result.json) completes all three round trips and seven physical surveys in 14.04 minutes. Camera, raw IMU and pressure remain selected throughout. Every transition clears the previous map, creates a distinct segment/frame and rejects immediate export with HTTP 409. Real physical motion produces a fresh seed in 4.66–4.88 seconds after each switch. All seven full exports remain, with no unexpected SLAM reset. These switches use the actual application control API; repeat GUI and installed-package checks remain separate.

| Segment | Ping use | Full exported points | Optimized Ping / pressure states |
| --- | --- | ---: | ---: |
| [0](modes-07/phase-00/result.json) | Fused | 2,343 | 89 / 121 |
| [1](modes-07/phase-01/result.json) | Received only | 2,867 | 0 / 111 |
| [2](modes-07/phase-02/result.json) | Fused | 2,207 | 96 / 112 |
| [3](modes-07/phase-03/result.json) | Received only | 2,999 | 0 / 110 |
| [4](modes-07/phase-04/result.json) | Fused | 2,167 | 87 / 111 |
| [5](modes-07/phase-05/result.json) | Received only | 2,908 | 0 / 112 |
| [6](modes-07/phase-06/result.json) | Fused | 2,420 | 96 / 111 |

Reception-only segments receive more than 1,100 Ping samples each and optimize zero Ping states. The [independent probe](modes-07/probe-result.json) retains all 45,006 receipt rows without fault. Its maximum local-pose gap is 202 ms. Maximum application pose age is 815 ms, below the unchanged one-second control deadline. The [analysis](modes-07/navigation-analysis.json) preserves both excursions over the 200 ms diagnostic marker; neither caused a navigation failure. The marker is not an acceptance deadline.

All seven native workers stop and all owned process groups are empty after normal shutdown. Run `python3 docs/validation/native-vio-lifecycle-20261028/summarize.py` from either repository root to verify hashes, every PLY vertex, distinct frames, switch gates, mission results and cleanup using only the standard library.

## DVL case stopped before expansion

The next [original-time DVL/pressure replay](dvl-first-failure/receipt.json.gz) fails before the requested bottom-lock-loss interval. It receives and optimizes valid DVL velocities, but accumulates three `sensor_events_backlog` resets and produces no initial full-map export. The planned second DVL/Ping combination is not run. All four workers and the diagnostic writer stop. This is a retained failure, not bottom-lock recovery acceptance.

The [complete status timeline](dvl-first-failure/statuses.json.gz) and [queue summary](dvl-first-failure/queue-summary.json) show the ordered event queue reaching its unchanged 34-event limit while received-input retention remains small and status writes take at most 7.8 ms. The DVL calibration selects sixteen recent full states; the corresponding camera/IMU/pressure profile uses the native default of three. DVL contributes a velocity factor at one measurement time, whereas acoustic tracks can span several states. Next diagnosis must measure which retention and computation each factor actually requires. No queue, freshness, noise or map-admission limit is changed to accept this result.

## Scope

These are lifecycle and nominal-duration checks. No per-point geometric accuracy assessment is claimed for these new field exports. The source-bound replay's [six assessed maps and matching isometric RGB/cloud overviews](../seed-scoring-20261028/README.md) cover the selected input combinations separately. Keep all emitted vertices; no truth data enters estimation. Initialization uncertainty, physical payload removal, DVL loss/recovery, low-visibility sonar and installed packages remain open gates in the application completion plan. The earlier long-running GUI ingress-backlog observation also remains preserved.

시각 카메라·원시 IMU·수압·단일 Ping 조합으로 초기화 후 21.32분 동안 실제 항해 11회를 마쳤습니다. 전체 지도 12개를 보존하며 마지막 지도는 6,312점입니다. SLAM 재시작은 없고 종료 후 소유 프로세스도 남지 않았습니다. 독립 ROS 수신 기록과 앱 기록을 함께 보존합니다. 이전 항법 유실의 원인이나 해결을 입증한 결과는 아니며, 해당 실패 기록도 유지합니다. Ping 융합을 끄고 켜는 3회 왕복에서도 이전 지도 저장을 차단하고 서로 다른 7개 구간의 전체 지도를 보존했습니다. 전환 후 초기화는 실제 이동 중 4.66~4.88초가 걸렸으며, 수신만 하는 구간에서는 Ping이 반영된 상태가 0개입니다. 모든 워커와 소유 프로세스는 정상 종료됐습니다. 이어진 DVL 시험은 해저 추적 유실을 넣기 전 이벤트 큐 재시작 3회로 실패해 다음 조합을 실행하지 않았습니다. 수신·저장 지연과 실제 최적화 연산량을 구분해 원인을 확인합니다. 새 항해 지도의 정확도와 초기화 불확실성, 반복 GUI 전환 및 설치형 앱 검증은 별도입니다.

[Repository documentation](../../../README.md)
