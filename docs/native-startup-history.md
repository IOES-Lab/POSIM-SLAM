# Retained startup measurements

Set `retain_startup_measurements: true` with `require_gravity_seed: true` in a
camera/IMU worker calibration to retain original startup frontend results.
The worker acknowledges `original_frontend_once_v1` in
`retained_startup_transport`. The default is disabled. Camera-free sonar and
camera/sonar-owner profiles refuse the request.

Send startup images with `LiveClient.startup_image` and wait for each original
`startup_tracks` reply. The worker owns those same frontend results until their
chronological handoff. The existing bounds are 32 frames, 10 seconds and at
most 0.5 seconds between frames. Results are neither regenerated nor evicted.

After computing a numerical seed, express it at the first retained frame and
send `LiveClient.inertial_seed` with that timestamp. A seed computed later may
be transported to the retained start using covered original IMU measurements;
processing waits until those measurements and the seed are available. Keep
position/yaw gauge choices, extrinsics and declared IMU bias/noise conventions
explicit. The seed is an optimizer initial value, not another likelihood row.

Release the original raw IMU through each state event, including its first
strictly later sample, and call `LiveClient.retained_image(stamp_ns)` once per
retained timestamp. The first event uses the normal 200 ms IMU bootstrap.
Optional `ping=(stamp_ns, range_m)` uses the same exact-time validation as a
fresh image. Submit DVL and pressure through their normal ordered transports.
Preserve every original measurement once, including overlapping bootstrap IMU.

These events use the existing VIO optimizer, bias states, priors, noise and
marginalization. They do not run optical tracking a second time or enlarge the
recent-state window. Fresh images resume after all retained results are handed
over. Missing, repeated, skipped or reordered results are refused. Keep the
host in initialization while catching up; sensor timestamps and freshness
requirements remain unchanged. This transport does not qualify initialization
uncertainty or navigation readiness.

Use `client.set_output_notification(callback)` to wake an event loop when a
validated worker output is queued. The callback consumes no packet and must
return immediately; call `poll()` from the owning loop. Notifications can
coalesce and may run on the reader or installing thread. Passing `None`
removes the callback. `close()` joins the reader before returning. Retain an
independent timer for input freshness and worker deadlines when no reply arrives.

## 초기 측정 보존

카메라·IMU 워커 보정에서 `require_gravity_seed: true`와 함께
`retain_startup_measurements: true`를 설정하면 원본 초기 특징 추적 결과를
보존합니다. 워커는 `retained_startup_transport`에서
`original_frontend_once_v1`을 확인합니다. 기본값은 꺼짐이며 카메라 없는
소나와 카메라·소나 소유자 구성은 이 요청을 거부합니다.

`LiveClient.startup_image`로 초기 영상을 보내고 각 `startup_tracks` 응답을
기다립니다. 워커는 원본 추적 결과를 순서대로 넘길 때까지 소유합니다.
기존 한도인 32프레임·10초·프레임 간 최대 0.5초를 유지하며 결과를 다시
만들거나 자동으로 버리지 않습니다.

수치 초기값을 구한 뒤 첫 보존 프레임의 좌표계·시각으로 표현하고 그
시각의 `LiveClient.inertial_seed`를 보냅니다. 나중 시각의 초기값은 실제로
덮인 원본 IMU 구간으로 첫 시각까지 옮길 수 있습니다. 필요한 측정과
초기값이 도착한 뒤 처리합니다. 위치·방위각의 기준, 장착 보정과 선언한
IMU 바이어스·잡음 좌표 규약을 명시합니다. 초기값은 최적화 시작값이며
측정 가능도 행으로 다시 더하지 않습니다.

각 상태 시각과 그 직후의 첫 표본까지 원본 IMU를 보내고, 보존한 시각마다
`LiveClient.retained_image(stamp_ns)`를 한 번씩 호출합니다. 첫 이벤트는
기존 200ms IMU 시작 이력을 사용합니다. 선택형 `ping=(stamp_ns, range_m)`은
새 영상과 같은 정확한 시각 검사를 사용합니다. DVL·수압도 기존 순서
전송을 사용합니다. 시작 구간이 겹치는 IMU도 원본 측정마다 한 번만 보냅니다.

기존 VIO 최적화·바이어스 상태·사전분포·잡음·주변화를 사용하며 특징을
다시 추적하거나 최근 상태 창을 늘리지 않습니다. 보존 결과를 모두 넘긴
뒤 새 영상을 처리합니다. 결과 누락·중복·생략·순서 오류는 거부합니다.
따라잡는 동안 호스트는 초기화 상태를 유지하며 원본 센서 시각과 최신성
조건을 지킵니다. 이 전송은 초기화 불확실성이나 항해 준비의 합격 판정이
아닙니다.

`client.set_output_notification(callback)`은 검증한 워커 출력이 큐에 들어오면
호스트 이벤트 루프를 깨웁니다. 콜백은 패킷을 꺼내지 않고 즉시 반환해야
하며, 소유자 루프에서 `poll()`을 호출합니다. 알림은 합쳐질 수 있고 읽기
스레드나 콜백 등록 스레드에서 실행될 수 있습니다. `None`으로 알림을
해제합니다. `close()`는 읽기 스레드가 끝난 뒤 반환합니다. 응답이 없을 때도
입력 최신성과 워커 기한을 검사하는 독립 타이머를 유지합니다.

## Managed current-field example / 조류 현장 예제

[English measured example](diagnostics/managed-current-20261108/README.md) ·
[한국어 실측 예제](diagnostics/managed-current-20261108/README.ko.md).

The example retains all maps and the 1.536 m outlier; physical lock loss and
installed use remain unverified.
예제는 모든 지도와 1.536m 이상점을 보존하며 물리적 bottom-lock 상실과
설치형 운용은 아직 검증하지 않았습니다.

[Physical DVL loss, complete returned maps and failed capacity/export gates](diagnostics/physical-dvl-range-20261108/README.md) ·
[실제 DVL 상실·전체 복귀 지도·용량/내보내기 실패](diagnostics/physical-dvl-range-20261108/README.ko.md).
