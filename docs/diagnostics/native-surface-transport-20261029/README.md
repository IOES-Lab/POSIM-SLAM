# Native snapshot-bound delayed likelihood transport

The optional camera-free native worker now receives delayed likelihood
increments on the estimator's own queue. `native_surface.prepare_native_ranges`
returns only new live rows, changed conditionals and original reading keys.
It does not require a fabricated whole-system verification prior after the
native state changes. The older complete-prior reference remains available.

The estimator transfers its legacy state/pressure marginal information once;
active IMU, DVL and pressure readings keep their original owners. An accepted
increment installs the marginal rows, complete changed-conditional closure,
unknown planes and original-reading ledger together. Optimization uses the
existing trial/rollback path. A changed snapshot, partial closure, reused key
or expired source is rejected. Explicit expiry preserves the required
conditional dependency closure and permanently closes old reading keys.

This is disabled by default. Original echo identity, supported surface
association, calibrated joint uncertainty and useful acoustic maps remain
unqualified. Synthetic rows below test transport and ownership. The original
recorded-input control adds no acoustic likelihood or synthetic measurement.

## API and resource ownership

An independent user configures `enable_surface_owner=true`,
`surface_owner_epoch`, the camera-free sonar clock, pressure and optionally
DVL. Single Ping, independent sonar points, legacy conditional capture and
visual factors are excluded from this optional owner mode.

Use `LiveClient.surface_snapshot(request_id)`, poll its accepted
`surface_control` reply, then call `read_surface_snapshot(reply)`.
`prepare_native_ranges(snapshot, ...)` accepts jointly whitened original
range rows in the owner's fixed coordinates. `surface_update(request_id,
increment)` submits the resulting pure increment. Request IDs increase;
a state or optimizer advance requires a new snapshot. The native owner
independently checks its basis and revisions before committing. The caller
retains responsibility for valid physical associations and noise.

`surface_expire(request_id, cutoff_ns, protect=...)` retains protected poses
and the complete conditional closure while closing range keys through that
cutoff. `surface_release(request_id)` releases the pending snapshot.

Bulk matrices use a private per-worker directory beneath its calibration
directory. Only bounded metadata enters the sensor queue; the existing
256 KiB output-line limit and 4 MiB pending-input budget are unchanged.
Files have bounded size, owned names/directories and identity checks;
pre-existing files are preserved. The client removes only its directory
after the worker and both transport threads stop. Acknowledgments distinguish
successful installation from a subsequent optimization exception.

## Evidence

The entire Basalt core, OpenGV and streaming worker were rebuilt with the
new control-marker ABI. [Source/build identities](source-manifest.json),
[commands](build-commands.json) and complete build logs are retained.
No installed SDK, provider receipt, LIVE process or Actions runner changed.
This build is not an installed-provider or relocation qualification.

- Ten compiled native marginal-owner controls pass.
- Sixteen increment/reference controls pass.
- Nineteen native streaming controls pass across five separate logs,
  including five new surface-transport cases, legacy sonar clock, retirement,
  DVL and pressure tests. Native rejection of incomplete closure and reused
  readings is exercised without relying on Python's validation.
- Frozen independent source and an outside-installed wheel each run 494
  tests: 478 pass and sixteen optional legacy native/decoder checks skip.
- The initial source-key fixture failure and a manual-client fixture's
  missing new field remain in the complete retained logs. The fixtures are
  corrected without relaxing production input validation.
- All sixteen protected user files remain byte-identical.

The synthetic stream control retains thirteen conditional nodes after 28
states and reads a 320,007-byte snapshot through the bulk-file channel.
Two successive late rows install across intervening optimization and
retirement. The no-range trajectory matches its legacy control exactly.

## Same original 15 m inputs

Both replays use the single [original dataset and checksum pointer](../sonar-low-altitude-15m-20261029/data-pointer.json).
All raw sensor clocks, mounts, noise, sixteen-state limits and original
IMU/DVL/pressure observations stay fixed. Camera/Ping and acoustic point
factors remain absent. The disabled-mode replay's 178 poses match the
published no-acoustic baseline exactly.

The enabled-mode replay also emits 178 states at the same timestamps.
Maximum position/quaternion component differences are recorded in the
[full summary](summary.json); the largest component difference is 1.41e-13.
There are 248 accepted controls, including 35 decoded snapshots and finite
conditional recoveries. A thirty-second explicit expiry horizon bounds the
archive at 81 conditional nodes. Largest snapshot size is 2,654,542 bytes.
The maximum native acknowledgment round trip is 22.32 ms; this excludes
Python file decoding/recovery. Offline replay takes 5.7 s disabled and
26.2 s enabled. This paced control does not establish continuous live
throughput. Workers close with the existing client SIGTERM (-15), not a
claimed normal-zero acquisition exit.

Complete replay states, compressed native events/controls, receipts and
all grouped logs remain here. Raw inputs are not duplicated. The original
102 acoustic hypotheses and forty larger prediction errors stay unchanged.
No new map or matched field/cloud overview is claimed because this control
adds no physical acoustic factors. Next qualify raw joint surface rows,
echo/support/noise and useful maps before provider registration, installed
recovery/export, final BlueROV/product/VRX acceptance and later WAM-V survey.
The recorded replay wrapper requires the corresponding WWW-POSIM diagnostic
driver. The library APIs and synthetic native tests run independently of
that application and its private modules.

한국어: 새 네이티브 경로는 과거 자세를 확정값으로 만들지 않고, 현재 상태와의
상관관계를 유지한 새 거리 제약만 추정기 스레드에 전달합니다. 상태 사본의 식별자,
원본 관측의 중복 사용, 조건부 관계의 누락과 만료 시각을 실제 소유자가 검사합니다.
기본값은 꺼짐입니다. 반사면 대응·잡음 보정과 유용한 소나 지도는 아직 검증하지
않았습니다. 합성 제약의 동작 검사와 실제 센서 원본의 수명 관리 검사를 구분합니다.

네이티브 스트리밍 19개 검사와 기존 소유자 검사 10개가 통과했습니다. 독립 소스와
별도 경로에 설치한 wheel은 각각 494개 중 478개가 통과하고, 선택 검사 16개는
명시적으로 생략했습니다. 수동으로 만든 테스트 클라이언트의 새 필드 누락과
관측 시각을 잘못 지정한 초기 실패도 전체 로그에 보존했습니다.

같은 15m 원본을 두 번 재생했으며 각각 178개 상태를 같은 시각에 생성합니다.
기존 경로는 이전 기준 자세와 정확히 같고, 새 경로의 차이는 수치 오차 수준입니다.
248회 제어와 35회 상태 읽기를 완료했고, 30초 만료 범위에서 조건부 노드는 최대
81개, 상태 파일은 최대 2,654,542바이트입니다. 22.32ms는 네이티브 응답만의 값이며
Python 읽기·복원 시간은 제외합니다. 오프라인 재생이며 라이브 처리율 합격은
아닙니다. 원본 센서·실패한 점군·설치된 SDK·LIVE·러너와 사용자 파일을 보존합니다.
