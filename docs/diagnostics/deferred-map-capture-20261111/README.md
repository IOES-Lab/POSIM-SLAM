# Deferred map capture checkpoint — 2026-11-11

## Complete native probe after page flush

[Run 43024427572](https://github.com/IOES-Lab/POSIM-SLAM/actions/runs/43024427572)
passed at `cbe24fc9e543dfcb70ebe3be027175167a49be83`: all 105 build/link steps
and the complete graph/window/input archive probe. The probe is byte-identical
to the failing run; no assertion, archive format or deadline was relaxed.
Artifact `13858140002` has ZIP SHA-256
`ee7dfe51c3b6e68ce901f4419fcaf3a95ebff8a06103281a2bd3952a79a13953`.
All ten archived files match the receipt, and its probe/lock hashes match this
source. The receipt also matches the raw job log.
[Native receipt](mapping-controls-cbe24fc.json) ·
[Independent artifact checks](mapping-artifact-check-cbe24fc.json).

This is Linux C++ control evidence. A fresh arm64 worker build and the original
190-second host HTTP/ROS replay remain pending. The earlier arm64 binary is not
reused as evidence for the changed native source. The independent integrity run
`43024427418` passed Python 3.10/3.12 and native Ping.
[Terminal job results](integrity-cbe24fc.json).

## Earlier failure and historical arm64 binding

The historical implementation files in [source-binding.json](source-binding.json)
match the arm64 candidate compiled and linked on 2026-11-10. Their binary and
link provenance were rechecked on 2026-11-11. The page-flush correction below
changes `mapping_graph.cc`; that candidate no longer matches this source.
This source checkpoint does not
change a host application's selected library or activate an installed worker.

The 22 existing Python live-client controls pass. `mapping_graph_probe.cc` adds
exact archive-byte comparison for equal and advanced tracking times, invalid
and throwing callback rejection, destination preservation and caller-argument
replacement. Linux CI run `43016030159` at `1f15991` compiled the full target
but failed the callback's nonempty temporary-file assertion. Artifact
`13852894701` preserves a 4,998-byte reference archive and a zero-byte temporary
file (ZIP SHA-256 `a1b1543332c027ccf5337932b74edd9d7bd16a4249854dc90cd6af3bb02ff60d`).
The stream had not flushed its buffered input pages before capture. The fix
flushes and checks the stream before invoking the callback. The assertion,
archive format, publication rule and deadlines are unchanged. The replacement
probe passed as recorded above; the failing run remains preserved.

The independent Python 3.10/3.12 and native Ping jobs in run `43016030227`
passed at `1f15991`. They do not qualify the C++ mapping target.

The canonical 190-second host replay remains unaccepted. An initial guarded
worker launch produced no poses before a service-inventory stop; later host
health checks prevented launch. These are not successful map, loss/recovery,
freshness or navigation results. Repeat the complete replay only after fresh
host admission, then check every retained artifact and the original deadlines.

# 지연 지도 캡처 체크포인트 — 2026-11-11

## 페이지 flush 수정 후 전체 native probe

`cbe24fc9e543dfcb70ebe3be027175167a49be83`의 run `43024427572`에서 105개
빌드·링크 단계와 전체 graph/window/input archive probe가 통과했습니다.
실패한 run과 probe의 바이트가 같습니다. Assertion·파일 형식·기한은 그대로입니다.
Artifact `13858140002`의 ZIP SHA-256은 위 영문과 같습니다. 아카이브 10개를
모두 receipt와 대조했고, probe/lock hash가 현재 소스와 같습니다. Receipt도
원본 job log와 일치합니다. 위 두 JSON에 전체 검증을 보존했습니다.

Linux C++ 대조 범위입니다. 새 arm64 worker build와 원래 190초 호스트
HTTP/ROS 재생은 남아 있습니다. 변경된 native source의 근거로 이전 arm64
바이너리를 재사용하지 않습니다. 별도 integrity run `43024427418`의 Python
3.10/3.12와 native Ping 작업도 모두 통과했습니다. 위 JSON에 종료 결과가 있습니다.

## 이전 실패와 과거 arm64 바인딩

[source-binding.json](source-binding.json)의 과거 구현 파일 세 개는 2026-11-10에
컴파일·링크한 arm64 후보와 일치합니다. 2026-11-11에 바이너리와 링크 출처를
다시 확인했습니다. 아래 flush 수정으로 `mapping_graph.cc`가 바뀌므로 해당
arm64 후보는 현재 소스와 일치하지 않습니다. 이 소스 체크포인트는 호스트 앱의 선택 라이브러리를 바꾸거나
설치된 worker를 활성화하지 않습니다.

기존 Python live-client 대조 22개가 통과했습니다. `mapping_graph_probe.cc`에
동일·진행한 추적 시각의 아카이브 바이트 비교, 잘못된 시각과 callback 예외의
거부, 기존 파일 보존, 호출자 인수 교체 검사를 추가했습니다. Linux CI run
`43016030159`의 `1f15991`은 전체 target을 컴파일했지만 callback의 임시 파일
크기 검사에서 실패했습니다. Artifact `13852894701`은 4,998바이트 기준
아카이브와 0바이트 임시 파일을 보존합니다. ZIP SHA-256은 위 영문과 같습니다.
원본 입력 페이지가 캡처 시점에 스트림 버퍼에 남아 있었습니다. Callback 전에
flush하고 오류를 검사하도록 수정했습니다. Assertion·파일 형식·게시 규칙·기한은
같습니다. 새 전체 native probe의 통과는 위에 기록했고 실패한 run도 보존합니다.

`1f15991`의 별도 run `43016030227`에서 Python 3.10/3.12와 native Ping 검사는
통과했습니다. C++ mapping target의 통과 근거로 쓰지 않습니다.

기준 190초 호스트 재생은 미수용 상태입니다. 최초 보호 실행은 서비스 목록 검사로
중단되기 전 자세를 출력하지 못했고, 후속 시도는 호스트 상태 검사에서 시작이
차단됐습니다. 지도·상실/복귀·freshness·항법 통과 근거로 쓰지 않습니다.
새 호스트 진입 검사를 통과한 뒤 전체 재생, 모든 보존 산출물과 원래 기한을
확인해야 합니다.
