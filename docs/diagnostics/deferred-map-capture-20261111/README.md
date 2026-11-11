# Deferred map capture checkpoint — 2026-11-11

The three implementation files in [source-binding.json](source-binding.json)
match the arm64 candidate compiled and linked on 2026-11-10. Their binary and
link provenance were rechecked on 2026-11-11. This source checkpoint does not
change a host application's selected library or activate an installed worker.

The 22 existing Python live-client controls pass. `mapping_graph_probe.cc` adds
exact archive-byte comparison for equal and advanced tracking times, invalid
and throwing callback rejection, destination preservation and caller-argument
replacement. Those new C++ probe cases have not been compiled or executed.

The canonical 190-second host replay remains unaccepted. An initial guarded
worker launch produced no poses before a service-inventory stop; later host
health checks prevented launch. These are not successful map, loss/recovery,
freshness or navigation results. Repeat the complete replay only after fresh
host admission, then check every retained artifact and the original deadlines.

# 지연 지도 캡처 체크포인트 — 2026-11-11

[source-binding.json](source-binding.json)의 구현 파일 세 개는 2026-11-10에
컴파일·링크한 arm64 후보와 일치합니다. 2026-11-11에 바이너리와 링크 출처를
다시 확인했습니다. 이 소스 체크포인트는 호스트 앱의 선택 라이브러리를 바꾸거나
설치된 worker를 활성화하지 않습니다.

기존 Python live-client 대조 22개가 통과했습니다. `mapping_graph_probe.cc`에
동일·진행한 추적 시각의 아카이브 바이트 비교, 잘못된 시각과 callback 예외의
거부, 기존 파일 보존, 호출자 인수 교체 검사를 추가했습니다. 이 새 C++ probe는
아직 컴파일하거나 실행하지 않았습니다.

기준 190초 호스트 재생은 미수용 상태입니다. 최초 보호 실행은 서비스 목록 검사로
중단되기 전 자세를 출력하지 못했고, 후속 시도는 호스트 상태 검사에서 시작이
차단됐습니다. 지도·상실/복귀·freshness·항법 통과 근거로 쓰지 않습니다.
새 호스트 진입 검사를 통과한 뒤 전체 재생, 모든 보존 산출물과 원래 기한을
확인해야 합니다.
