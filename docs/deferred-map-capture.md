# Deferred tracking capture for map exports

`MappingGraphRegistry::writeWithInputsAtCapture` prepares the complete immutable
graph and its original input pages before calling the supplied tracking-time
callback once. It flushes the input pages to the unpublished temporary file
and checks stream errors before capture. This is a stream flush, not a disk
durability guarantee. The timestamp must be positive and cover every graph frame.
The callback can throw to reject publication. The existing `writeWithInputs`
API keeps its caller-supplied timestamp and archive format.

The stream pins one mapping revision before preparation. After preparation it
reads the actual latest tracking snapshot under the snapshot mutex, retaining
the pinned map's points, IDs, candidates and graph. It checks mapper health and
the original 10-second export deadline before capture, after writing, and while
holding the output mutex before emitting success. The deadline starts before
waiting for the requested mapping revision. The host still owns freshness
validation, its timeout and supervision of a blocked output pipe.

The graph file is published by rename. PLY, graph, candidates and trace are
separate files; a later failure can leave an unaccepted partial set. Consumers
must require a successful response and validate its request and session.

# 지도 내보내기의 지연된 추적 자세 캡처

`MappingGraphRegistry::writeWithInputsAtCapture`는 불변 지도 그래프와 원본 입력
페이지를 모두 준비한 뒤 추적 시각 callback을 한 번 호출합니다.
캡처 전에 입력 페이지를 미게시 임시 파일로 flush하고 스트림 오류를 확인합니다.
이는 스트림 버퍼의 쓰기 완료이며 디스크 영구 저장을 보장하지는 않습니다. 시각은 양수이며
모든 그래프 프레임의 시각 이상이어야 합니다. Callback이 예외를 발생시키면
파일 게시를 거부합니다. 기존 `writeWithInputs` API는 호출자가 준 시각과
아카이브 형식을 유지합니다.

Stream은 준비 전에 지도 revision 하나를 고정합니다. 준비가 끝나면 snapshot
mutex 아래에서 실제 최신 추적 자세를 읽으며, 고정한 지도의 점·ID·후보·그래프를
유지합니다. 캡처 전, 파일 작성 후, 성공 응답 직전의 output mutex 안에서 지도
상태와 기존 10초 기한을 검사합니다. 기한은 요청한 지도 revision을 기다리기
전부터 계산합니다. 호스트는 freshness 검사, 자체 제한 시간과 응답 파이프의
쓰기 정지 감시를 담당합니다.

그래프 파일은 rename으로 게시합니다. PLY·그래프·후보·trace는 별도 파일이므로
후속 실패 시 승인되지 않은 일부 파일이 남을 수 있습니다. 소비자는 성공 응답을
확인하고 요청과 세션이 일치하는지 검증해야 합니다.
