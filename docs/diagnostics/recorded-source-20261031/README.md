# Recorded source / 기록된 소스

These three public module snapshots exactly match the original 2026-10-31
experiment source hashes. They are retrieved from public commit `99bb12c8`;
no private working-tree source is copied. Original dated records, verifiers,
thresholds, inputs and failed maps remain unchanged.

Use `python tools/verify_recorded_diagnostic.py docs/diagnostics/RECORD/verify.py`
to run a source-bound historical verifier in an independent temporary package.
Every snapshot is hash-checked before use; the package workspace is limited to
32 MiB and removed after exit. CI tests current implementation behaviour and
the new complete sampled-ray field separately. Current/default compatibility
controls compare against these exact prior modules.

세 공개 모듈은 과거 실험에 기록된 소스 해시와 같습니다. 비공개 작업 파일은
복사하지 않습니다. 원본 기록·검증기·판정 기준·입력·실패 지도를 유지합니다.
위 명령은 해시를 검사한 별도 임시 패키지로 과거 검증기를 실행하며 종료 후
제거합니다. 현재 구현과 새 음선 보정 검사는 별도로 실행합니다. 새 기본
동작은 원본 모듈과 직접 비교합니다. 각 소스의 Apache-2.0 고지와 루트
LICENSE를 유지합니다.
