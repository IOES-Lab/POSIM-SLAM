# IMU delivery at state events / 상태 시각에 맞춘 IMU 전송

`posim_slam.event_imu.EventImu` releases raw IMU from a caller-owned bounded history. Camera and camera-free sonar use the same delivery rule. The caller validates raw samples and waits for the exact native state acknowledgement before releasing another event.

`select(samples, event_ns)` returns the unsent contiguous interval through the first raw sample strictly after `event_ns`. Bootstrap starts with the existing 200 ms history. The next interval starts after the last transmitted sample. A closely spaced auxiliary event can already be covered by that sample and needs no duplicate IMU packet.

Future samples stay in the caller's history while the native worker processes the current state. Missing future coverage, overwritten history and raw gaps over 20 ms fail before the selected interval is transmitted. The helper changes packet delivery time; original sensor timestamps and values remain unchanged. Native input limits, stale deadlines, sensor noise and map-admission criteria remain the caller's existing contracts.

## 한국어

`posim_slam.event_imu.EventImu`는 호출자가 보관하는 제한된 원본 이력에서 IMU를 전송합니다. 카메라와 카메라 없는 소나는 같은 규칙을 사용합니다. 호출자는 원본 측정값을 검증하고 정확한 네이티브 상태 응답을 확인한 뒤 다음 상태를 처리합니다.

`select(samples, event_ns)`는 아직 전송하지 않은 연속 구간에서 해당 상태 시각 직후의 첫 IMU 샘플까지 반환합니다. 시작 구간은 기존 200ms 이력을 사용합니다. 다음 구간은 마지막 전송 샘플 이후부터 시작합니다. 가까운 보조 센서 상태는 이미 전송한 직후 샘플로 덮일 수 있으며, 이때 IMU를 중복 전송하지 않습니다.

워커가 현재 상태를 처리하는 동안 이후 샘플은 호출자의 이력에 남습니다. 직후 샘플 부재·덮어쓴 이력·20ms를 넘는 원본 단절은 해당 구간을 전송하기 전에 오류로 처리합니다. 원본 센서 시각과 측정값을 유지하며 native 입력 한도·신선도·잡음·지도 채택 기준은 바꾸지 않습니다.

[Implementation](../../src/posim_slam/event_imu.py) · [Timing and history controls](../../tests/test_event_imu.py)
