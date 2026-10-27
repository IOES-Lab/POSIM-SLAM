# Visual-map information contract / 시각 지도 정보 계약

`posim_slam.map_quality.native_settings()` requests the native joint linearized
information query, a 0.1 m world-position three-sigma budget and reprojection
culling of stored points when their IDs are observed. The native worker's bare
configuration keeps these optional settings off. A host explicitly requests
them and validates the acknowledgement with `validate_ready()`.

`validate_geometry(report, window)` checks world coordinates, included pose
uncertainty, the absence of damping, matching counts and the requested budget.
An unobserved or empty window is valid. Missing information or substituting
host-camera coordinates is an error. The joint query marginalizes shared
nuisance variables together and includes enabled native auxiliary factors and
the marginal prior. Fixed calibration, native position/yaw gauge and sensor
noise assumptions remain; the report does not certify map accuracy.

`native_settings()`는 공동 선형화 정보 계산, 세계 좌표 위치의 3시그마 한도 0.1m,
같은 ID를 다시 관측할 때 저장점 재투영 검사를 요청합니다. 네이티브 작업자의 기본
구성에서는 이 선택 기능을 끕니다. 호출자가 명시적으로 요청하고 `validate_ready()`로
응답을 검증합니다.

`validate_geometry()`는 세계 좌표, 자세 불확실성 포함, 감쇠 제외, 점 수와 한도를
검사합니다. 관측 정보가 부족하거나 창이 비어 있는 결과는 유효합니다. 정보가 없거나
카메라 상대 좌표로 대신한 결과는 오류입니다. 공유 보조변수를 함께 소거하고 선택한
네이티브 보조 센서 요인과 주변화 사전 정보를 포함합니다. 고정 보정값·초기 위치와
방위 기준·잡음 가정은 남으며 지도 정확도를 보증하지 않습니다.

- [Contract implementation](../../src/posim_slam/map_quality.py)
- [Joint native information](../../native/basalt/joint_map_information.hpp)
- [Host and world coordinate query](../../native/basalt/map_information.hpp)
- [Executable contract controls](../../tests/test_map_quality.py)

[Native RGB/map and recovery evidence / 네이티브 비교·복구 기록](../validation/visual-live-map-information-20261028/README.md).
