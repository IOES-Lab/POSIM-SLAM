# Ordered camera inputs / 카메라 입력 순서 검증

## English

Camera/IMU and camera/IMU/Ping now use the same original-time event owner as
the optional-sonar/DVL combinations. A native state acknowledgement releases
the next image. Received image bursts yield to output polling; timestamps,
pixels, the 4 MiB worker queue, sensor-age limits and point-admission policy
remain unchanged. The private application adapter owns this scheduling; no
application source is included in the independent library.

The former adapter repeatedly exceeded its input-byte budget during a fresh
field initialization. Yielding between submissions alone still produced a
native sensor-event backlog. The common acknowledgement path removes that
separate unacknowledged camera stream. Deterministic tests cover queued images
released by camera, Ping and IMU callbacks, FIFO preservation, and camera-only
ordering without an auxiliary observation. Nine affected application check
scripts pass.

The recorded camera/IMU/Ping regression passes at 1× in isolated ROS domain120.
Intentional IMU loss empties the old map and rejects its export; input return
creates a fresh frame and map. Both workers and receiver threads stop. The
receipt binds 74 component and nine runtime source files, unchanged throughout
the replay. This is a development integration proof, with 480,000 pixels,
512 feature cells and the patch frontend; it does not qualify the lower defaults.

| Complete export | Points | Surface RMSE | P99 | Beyond 10 cm | Trajectory RMSE |
| --- | ---: | ---: | ---: | ---: | ---: |
| Initial | 3,650 | 13.48 mm | 3.29 cm | 6 | 1.63 cm |
| Recovered | 1,380 | 7.67 mm | 2.05 cm | 1 | 2.71 cm |

Maximum surface distances are 47.03 cm and 11.97 cm. Every point is preserved.
The initial export supports all three targets and both walls; the recovered
export does not cover all of them. These distances to static collision triangles
and the trajectory metrics do not certify complete reconstruction or navigation.
The accepted [6,213-point map](../map-accuracy-review-20261028/README.md) remains
unchanged. Zero threshold exceedances are not an acceptance requirement.

![Initial map and matched isometric RGB](initial-rgb-pcl.png)

![Recovered map and matched isometric RGB](recovered-rgb-pcl.png)

The figures use the same isometric direction, perspective, −4 to −1.5 m Turbo
height limits and 64 pt² markers. Visible/total counts disclose camera clipping.
RGB shows attenuation, suspended particles and seabed caustics, recorded after
the mission from a separate comparison camera. The estimator uses the downward
camera. An explicit experimental particle-shader override is used; it is not
the selected engine's default shader. Alignment uses initial yaw and translation
only, with scale fixed to one. No reference geometry enters estimation.

### Fresh field: tracking passes, map fails

The independent native field run uses a checked 7 m × 4 m, two-cycle survey at
0.6 and 1.2 m depths. Motion starts before demanding moving-initializer poses.
After ten constrained poses the owned worker is deliberately terminated. A new
frame produces five constrained poses, then tracks 1,126 poses over 112.5 s.
There is no additional worker restart. Position RMSE is 2.36 cm, maximum 5.28 cm;
the evaluated path is 45.83 m. Measured target clearance is 0.847 m for a required
0.5 m body envelope; terrain is checked separately by the mission controller.

**The field map is not accepted.** It archives zero points and the panel refuses
map availability. At the recorded panel snapshot, 837 active landmarks remain
outside the requested joint world-position three-sigma budget of 0.1 m. Accurate
measured motion alone does not resolve this uncertainty result. The full test
fails, while its trajectory-only gate passes. The next work concerns startup
information and map uncertainty; this report does not lower the gate or declare
the field ready.

Failure finalization preserves 1,248 camera frames, 12,511 IMU samples, 250 sonar
frames, 1,251 Ping samples, 125 DVL samples and 1,251 pressure samples without
recording errors. The mission observes 120 source seconds; input collection
continues during the failed panel probe. The supervisor exits normally and all
four owned process groups are empty. Raw NPZs/SDKs remain in private storage;
the kit retains the failed probe, source binding, timeline hash and assessment.
It does not claim every sonar timestamp equals a camera timestamp.

`python summarize.py` verifies the manifest, full exports, every-point statistics,
source freeze, loss/recovery and failed-field evidence. Basalt remains
`0f3b2b52c807f70ff4e2973ce253c73329eea7bc`; component code is `6f043c3`.
The camera-only useful map, other sensor combinations, installed GUI, VRX and
WAM-V survey acceptance remain open.

## 한국어

카메라·IMU 및 카메라·IMU·Ping은 선택 소나·DVL 조합과 같은 원본 시각 관측
관리 경로를 사용합니다. 네이티브 상태 응답을 확인한 뒤 다음 영상을 제출합니다.
쌓인 영상 입력 사이에 출력도 읽습니다. 시각·픽셀·워커 4MiB 한도·센서 수신
유효시간·지도 점 채택 기준은 유지합니다. 처리 순서는 비공개 앱 어댑터가
관리하며 독립 라이브러리에 앱 소스를 포함하지 않습니다.

기존 어댑터는 새 현장 초기화 중 입력 바이트 한도를 반복해서 넘었습니다.
영상 제출 사이에 출력 읽기 기회를 주는 변경만으로도 네이티브 관측 대기열
초과가 남았습니다. 공통 응답 확인 경로로 별도 무응답 카메라 전송을 제거했습니다.
카메라·Ping·IMU 콜백에서 풀리는 영상 묶음, FIFO 보존, 선택 센서 관측이 없는
카메라 전용 순서를 시험합니다. 관련 앱 검사 스크립트 9개가 통과합니다.

실제 ROS domain120의 1배속 카메라·IMU·Ping 회귀시험은 IMU 유실 시 기존
지도를 비우고 다운로드를 거부한 뒤 새 좌표 구간과 지도를 만듭니다. 두 워커와
수신 스레드가 종료됐고, 구성요소 74개·런타임 9개 파일이 시험 중 유지됐습니다.
480,000픽셀·특징 셀 512개·patch 프런트엔드의 개발 통합 검증이며 낮은 기본
예산의 검증은 별도입니다. 위 표는 전체 점 평가입니다. 최대 표면 거리는 초기
47.03cm·복구 11.97cm이며 모든 점을 보존합니다. 초기 지도는 세 물체와 두 벽을
지지하지만 복구 지도는 모두 덮지 못합니다. 기존 6,213점 지도도 그대로 유지하며
10cm 초과점을 0개로 만드는 기준은 적용하지 않습니다.

그림은 같은 등각 방향·원근 투영, −4~−1.5m Turbo와 64pt² 점을 사용합니다.
화면 안 점 수와 전체 점 수를 표시합니다. RGB는 임무 뒤 별도 비교 카메라로
기록한 장면이며 수중 감쇠·입자·해저 집광무늬를 포함합니다. 추정 입력은 하향
카메라입니다. 입자 셰이더는 명시한 실험용 오버라이드이며 선택 엔진의 기본
셰이더로 검증한 결과는 아닙니다. 평가 정렬은 초기 yaw·평행이동만 적용하고
배율은 1로 고정합니다. 실제 지형은 추정 뒤 평가에만 사용합니다.

새 네이티브 현장 시험은 폭 4m·길이 7m 경로를 수심 0.6/1.2m에서 두 번
주행합니다. 초기화에 필요한 움직임을 먼저 공급하고 시각 구속 자세 10개 뒤
소유한 워커를 의도적으로 종료합니다. 새 구간이 시각 자세 5개를 생성한 뒤
112.5초 동안 1,126개 자세를 추적하며 추가 재시작은 없습니다. 평가 거리
45.83m에서 위치 RMSE 2.36cm·최대 5.28cm입니다. 시험 물체까지 실제 최소
여유는 0.847m로 요구 몸체 여유 0.5m를 충족하며 지형은 제어기가 별도 확인합니다.

**현장 지도는 불합격입니다.** 채택된 점이 0개여서 패널이 지도 제공을 거부합니다.
패널 기록의 활성 특징점 837개는 세계 좌표의 공동 3시그마 0.1m 기준을 통과하지
못합니다. 실제 궤적 오차가 작아도 이 불확실성 문제는 남습니다. 궤적 단독 검사는
통과하지만 전체 시험은 실패로 보존합니다. 다음 검토 대상은 초기화 정보와 지도
불확실성입니다. 임계값을 낮추거나 현장 준비 완료로 표시하지 않습니다.

실패 종료 처리에서도 카메라 1,248장·IMU 12,511개·소나 250프레임·Ping 1,251개·
DVL 125개·압력 1,251개를 기록 오류 없이 보존합니다. 임무는 원본 시각 120초를
관측하며 실패한 패널 검사 동안에도 입력을 수집합니다. 감독 프로세스는 정상
종료하고 소유한 프로세스 그룹 네 개가 모두 비었습니다. 원본 NPZ·SDK는 비공개
저장소에 두고 이 키트에는 실패 보고서·소스 결합 정보·타임라인 해시·평가를
보존합니다. 모든 소나와 카메라 시각이 일치한다는 주장은 하지 않습니다.

위 재현 명령은 매니페스트·전체 점·오차 통계·소스 유지·유실과 복구·현장 실패를
검사합니다. Basalt 리비전과 구성요소 코드 리비전은 위와 같습니다. 유용한 카메라
전용 지도, 다른 센서 조합, 설치형 GUI, VRX와 WAM-V 측량 검증은 남아 있습니다.
