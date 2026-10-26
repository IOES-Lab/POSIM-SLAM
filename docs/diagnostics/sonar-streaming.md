# Camera-free sonar streaming / 카메라 없는 소나 스트리밍

`SonarSession` owns one bounded sonar-clock segment using the native Basalt
worker, raw IMU, required pressure depth and measured intensity tracks. Optional DVL uses
its original timestamps. Explicit pressure-off configurations are research ablations. Raw gyro interpolation and hydrostatic pressure
conversion are shared by live and recorded adapters. No camera, simulator pose,
ideal orientation or known terrain is an estimator input.

`SonarSession`은 native Basalt worker, 원본 IMU와 측정한 강도 특징 추적으로
소나 시각 기반 세그먼트를 관리합니다. 압력 수심은 필수이며 선택형 DVL과 함께 원래 시각을 사용합니다. 압력을 끈 명시적 설정은 연구용 대조 실험입니다.
실시간·기록 어댑터는 원본 자이로 보간과 정수압 변환 함수를 공유합니다.
카메라·시뮬레이터 위치·이상적인 자세·알려진 지형은 추정기 입력에 포함하지 않습니다.

## Required depth and multiple views / 필수 수심과 여러 시점

The application requires raw IMU and pressure depth for sonar SLAM. Startup waits for valid pressure before allocating a worker. Tracking/export readiness requires a pressure factor acknowledged as included in optimization within two seconds on both sensor and wall clocks. Invalid pressure traffic alone cannot satisfy this condition. A pressure loss/return ROS replay with the current native binary produces 174 poses, one segment reset and a recovered export of 10 hypotheses. All owned workers stop. This is transport/recovery evidence, not map-accuracy qualification.

앱의 소나 SLAM은 원본 IMU와 압력 수심을 필수로 사용합니다. 유효한 압력이 도착해야 워커를 시작합니다. 추적·내보내기 준비 상태에는 센서 시각과 실제 경과 시간 모두 2초 이내인 압력 제약의 최적화 반영 확인이 필요합니다. 무효한 압력 메시지만 수신하는 경우는 이 조건을 만족하지 못합니다. 현재 네이티브 바이너리의 압력 소실·복구 ROS 재생은 자세 174개, 구간 초기화 1회, 복구 후 가설점 10개 내보내기를 확인했습니다. 소유한 워커는 모두 종료했습니다. 전송·복구 근거이며 지도 정확도 검증은 아닙니다.

Multiple observations constrain a shared point or surface hypothesis. Different sensor positions and attitudes can provide elevation information; repeated frames at one pose do not prove observability. Pressure constrains vertical vehicle motion, not seabed clearance. Current association, curved-terrain modeling and joint uncertainty still need qualification. Legacy planar acoustic diagnostics cannot be selected as fused SLAM in the application.

여러 관측으로 공유 점 또는 표면 가설을 구속합니다. 서로 다른 센서 위치·자세는 높이 정보를 제공할 수 있지만, 한 자세의 반복 프레임만으로 관측 가능성을 입증할 수 없습니다. 압력은 차량의 수직 이동을 구속하며 해저 이격 거리를 직접 측정하지 않습니다. 현재 대응 관계·곡면 지형 모델·공동 불확실성은 검증이 남아 있습니다. 기존 평면 음향 진단은 앱의 융합 SLAM 모드로 선택할 수 없습니다.

## ROS replay evidence

The application adapter was exercised through actual ROS topics with an SDF
from which camera sensors were removed. The native binary SHA-256 was
`b236407be035edc95a126ee0bd29f237b71e68cf9e6613c2e72d282dd31cc263`.
The input was recording C from the [causal-depth comparison](sonar-causal-depth.md).
The 60-second raw sequence was replayed at 0.7× speed. It produced 174 pose
messages. A PLY export before interruption contained 71 acoustic hypotheses.
Four seconds of deliberately omitted sonar caused one `sonar_stale` reset,
cleared the current archive and rejected export. Genuine subsequent input
resumed pose output in a new segment/frame. Both worker processes stopped.
The recovered segment registered 24 DVL and 25 pressure optimization states.

카메라 센서를 제거한 SDF로 실제 ROS 토픽과 native worker를 연결했습니다.
입력은 [수직 탐사 비교](sonar-causal-depth.md)의 C 기록입니다. 60초 입력을
0.7배속으로 재생해 자세 메시지 174개를 받았고, 단절 전 PLY에는 음향 가설점
71개가 포함됐습니다. 소나 입력을 4초간 생략하면 `sonar_stale`로 한 번 초기화하고,
현재 점군을 비우며 내보내기를 거절했습니다. 이후 실제 기록 입력으로 새 세그먼트와
좌표계의 자세 출력이 재개됐습니다. 두 worker 모두 종료했습니다. 복구 세그먼트는
DVL 24회, 수압 25회를 최적화에 포함했습니다.

This verifies transport, ownership, export and segment recovery. It does not
qualify terrain mapping, real-time deadlines, a live Gazebo run, 2 m camera
visibility or app-panel integration. Recovery does not establish a nonempty or
accurate recovered map. The export scope is `accumulated_local_sonar_hypotheses`,
with `map_qualified=false` and `navigation_ready=false`. No new dependency or
third-party code was added.

이는 전송·측정 소유권·내보내기·세그먼트 복구 검증입니다. 지형 지도 정확도,
실시간 처리 기한, 실행 중인 Gazebo, 카메라 가시거리 2 m, 앱 패널 연결은 검증하지
않았습니다. 복구 후 점군이 충분하거나 정확하다는 뜻도 아닙니다. 내보내기에는
`accumulated_local_sonar_hypotheses`, `map_qualified=false`,
`navigation_ready=false`를 명시합니다. 새 의존성이나 외부 코드는 추가하지 않았습니다.

## Terrain constraints and low visibility

A subsequent [native low-visibility run](sonar-low-visibility.md) covers camera-free startup, the 2 m PBR rendering condition and application HTTP export. Its map remains unqualified.

후속 [저가시성 네이티브 시험](sonar-low-visibility.md)은 카메라 없는 시작, 2 m PBR 렌더링 조건과 앱 HTTP 내보내기를 확인합니다. 지도 정확도는 미검증입니다.

Known-map terrain-aided navigation correlates measured bathymetry with a prior
map ([NPS](https://nps.edu/web/research/-/analysis-of-undersea-terrain-aided-navigation-2020)).
Map-free terrain SLAM estimates both terrain and pose. FLS terrain smoothness
factors have prior art in [Wang, Shan and Englot, ICRA 2019](https://personal.stevens.edu/~benglot/Wang_ICRA_2019.pdf).
They complement tracked returns; they do not make featureless flat terrain
provide horizontal position information. FLS images lose elevation angle.
View-dependent reflecting points need not be fixed landmarks; [Westman et al.,
IROS 2020](https://www.cs.cmu.edu/~kaess/pub/Westman20iros.pdf) investigate their
geometry through controlled sensor translation.

기존 지도를 쓰는 TAN은 측정 수심과 선행 지도를 정합합니다. 지도 없는 지형 SLAM은
지형과 위치를 함께 추정합니다. Wang 등의 전방 소나 지형 평활 제약은 알려진
선행연구이며, 특징 없는 평지가 수평 위치 정보를 제공하게 만들지는 않습니다.
전방 영상 소나에는 픽셀별 고각 정보가 없고, 시점에 따라 반사점이 이동할 수 있습니다.
Westman 등의 연구는 센서 이동에 따른 반사점 기하를 다룹니다.

Qualification should compare the same raw inputs with terrain constraints off
and on, including flat, repeated and distinctive relief. Camera input must be
unnecessary for initialization; 2 m optical visibility should be verified in
the renderer separately from a camera-absent run. Simulator geometry is scoring
truth, not an undeclared prior map. No novelty claim is made for terrain
smoothness, shadow geometry, active motion or IMU/DVL fusion. A proposed physical
return-association method requires evidence and comparison with this prior art.

검증에서는 동일한 원본 입력에 지형 제약만 켜고 끄며 평지·반복 지형·굴곡 지형을
비교해야 합니다. 초기화는 카메라에 의존하지 않아야 하며, 카메라 없는 시험과 별도로
렌더러의 실제 광학 가시거리 2 m를 확인해야 합니다. 시뮬레이터 지형은 평가 정답이며,
명시하지 않은 선행 지도로 넣지 않습니다. 지형 평활화·그림자 기하·능동 이동·IMU/DVL
융합 자체를 신규 연구 성과로 주장하지 않습니다. 물리적 반사점 연결 방법의 연구
가능성은 이 선행연구와 비교하고 검증해야 합니다.
