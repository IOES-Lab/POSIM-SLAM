# Moving sonar, IMU, pressure and DVL: current sensor record

This record uses the POSIM current-main sonar integration `867d214`, with the
source-bound application corrections and Metal binary identified in
`source.json`. It has no single Ping sensor. A real ArduSub controller completes
the seven authored waypoints. Assessment-only truth measures 18.47 m travel,
7.06 × 4.04 × 1.11 m position extent and 185.38° heading span during motion.

All original estimator-input frames are retained in **one** `capture.tar.gz`.
It contains 132 sonar frames, 330 full RGB exposures, 6,592 raw IMU samples,
66 bottom-locked DVL measurements and 659 pressure samples. The timeline and
source/gray8 checksums preserve the received pixels and timestamps. Truth is a
separate assessment file and never an estimator input. The complete sonar
replay span is 2.50–67.01 s, with at most 10 ms between IMU samples. Native
acquisition stops normally; the retained native replay workers close by SIGTERM
through their existing client-close protocol, separately from acquisition.

RGB and sonar share the physical origin, direction and 120° horizontal field of
view in this diagnostic rig. Sonar elevation is 20°, range is limited to 10 m
and output is 256 beams × 399 bins at about 2 Hz. The 5 Hz RGB stream has fifteen
exact sonar-time pairs; other nearest exposures differ by up to 100 ms. Camera
input is excluded from both estimates below. The figure uses an exact 36 s pair.

The PBR water-path profile bounds transmission to 1% at 2 m. Camera clipping
remains 100 m. Sunlight, caustics, suspended particles and waves are enabled.
The lower-left panel is an explicitly separate clear-water field re-render at
the recorded body pose: haze and absorption are off for context, with the other
effects retained. The original low-visibility isometric RGB is also included.
Neither overview image nor its annotated geometry enters estimation.

## Same-input baseline comparison

Both runs use unchanged committed POSIM-SLAM `e0521d6` sources and the same
native binary, calibration, pressure/DVL times and diagnostic noise. These noise
scales are research assumptions, not hardware calibration. The native acoustic
history remains sixteen states; no map-admission or geometry check is relaxed.
Alignment fixes only the initial yaw and translation, with scale fixed at one.

| Existing frontend | All emitted hypotheses | Trajectory RMSE | Surface RMSE / p90 / maximum | Beyond 10 cm |
| --- | ---: | ---: | --- | ---: |
| Intensity features | 96 | 0.0459 m | 0.2239 / 0.4035 / 0.9159 m | 43/96 |
| Echo boundaries | 34 | 0.2190 m | 0.3420 / 0.5492 / 0.8177 m | 26/34 |

The distance is the exact nearest physical box/seabed surface distance for
**every** emitted point. The 10 cm count is a diagnostic, not a universal marine
acceptance standard. It does not prove correspondence identity or surface-area
coverage. The archive explicitly lacks verified physical identity and joint
landmark uncertainty. Both maps remain unqualified. A smaller maximum on the
sparser boundary map does not establish improvement.

![Intensity features, full map and matched sensor/field views](comparison-baseline.png)

![Echo boundaries on the same input and reference views](comparison-boundary.png)

Both figures retain every point at 64 pt², with common blue-to-red Turbo height
limits. Grey boxes are assessment references. The raw RGB/sonar pair is at 36 s;
the accumulated map uses observations through 67.01 s. The comparison points to
association and 3D-landmark modelling as the next diagnostic boundary, while
retaining pose, calibration and uncertainty questions. No new algorithm or
research novelty is claimed by this acquisition/comparison.

The model requests 1,000 Hz IMU but this 10 ms physics profile actually publishes
100 Hz. DVL publishes about 1 Hz and pressure about 10 Hz. These measured rates
are separate from configured rates and wall-clock RTF. The baseline keeps its
declared-rate calibration; no timestamp, sample, noise or physics change is
made to improve these results.

`python3 verify.py` checks file hashes, the raw archive inventory and all retained
point/error populations with the standard library. `capture.tar.gz` contains
sensor data/configuration only, without WWW-POSIM implementation sources.
Simulator-derived meshes/images retain the [Orca5 MIT notice](ORCA5-LICENSE.txt).
The source repository is [clydemcqueen/orca5](https://github.com/clydemcqueen/orca5).
Library, OpenCV, Basalt and other component notices remain in the repository's
existing license/notice records. Data capture and mapping quality are separate
from SDK/package qualification.

## 한국어

최신 소나 커널로 실제 제어기의 이동을 기록했습니다. 소나·IMU·압력·DVL을
사용하고 단일 Ping은 없습니다. 원본 입력은 하나의 압축 파일에 보존하며,
정답 자세는 평가용으로만 분리합니다. 수중 RGB는 2 m에서 표면 투과율 1%인
조건입니다. 비교용 RGB와 소나는 같은 위치·방향이며, 그림의 두 원본 노출
시각은 36초로 같습니다. 아래의 등각 방향 RGB는 지형을 알아볼 수 있도록
감쇠를 끈 별도 참조 렌더링입니다. 원래의 짙은 안개 RGB도 함께 보존합니다.

같은 입력·바이너리·잡음으로 기존 두 방법을 비교했습니다. 명암 특징 방법은
96점 중 43점, 반사 경계 방법은 34점 중 26점이 실제 표면에서 10 cm를 벗어납니다.
전체 점을 평가하고 표시했으며, 작은 최댓값만으로 경계 방법을 개선으로
판단하지 않습니다. 아직 신뢰할 수 있는 물체 복원으로 인정하지 않습니다.
다음 진단은 동일 특징의 물리적 지속성과 다중 시점의 3차원 제약입니다.
허용 오차·신선도·잡음을 바꾸지 않습니다.
