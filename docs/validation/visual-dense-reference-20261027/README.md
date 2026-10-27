# CPU 다중 시점 깊이 복원 기준실험 / CPU multiview depth reference

2026-10-27. Recorded RGB / IMU / DVL input and fixed estimated VIO camera poses.
The unmodified Mapillary OpenSfM CPU depth kernel produces this **offline
reference**. Runtime integration and complete-map qualification remain open.

기록한 RGB·IMU·DVL 입력과 추정 VIO 카메라 자세를 사용한 **오프라인 기준실험**입니다.
Mapillary OpenSfM의 CPU 깊이 복원 커널을 수정 없이 사용합니다. 실시간 앱 통합과
완전한 지도 검증은 아직 완료되지 않았습니다.

![Same-camera isometric map and field RGB comparison](comparison.png)

Left: archived sparse landmarks. Centre: CPU multiview depth reference. Right:
actual field RGB at the same camera pose. Underwater attenuation, suspended
particles and seabed caustics are visible. All admitted points are scored; the
figure projects them into the shared camera view with 64 pt² markers. Each cloud
uses its complete height range for Turbo colours.

왼쪽은 기존 희소점, 가운데는 CPU 다중 시점 복원, 오른쪽은 동일 시점의 실제 RGB입니다.
수중 감쇠·부유 입자·해저 집광무늬를 포함합니다. 모든 생성점을 평가하며, 비교 그림은
동일 카메라 범위에 투영합니다. 점 크기는 64 pt², Turbo 범위는 각 점군 전체 높이입니다.

| Measurement / 측정 | Sparse / 희소점 | Depth reference / 깊이 복원 |
| --- | ---: | ---: |
| Exported points / 내보낸 점 | 2,320 | 322,090 |
| 90th-percentile surface error / 표면 오차 90분위 | 6.19 cm | 3.13 cm |
| Maximum surface error / 최대 표면 오차 | 7.02 m | 23.01 cm |
| Points within 10 cm / 10 cm 이내 점 | 2,176 | 320,382 |

The reference median is 1.33 cm and its 95th percentile is 3.87 cm. Exact
point-to-triangle distances use the physical boxes and seabed only after sealing
the output hash. The independent accelerated evaluator agrees with the existing
Python evaluator within 1.8e-15 m on random and analytic controls.

기준 복원의 중앙값은 1.33 cm, 95분위는 3.87 cm입니다. 출력 해시를 확정한 뒤에만
물체 충돌면과 해저를 이용해 정확한 점–삼각형 거리를 계산합니다. 가속 평가기는
독립 Python 평가기 및 해석적 제어 사례와 일치합니다.

Point count and aggregate error do not establish completeness. On a 5 cm model
face grid, top-face coverage within 5 cm changes as follows. This descriptive
metric includes all model faces; visibility and association are not inferred.

점 수와 전체 오차만으로 완성도를 판정하지 않습니다. 물체 상부를 5 cm 격자로 표본화해
복원점과의 거리가 5 cm 이내인 비율을 비교합니다. 모든 면을 대상으로 하는 기술적
지표이며 가시성이나 물체 대응을 판정하는 값은 아닙니다.

| Top face / 상부 | Sparse / 희소점 | Depth reference / 깊이 복원 |
| --- | ---: | ---: |
| Target 0 | 59.3% | 74.5% |
| Target 1 | 1.5% | **0.0%** |
| Target 2 | 57.9% | 100.0% |

Wall 1 is also almost absent. **The complete map is not qualified.** The retained
baseline-only neighbour graph is compared with image covisibility below. No
threshold was relaxed or scene-based point filtering used to qualify this result.

벽 1도 거의 복원되지 않았으므로 **완전한 지도 합격으로 인정하지 않습니다.**
아래 추가 실험에서 거리 기반 인접 뷰 선정과 실제 공통 영상 특징을 이용한
선정을 비교합니다. 합격을 위해 임계값을 완화하거나 장면 기반 점 제거를 하지 않았습니다.

The run computes 98 depthmaps, cleans by cross-view consistency, and prunes/merges
16 reference views. Processing takes 464.3 s with 337.4 MiB maximum RSS on Apple
Silicon macOS. It uses the existing SDK; full RGB records and depthmaps remain
local. [Settings and summary](summary.json), [face coverage](surface-coverage.json),
[complete PLY](map.ply), and [analytic controls](analytic-controls.json) are retained.

98개 깊이맵을 계산한 뒤 시점 간 일관성 검사와 중복 제거를 적용해 기준 16시점을 합칩니다.
Apple Silicon macOS에서 464.3초, 최대 RSS 337.4 MiB를 측정했습니다. 기존 SDK를
재사용하며 원시 RGB와 깊이맵은 로컬에 보존합니다. 전체 점군과 요약은 위 파일에 있습니다.

Kernel: [Mapillary OpenSfM 65571c8](https://github.com/mapillary/OpenSfM/tree/65571c8ae9ac6a1d0bcd3fdc1ea5fff97b1b5611),
BSD-2-Clause; [retained notice](opensfm-license.txt). OpenCV core/imgproc 4.13.0
comes from the existing SDK. These are conventional photometric depth estimation
and geometric consistency methods, not a new research-method claim.

Scene/texture: [Orca5 5f7cdc1](https://github.com/clydemcqueen/orca5/tree/5f7cdc1f044926ebd56f2ce8a1fc4a82990cfa5e),
MIT; [retained notice](orca5-license.txt). 장면·텍스처의 Orca5 MIT 고지를 보존합니다.
이 실험은 기존 광도 기반 깊이 추정과 기하 일관성 방법의 기준 검증이며 신규 학술 방법을 주장하지 않습니다.

## Observed-track neighbours / 관측 특징 기반 인접 뷰

![Observed-track depth map and identical field RGB](covisibility-comparison.png)

Only the neighbour graph changes. The published OpenSfM rule ranks views sharing
more than 20 observed VIO tracks with estimated-point parallax between 3° and 30°;
at most seven matching views enter the same depth kernel. Images, estimated poses,
16 reference views and depth/consistency thresholds stay fixed. No simulator
geometry enters view selection or reconstruction.

인접 뷰 선정만 변경했습니다. OpenSfM의 기존 규칙에 따라 추정 점의 시차가
3°–30°이고 공통 VIO 관측 특징이 20개를 초과하는 뷰를 점수순으로 최대 7개
선택합니다. 영상·추정 자세·기준 16시점·깊이 및 일관성 임계값은 동일합니다.
시뮬레이터 형상은 뷰 선정과 복원에 사용하지 않습니다.

The result retains and scores all **364,716 points**: median error 1.31 cm,
90th percentile 3.02 cm, 95th percentile 3.56 cm, maximum 52.98 cm, and 364,636
points within 10 cm. Target 0 top-face coverage within 5 cm increases from 74.5%
to 91.7%; target 2 remains 100%. Target 1 top-face coverage stays **0%**, and wall
1 has no points within 10 cm. **The complete map still fails qualification.**
Better neighbours alone do not resolve the missing surfaces; their input-view
visibility and texture observability require inspection before further changes.

전체 **364,716점**을 보존하고 평가했습니다. 오차 중앙값 1.31 cm, 90분위
3.02 cm, 95분위 3.56 cm, 최대 52.98 cm이며 364,636점이 10 cm 이내입니다.
물체 0 상부의 5 cm 이내 복원 비율은 74.5%에서 91.7%로 증가하고 물체 2는
100%를 유지합니다. 물체 1 상부는 여전히 **0%**이며 벽 1은 10 cm 이내 점이
없습니다. **완전한 지도는 계속 불합격입니다.** 인접 뷰 선정만으로 누락 면을
해결하지 못했으므로 추가 변경 전에 입력 영상의 가시성과 텍스처 관측성을 확인합니다.

Processing takes 461.0 s and 388.3 MiB maximum RSS. The same-camera figure uses
64 pt² markers and full-field Turbo height ranges, with underwater attenuation,
particles and seabed caustics in RGB. [Summary](covisibility-summary.json),
[face coverage](covisibility-surface-coverage.json) and
[complete PLY](covisibility-map.ply) preserve this separate experiment.

처리 시간은 461.0초, 최대 RSS는 388.3 MiB입니다. 동일 카메라 비교 그림은
64 pt² 점과 전체 높이 범위의 Turbo 색상을 사용합니다. RGB에는 수중 감쇠·입자·
해저 집광무늬가 포함됩니다. 이 추가 실험의 요약·면별 복원 비율·전체 PLY를
별도 파일로 보존합니다. 실시간 통합과 설치형 앱 검증은 아직 완료되지 않았습니다.
