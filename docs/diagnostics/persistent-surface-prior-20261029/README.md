# Persistent joint surface information

`surface_prior.py` and `native/basalt/surface_prior.hpp` retain a square-root
Gaussian prior over poses/inertial states, unknown planes and a pressure datum.
Old source poses can be eliminated while their correlations with surviving
poses and surfaces remain in the same prior. A fitted plane is never installed
as a separate, independent observation. The caller consumes the previous prior
once together with newly whitened raw rows; trial damping is excluded.

Each variable keeps its owner reference. Current residuals use the owner's
accumulated coordinates, including two plane-normal coordinates and signed
distance. Missing references, changed references, a different session, or a new
range whose measurement pose is absent are errors. A full inertial state may
retire velocity/bias coordinates while retaining its six pose coordinates.
Unobservable directions remain unobservable; no epsilon information is added.
The residual-only cost constant is kept in one bounded extra row.

The prototype bounds each system at sixteen state blocks, eight planes and one
pressure datum (265 coordinates); these are diagnostic bounds, not a change to
the application's state-window settings. An original-range ledger retains at
most 32,768 recent keys. The caller may explicitly close admission through a
timestamp, then prune older identities. Seen **and unseen** ranges at or before
that cutoff remain inadmissible. Delayed-association deadlines must be respected
before closing it. Cross-time measurement noise requires a jointly assembled
block or retained latent variables; state marginalization does not manufacture
an independence assumption for future measurement errors.

`surface_prior_owner.hpp` reads actual Basalt 6/15-state references, accumulated
pose/velocity/bias coordinates, plane coordinates and an explicit caller-owned
pressure scalar. It is read-only. These controls do not install a production
prior or retire live IMU/pressure/visual/sonar factors. That complete transaction
and physical support reconstruction remain required before live fusion.

## Measured controls

The delayed-update control starts with three poses, two unknown planes and
21 original ranges. It retires the first pose, keeps the correlated plane
variables, introduces a fourth pose and seven new ranges, and consumes the
previous prior once. The final prior retains poses 2/3/4, both planes and all
28 original measurement identities. The comparison assembles the same raw rows
in one batch before eliminating the first pose.

- Sequential/batch information-matrix maximum difference: 8.74e-11.
- Offset-gradient maximum difference: 7.28e-12.
- Covariance maximum difference: 3.36e-14.
- Residual-only cost difference: 3.64e-12.
- Actual Basalt-owner finite-difference maximum error: 2.46e-9.

Seven source controls and seven native controls (five portable, two actual
owner) pass. They cover dense Schur comparisons, full cross terms, all six
global gauge directions, rank-deficient planes, inertial-to-pose downgrade,
pressure/plane cross information, scalar-only priors, variable permutations,
reference/session rejection and duplicate/cutoff rejection. Source and installed
wheel tests each run 468 cases: 452 pass and sixteen optional legacy checks are
skipped with their reasons retained. The new seven native checks run separately
without skips. These are conditional Gaussian and coordinate controls, not a
new navigation or map-accuracy result.

The owner test initially compared authored rotation matrices bit-for-bit with
Sophus's matrix-to-quaternion canonicalization. The retained failure is a
fixture roundoff difference. Tests now compare authored/owner references within
four machine epsilons while runtime prior/reference matching remains exact.
Owner references are authoritative and must be preserved with the owner state.
Basalt remains at `0f3b2b52c807f70ff4e2973ce253c73329eea7bc`;
[source pins](../../../native/basalt/sources.json) and [notices](../../../THIRD_PARTY.md)
remain unchanged.

## What the actual 15 m record still requires

`delayed-source-paths.json` reclassifies all 664 predictions from the previous
[complete timestamp audit](../joint-surface-ranges-20261029/README.md):

| Observation order | Predictions | Sonar pose still published | Both endpoints still published |
| --- | ---: | ---: | ---: |
| DVL before sonar | 261 | 261 | 165 |
| Sonar before DVL | 403 | 47 | 47 |

No case retains every original support pose. Original support reaches 30.0 s
back at association availability (median 18.49 s). These remain published
snapshot checks, without intermediate owner traces or complete old support
beam IDs. The 356 already-retired sonar poses cannot become new fixed-pose
measurements. Retained Gaussian conditionals/keyframes are needed to recover
their uncertainty and correlation. Plane bootstrap and finite support must also
remain connected to original uncertain observations. Keeping an unknown plane
variable alone does not reconstruct those missing source states.

Next integrate the correlated prior into one complete owner transaction and
retain the conditional information needed by delayed support/sonar poses.
Respect original reading identity, a bounded admission horizon and the existing
noise/freshness/state-window settings. Then qualify physical surface/echo
correspondence and range noise using the same originals. All forty large onset
errors remain in the preceding record. No new sensor capture, truth input,
noise tuning, full-cloud filtering or live surface map is introduced here.
The [actual matched RGB/sonar/field/full-cloud overview](../sonar-low-altitude-15m-20261029/comparison-baseline.png)
continues to represent the measured mapping result.

## Reproduction

From a source checkout with NumPy installed:

```sh
python -m unittest discover -s tests -p 'test_surface_prior.py'
c++ -std=c++17 -O2 -DEIGEN_MPL2_ONLY -I/path/to/eigen3 -I/path/to/json/include \
  native/basalt/surface_prior_probe.cc -o /tmp/surface_prior_probe
POSIM_SLAM_SURFACE_PRIOR_PROBE=/tmp/surface_prior_probe \
  python native/basalt/test_surface_prior_native.py
python docs/diagnostics/persistent-surface-prior-20261029/verify.py
```

Build `basalt_surface_prior_owner_probe` in the existing CMake build with the
pinned SDK/dependencies. Set `POSIM_SLAM_SURFACE_PRIOR_OWNER_PROBE` to it and run
`native/basalt/test_surface_prior_owner.py`. `build-commands.json` records the
executed commands. The verifier recomputes the batch/sequence results and all
664 timestamp classifications. Optional `--native-probe`, `--owner-probe` and
`--source-tree` arguments also check current binaries/exact source fingerprints.
No original sensor archive is copied into this record.

## 한국어

과거 자세를 제거할 때 미지 평면과 압력 기준값을 변수로 남기고, 현재 자세와의
상관을 같은 제곱근 사전정보에 보존합니다. 이전 사전정보와 새 원본 관측은
한 번씩만 사용합니다. 원본 식별자와 명시적인 과거 관측 접수 마감 시각으로
중복 사용을 막습니다. 불확실한 방향에 작은 가짜 정보를 넣지 않습니다.
압력 기준값·평면의 교차 정보와 관성 상태의 속도·바이어스 폐기도 검사합니다.

과거 자세 하나를 제거한 뒤 새 자세·거리 7개를 추가해도, 원본 거리 28개를
한 번에 푼 결과와 공분산 차이가 최대 3.36e-14였습니다. 실제 Basalt 소유자
좌표에서의 미분 오차는 최대 2.46e-9입니다. 새 소스 검사 7개와 네이티브 검사
7개가 통과했습니다. 소스·설치 wheel은 각각 468개 중 452개가 통과하고,
선택형 기존 검사 16개를 사유와 함께 건너뛰었습니다. 실제 상태 참조는
그대로 유지하며, 작성한 회전 행렬과 Sophus 정규화 결과의 비교에만 기계
반올림 오차를 허용했습니다. 런타임의 참조 일치 검사는 여전히 정확 비교입니다.

실제 예측 664건 중 DVL이 먼저 본 261건은 소나 자세가 남아 있고, 소나가 먼저
본 403건 중에서는 47건만 남아 있습니다. 나머지 356건과 최대 30초 전의
지원 관측을 처리하려면, 제거한 자세의 조건부 정보·불확실성도 보관해야 합니다.
평면 변수의 지속 보존만으로 과거 지원 범위가 복원되지는 않습니다.
연결부는 읽기 전용이며 라이브 사전정보 설치와 원본 제약 폐기를 아직 활성화하지
않았습니다. 기존 실제 영상·전체 점군과 큰 오차는 위의 15m 기록에 유지합니다.
