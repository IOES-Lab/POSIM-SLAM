# Joint DVL / FLS surface ranges

## Conditional model and real owner controls

`range_surface_joint.py` and `native/basalt/range_surface.hpp` linearize original
per-beam ranges against current poses and unknown local planes. DVL uses the
first continuous finite-cone return. FLS uses the first continuous vertical-fan
intersection, including an interior extremum. Both preserve mounting offsets,
signed planes and sensor limits. This is an ideal geometric model; real ray
sampling, acoustic response and echo identity need separate qualification.

Every reading has one `(stamp_ns, sensor_id, beam_id)` key. A factor declares
its surface hypothesis. Raw cone readings may provide support without becoming
additional factors. Their nominal centre-ray seeds are recomputed from current
poses and immutable ranges; frozen world XYZ support is rejected. The entire
cone footprint must fit the observed hull and a conservative sample-gap bound.
Support guards add no Cartesian measurement or fitted-plane prior.

The block uses the full supplied range covariance. Support-only rows use the
active principal submatrix, rather than conditioning on unfused readings.
Unknown noise is rejected; simulator resolution is not a variance estimate.
The bounds are 16 poses, eight planes and 512 original readings per block.
Accumulated pose/plane rotation coordinates include the SO(3) left Jacobian.

`makeOwnedRangeSurfaces` places these rows in the existing Basalt grouped
elimination, trial update/rollback and marginal-prior system. Every original
support pose is an owner dependency. A retired source is rejected before
Basalt's fatal missing-pose path. The whole group and its unknown planes retire
together; this adapter does not retain a persistent plane across retirement.

Six native/source tests pass. The accumulated-coordinate fixture has three
poses, two planes and 21 original factor rows with correlated noise. Maximum
native/source Jacobian difference is 7.82e-14; actual-owner finite-difference
error is 3.27e-8. Dense joint elimination and the owner's marginal information
agree. Optimization reduces cost from 3475.09 to 42.52 in twenty accepted steps
and reaches its iteration limit; this is not a convergence or navigation claim.
Installation removes the whole retired group; replay and retired-source reuse
are rejected. A second fixture has two factor rows and four support-only ranges
at five poses, with no independent support information added.

The source controls also verify all six global rigid-motion null directions,
full-covariance permutation invariance and the false confidence caused by
fixing an unknown plane. An independently built wheel passes 445 of 461 tests;
sixteen optional legacy native/decoder checks are skipped with exact reasons in
`wheel-receipt.json`. The six new native tests run separately, without skips.
No private application implementation is required. Basalt remains pinned at
`0f3b2b52c807f70ff4e2973ce253c73329eea7bc`; see [source pins](../../../native/basalt/sources.json)
and [notices](../../../THIRD_PARTY.md).

## Actual 15 m record: retention is the next boundary

The audit reuses every one of the [664 independent predictions](../sonar-low-altitude-15m-20261029/README.md),
including all forty errors beyond 10 cm. It adds no sensor capture, truth input,
range filtering or map point. At each association's availability it checks the
first published native retained-state snapshot, including both endpoint poses
and every DVL support timestamp. Raw support beam IDs are not present in that
older report, so this audit proves timestamp dependency coverage only.

- Published snapshots cover all 664 predictions.
- Both endpoint poses remain in 212 cases.
- All original support-source poses remain in **zero** cases.

These are published snapshots, not traces of intermediate owner transactions.
The result rules out claiming complete source ownership from endpoint coverage.
Before enabling live surface fusion, preserve unknown surfaces and their
correlations through marginalization, together with valid support provenance.
Do not replace retired support by a fitted plane with an independent prior or
increase the pose window to conceal the lifetime problem. Ambiguous/nearer
echoes and calibrated range uncertainty remain open. The production worker and
map-admission settings are unchanged; no new live map is qualified here.
The existing [matched real RGB/sonar/field/full-cloud overview](../sonar-low-altitude-15m-20261029/comparison-baseline.png)
remains the measured mapping result.

## Reproduction

Run from a source checkout with NumPy installed:

```sh
python -m unittest discover -s tests -p 'test_range_surface_joint.py'
python -m unittest discover -s tests -p 'test_surface_pose_retention.py'
c++ -std=c++17 -O2 -DEIGEN_MPL2_ONLY -I/path/to/eigen3 -I/path/to/json/include \
  native/basalt/range_surface_probe.cc -o /tmp/range_surface_probe
POSIM_SLAM_RANGE_SURFACE_PROBE=/tmp/range_surface_probe \
  python native/basalt/test_range_surface_native.py
python docs/diagnostics/joint-surface-ranges-20261029/verify.py
```

The native-owner target is `basalt_range_surface_owner_probe` in the existing
CMake build configured with the pinned SDK/dependency sources. Set
`POSIM_SLAM_RANGE_SURFACE_OWNER_PROBE` to that binary and run
`native/basalt/test_range_surface_owner.py`. `build-commands.json` records the
executed macOS compile/link commands using the existing source-hashed core.
`qualification.json` retains source/binary hashes and exact control outputs.
`surface-pose-retention.json` keeps every audited prediction. The verifier
recomputes its result from the existing compact 15 m reports; no raw archive is
copied. Use `--source-tree PATH` to verify the exact candidate source hashes.

## 한국어

빔별 원본 거리와 현재 자세·미지 평면을 같은 계산 블록에서 선형화합니다.
DVL은 유한 원뿔의 첫 연속 교점, 전방 소나는 수직 팬 전체의 첫 연속 교점을
사용하며 내부 극값도 확인합니다. 실제 광선 샘플링·음향 응답·에코 대응의
검증은 별도로 필요합니다. 각 관측의 시각·센서·빔 식별자를 한 번씩 유지합니다.
관측 범위 검사용 원본 거리는 현재 자세에서 다시 계산하고, 고정된 세계 좌표나
별도의 평면 사전정보로 바꾸지 않습니다. 전체 공분산과 누적 회전의 미분을
유지하며, 미지의 잡음은 명시적인 보정 모델 없이 0으로 채우지 않습니다.

Python/C++ 비교와 실제 Basalt 소유자 검사 여섯 개가 모두 통과했습니다.
누적 회전·두 평면 시험의 계산 차이는 최대 7.82e-14, 실제 상태 갱신을 사용한
미분 차이는 최대 3.27e-8입니다. 기존 묶음 소거·시험 복원·주변화 경로에서
관측 그룹 전체를 처리하고, 폐기한 자세나 이미 사용한 그룹의 재사용을 거부합니다.
비용은 3475.09에서 42.52로 줄었으나 반복 한도에서 끝났으므로 수렴·항법
성공을 뜻하지 않습니다. 별도 설치한 wheel의 461개 검사 중 445개가 통과했고,
선택형 기존 네이티브·디코더 검사 16개는 사유와 함께 건너뛰었습니다.

실제 15m 기록의 예측 664건과 10cm 초과 오차 40건을 모두 유지했습니다.
관측을 연결할 수 있는 시점의 발행된 상태를 검사하면 양 끝 자세가 남은 경우는
212건이지만, 원본 DVL 지원 관측의 모든 자세가 남은 경우는 0건입니다.
중간 소유자 처리 순서와 원본 지원 빔 식별자까지 추적한 증거는 아닙니다.
다음 구현은 과거 평면과 현재 자세 사이의 상관을 주변화 후에도 보존해야 합니다.
에코 대응과 거리 잡음 보정도 남아 있어 라이브 평면 융합은 활성화하지 않았습니다.
같은 실제 RGB·소나·현장·전체 점군 비교는 위의 15m 기록 링크에 보존합니다.
