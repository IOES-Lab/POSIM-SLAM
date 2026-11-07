# Correlated delayed surface updates

`delayed_surface_update.prepare_delayed_ranges` prepares a joint update from
whitened raw-range rows, immutable pose/plane references, the live information
and the retiring-state conditional archive. It returns updated conditionals,
the complete folded prior, new live rows and the original-range ledger.

The native owner keeps its active nonlinear factors and installs **only the new
live rows**, together with the updated archive and ledger. A caller replacing
the complete consumed linear system uses the folded prior instead. Installing
both duplicates information. Prior identities, revisions, epochs, original
reading keys and source snapshots are checked before publishing the result.
Preparation leaves every input unchanged on success or rejection.

For each retiring block, write `x_old = M x_separator + d + L noise + U free`.
Substitution into the new likelihood propagates its covariance as
`I + A L Lᵀ Aᵀ`; free directions are projected out. Re-eliminate one block at a
time and retain the updated shared conditionals. The live poses and unknown
planes stay variables. This preserves their correlations and avoids a fixed
recovered-pose or fitted-plane observation. No epsilon prior is introduced.

The original **664** support queries need up to **1,444** coordinates if assembled
together; **187** exceed the existing 1,024-column limit. Sequential fronts need
at most **304**. A dense prototype is rejected for that reason. An initial
large-row projection also creates roughly `4e-10` of spurious information in a
zero-information control. Propagating only the new likelihood yields exactly
zero new live information in the final controls. Both rejected sources and
their receipts remain here.

Ten source controls compare information, gradient, full residual cost, mean and
covariance against independent dense or analytic solutions. They cover coupled
finite-cone/fan rows, pressure, unknown planes, repeated late updates, shared
ancestors, inertial-state downgrade, null directions, atomic rejection and
extreme precision. A **200-state / 1,200-coordinate** chain needs only a
**12-column** front and matches an independent Gaussian update.

Two original native conditional closures are numerically refactored at
298/304 columns. These are explicitly **null-information algebra controls**:
the old record has no complete live information matrix, so a zero-information
root and zero synthetic rows are used. They add no physical range measurement.
Pose-mode differences stay below `4e-16`; covariance differences stay below
`7e-17`. Matrix reservations are 5.70/5.75 MB, excluding NumPy/LAPACK allocator
scratch and caller-owned arrays. Timings are retained; native throughput is
unqualified. The fixed live-window, archive and memory limits are unchanged.

Final independent source and outside-installed wheel each run **487** cases:
**471 pass, 16 optional legacy cases skip**. Protected user edits are excluded.
The incomplete first source export and oversized test-prior fixture are retained
as test-setup failures. No dependency or native binary changes accompany this
reference implementation.

Native owner installation, source-correct active/marginal row ownership,
finite physical support, echo correspondence, range-noise calibration and useful
map geometry remain open. The existing 102 point hypotheses and forty larger
prediction errors remain in the
[original paired-data assessment](../sonar-low-altitude-15m-20261029/README.md).

From the POSIM-SLAM checkout, replay the audit without application sources:

```sh
PYTHONPATH=src python tools/audit_delayed_surface_width.py \
  --packets docs/diagnostics/retired-pose-conditionals-20261029/archive-packets.json.gz \
  --requests docs/diagnostics/joint-surface-ranges-20261029/surface-pose-retention.json \
  --null-rows 67 69 --output /tmp/delayed-surface-audit.json
python docs/diagnostics/delayed-surface-update-20261029/verify.py
```

한국어: 과거 자세의 조건부 관계와 현재 자세·표면의 상관관계를 함께 보존하는
지연 거리 갱신 참조 구현입니다. 원본 의존 관계를 작은 블록으로 순서대로 처리하고,
같은 관측의 재사용과 결과 일부만 적용하는 실수를 검사합니다. 네이티브 추정기 적용,
실제 소나 거리 융합, 처리 주기와 지도 정확도의 합격은 아직 남아 있습니다.
