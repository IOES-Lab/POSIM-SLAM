# Retired-pose Gaussian information / 과거 자세의 조건부 정보

The actual native marginalization now has an optional read-only capture of the
retiring coordinates' Gaussian conditional. It keeps fixed owner references,
regression on retained states/pressure, observable noise and unbounded directions.
The same undamped consumed IMU/DVL/pressure rows supply both the installed prior
and this conditional. No independent pose observation is added. The diagnostic
mode rejects visual landmarks, acoustic point tracks and local-plane groups.
Default capture is off; the installed SDK worker is unchanged.

On the original 15 m low-altitude record, all 178 published state rows are
byte-identical across the old worker, the new worker with capture off and capture
on. All 179 output events agree after removing the added archive field. Estimator
calibration is identical apart from the two explicit capture/epoch settings.
The archive contains 163 conditionals. Its marginal information/gradient relative
differences from the actual installed prior are at most 7.59e-16 / 3.74e-10.

Every one of the 664 existing range predictions recovers all original source
poses jointly, including the 356 retired sonar-pose predictions. Shared ancestor
noise preserves cross-pose covariance; a missing/null direction is never a
zero-variance observation. Recovery uses a bounded 30 s diagnostic query horizon
with explicit protection of admitted boundary poses and separator closure.
Peak stored matrices are 2,381,280 bytes (82 nodes), with 5,987,520 bytes of cached
recovery matrices. This is one recorded workload, not a real-time benchmark.

The recovered covariance **conditions on the live coordinates being known**.
It excludes their uncertainty and is not a navigation confidence or calibrated
joint map covariance. Native late-factor installation, posterior conditional
updates, finite physical support, echo identity and range-noise calibration
remain required. A late factor must update the complete correlated prior and
conditionals atomically before it may affect the live solution. No new map or
sensor capture is claimed. All earlier 102 point hypotheses and forty large
range-prediction errors remain in the original record.

Nine new source controls and two new native controls pass, alongside three
affected native pressure controls. Source and independently installed wheel each
run 477 cases: 461 pass and sixteen optional legacy checks are explicitly skipped.
Native/reference full-rank and rank-deficient splits include zero-row/zero-column
endpoints, shared noise, state 15-to-6 downgrade, expiry/protection, exact reference,
epoch, missing separator, duplicate ownership and capacity rejection. The first
test invocation and fixture-shape failures remain in the compressed logs; both
were corrected without changing estimator thresholds.

실제 네이티브 최적화에서 과거 자세를 제거하기 직전, 남은 자세·압력 기준값과
연결되는 조건부 식·잡음·관측 불가능한 방향을 보존합니다. 같은 15 m 입력의
178개 궤적 상태는 기존 바이너리 및 새 기능의 켜짐/꺼짐 사이에서 바이트까지
동일합니다. 지연 예측 664건 모두에서 원본 지원 자세를 함께 복원하며, 그중
356건은 이미 제거된 소나 자세를 포함합니다. 공유 잡음을 같은 원인으로
취급해 자세 사이의 상관을 유지합니다.

복원 공분산은 현재 자세를 알고 있다는 조건부 값입니다. 실제 항법 불확실성이나
지도 품질을 뜻하지 않습니다. 지연 관측을 최적화에 적용할 때의 전체 사전정보·
조건부 식 동시 갱신, 실제 에코 대응, 유한 표면 지지와 거리 잡음 보정은 남아
있습니다. 설치된 SDK와 기존 점군·오차 기록은 유지하며 새 지도 합격을 선언하지
않습니다. 검사 수, 한계, 재현 자료는 위 영문과 같습니다.

`python verify.py --source-tree /absolute/path/to/POSIM-SLAM` checks retained
source hashes, every compressed packet and all 664 joint recoveries. It needs
NumPy and this library. The 7.4 MB Gaussian stream is stored once per evidence
location in compressed form; the 97 MB original sensor dataset is not copied.
The field replay driver stays in the private application; the public native
control and compressed-packet verifier run independently of that application.
The app mirrors the compact evidence and links the original RGB/sonar/cloud
comparison. Exact commands, source hashes, binary/wheel hashes and failure logs
are retained here. The native Basalt revision remains
`0f3b2b52c807f70ff4e2973ce253c73329eea7bc`.
