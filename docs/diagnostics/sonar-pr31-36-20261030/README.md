# Sonar PR #31–36: source integration and replacement input

[한국어](README.ko.md)

The selected acoustic source is `7afd71e7b7a1a13eca8c4b8deb9c6e8461285cd5`
on `codex/sonar-slam-source-20261030`. It combines POSIM main `8c47601`, #6 and
all six follow-ups. No follow-up is merged into #6 or remote main.
The application builds this source directly, with an empty patch map.

| PR | Selected head | Responsibility |
| --- | --- | --- |
| [#6](https://github.com/IOES-Lab/POSIM/pull/6) | `d51d7b85` | Acoustic visibility, scan geometry, ray normals, metric fan and per-pixel black floor |
| [#31](https://github.com/IOES-Lab/POSIM/pull/31) | `7dc96021` | Single creation and mounted pose |
| [#32](https://github.com/IOES-Lab/POSIM/pull/32) | `f1055f48` | Context ownership and captured simulation timestamp |
| [#33](https://github.com/IOES-Lab/POSIM/pull/33) | `5f327fd5` | Partial matrix tiles |
| [#34](https://github.com/IOES-Lab/POSIM/pull/34) | `3421795d` | Original-length FFT, frequency window and locked Cargo build |
| [#35](https://github.com/IOES-Lab/POSIM/pull/35) | `02f6dccd` | Empty-frame preview fast path and paired recovery regression |
| [#36](https://github.com/IOES-Lab/POSIM/pull/36) | `fbfd000d` | Physical range endpoint, CPU binning and outer display cell |

Original authors/history remain in the source branches. #35/#36 tests were
updated on their own fork branches; the integration is separate. Both exact
heads pass Code Formatting Lint. That workflow is not a simulation test.

## Empty frames and range endpoints

Current #6 already blacks individual zero/non-finite pixels. #35 adds a
whole-empty-frame short circuit and tests paired raw/image recovery. The older
false-white reproduction at `a1786726` remains historical evidence; this record
does not assert that current #6 still has that failure.

The updated test selects Metal/headless rendering explicitly, distinguishes
metric previews from legacy beam rasters, uses published simulation timestamps
as scene-change fences, and uses volatile sensor-data QoS. An initial current-base
attempt with incompatible transient-local durability receives no raw samples;
its failure is retained. No deadlines or sample-count assertions are relaxed.

CPU and WGPU/Metal each pass **125/125 unique timestamp-matched pairs**, including
three empty phases and positive-return recovery. The fixture has 32 beams,
400 ranges and an 801 × 401 metric preview. Each process exits normally.
These runs use `11a2b6c4`; its production source matches final `7afd71e7`
(the later merge changes only the test). They do not close historical
full-resolution transport failures.

For sound speed 1500 m/s, bandwidth 29.9 kHz and maximum range 15 m,
`dr = c/(2B)` is 0.025083612 m and `ceil(2RB/c)+1` is **599 samples**.
The earlier 598-sample record ends at 14.974916 m; it is preserved under its old
source identity. Consumers read the actual range vector and dimensions. No
synthetic endpoint is appended to recorded input.

Range-sampling and ray-normal CTests pass. The source CPU regression now covers
108 point conditions × 17 beams, including 15 m. The actual final production
WGPU operator, verified with `dladdr`, passes **18/18** single-reflector
conditions at N=599; N=598 fails **6/18**, with near-maximum echoes wrapping to
zero. Both counts use matched physical tensors, seeds and frame index, with
their respective Hamming windows. All 36 complex arrays remain in
[endpoint-ffi-all.npz](endpoint-ffi-all.npz). The criterion `dr/2 + 1e-5 m`
tests the sampling grid; it is not SLAM noise or a marine accuracy threshold.

Run the portable saved-output verifier with NumPy:

```sh
python verify.py
```

## Actual Orca5 terrain capture

![Actual synchronized RGB and acoustic input, with sensor mounting](sonar-input-comparison.png)

This native Gazebo record enables fog/absorption, particles, sunlight/caustics
and waves. Optical visibility is 2 m, defined by 1% scene transmission at that
distance. The reference RGB camera uses the sonar's exact link/pose and 120°
horizontal FOV; the vehicle camera keeps its distinct original mount. RGB cannot
resolve the distant seabed in this view. Sonar axes are metres, with far upward.
The image is range/azimuth intensity, not a 3D cloud.

The figure is the first sonar frame at least 20 s after mission start with both
reference/isometric RGB within 50 ms: sonar 28.01 s, RGB 28.00 s. Across the entire
record, 40 reference pairs have identical stamps and 55 differ by 10 ms. Timestamps
are not rewritten. The reference camera and truth are assessment-only; no sonar
or VIO estimator is started. Mission control uses simulated Gazebo external
navigation, so completing this route does not establish SLAM navigation.

- 95 sonar frames × 599 ranges × 256 beams; 239 vehicle RGB frames.
- 4,763 IMU samples, 476 pressure samples and 47 DVL samples.
- All 47 DVL samples report valid bottom lock with four locked beams; Ping is
  physically absent. This is not a bottom-lock loss/recovery test.
- 96 sonar-reference RGB, 48 isometric RGB and 1,191 assessment-only poses.
- Every sonar payload hash, shape, range vector, monotonic timestamp and
  <=50 ms truth bracket passes the full-input audit. Maximum sonar stamp gap
  is 0.51 s at the requested 2 Hz.
- The route completes at commanded depths 1.7/2.2 m, with 1.372 m minimum planned
  terrain clearance. All four owned process groups are empty; normal exit,
  no forced kill. LIVE and Actions runners are untouched.

The terrain is the actual 60 × 60 m, scale-three Orca5 demo terrain, not the earlier
box-only field. Its deepest mission leg initially fails the unchanged terrain
planner. The revised operating depths preserve XY/yaw and the 0.5 m depth change;
no terrain, safety margin, sensor noise or map admission is weakened.

Preserved preflight failures cover missing canonical package-helper staging,
incorrect equal-mount assumptions, an XML namespace loss in the private capture
writer, the old mission hold-time schema, the unsafe depth and a box-only
assessor applied to terrain. The corrected assessor delegates registered terrain
to the existing terrain planner, checks additional boxes and rejects unknown
meshes. Four positive/negative controls pass. The first offline hash check also
confused ROS payload and NPZ-container hashes; it is corrected against the
collector's original payload definition without changing the data.

## Scope and next gate

The source-bound provider is rebuilt without SDK replacement or new provider
APIs. The trial explicitly reuses the source-matching renderer cache and external
Gazebo renderer/loader dependency controls. It does not qualify a released SDK,
installed application, other native platforms or a useful SLAM map. POSIM-SLAM
algorithms and versions are unchanged by this input/diagnostic update.

Use this 599-sample record for physical reflector persistence and range-response
likelihood checks. Retain the previous 598-sample regression inputs. Do not freeze
speckle, duplicate already-applied window corrections, trim clouds or relax
uncertainty/freshness/admission to make a method pass. Useful complete maps,
DVL loss/absence, installed operation, VRX and WAM-V surveying remain open.

Bounded metadata, complete operator outputs and the figure are mirrored in
WWW-POSIM and POSIM-SLAM. Canonical raw inputs and all failed/preflight/native
logs remain once at
`/Volumes/ExternalHDD/POSIM/validation-data/posim-slam/sonar-pr35-36-20261030`.
No GitHub Artifact download is used.
