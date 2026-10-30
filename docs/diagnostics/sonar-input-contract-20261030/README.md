# Sonar zero-power and sampled-range contract

[한국어](README.ko.md)

This engineering record validates the consumer boundary for POSIM integration
`7afd71e7b7a1a13eca8c4b8deb9c6e8461285cd5`: current main plus #6 and
#31–36 on a separate branch. None of those PRs is merged into #6 or remote main.
The source ownership and 15 m Orca5 reference remain in the
[preceding record](../sonar-pr31-36-20261030/README.md).

Zero acoustic power is a valid `-inf` observation. The shared validator retains
it, still rejects NaN/positive infinity and malformed geometry/clocks, and never
rewrites the original array. Image filters use a separate bounded working copy;
candidate pixels must have an actual finite return at their native bin.
Quantitative edge, ground and shadow methods report unavailable noise evidence
instead of inventing a finite floor. A completely empty frame makes no acoustic
landmark; it still advances sensor receipt and permits valid IMU/DVL/pressure
processing.

The calibrated input uses the physical model's bandwidth B and sound speed c:
spacing c/(2B), sample count ceil(2RB/c)+1. The full range and bearing vectors
must match that grid. A 10 m / 29.9 kHz / 1500 m/s profile has 400 samples and
an actual last coordinate of 10.00836086 m. That coordinate is retained, while
physical point distance/FOV remain bounded by the configured sensor support.
A positive finite noisy measurement is separate from the latent point's
visibility. Python and native factors enforce that distinction. No range
epsilon, noise, freshness, map-admission or physical clipping limit is enlarged.
The application requires numerically equal ray and transform maximum-distance
tags, since an approximate equality can change the sample count at a ceil boundary.

## Recorded checks

- Original source CPU and WGPU/Metal regressions each pass 125 timestamp-matched
  raw/metric-preview pairs with normal server exit. The additional observer
  receives all 190 CPU and 192 WGPU arrivals, including 108/110 empty frames.
  Both feature frontends preserve the raw bytes and recover after returns resume.
- `inputs.npz` includes the complete 192-frame WGPU record and one original CPU
  snapshot from each of five phases, with original stamps and payload hashes.
  This is the 32-beam, 400-range, 900 kHz, 130° × 12° panel fixture; it is **not**
  the Sonoptix demo profile. The separate canonical 15 m/256-beam/599-range
  Orca5 record validates all 95 calibrated frames.
- All images, intensity/edge track traces, quantitative edge/ground/surface
  arrays and regions on those 95 entirely finite canonical frames are identical
  to library `c96dff1b9f40753684b39cf78e19168529f793fa`. This control does not
  claim full native-factor or map-output equivalence.
- Python: 543 tests, 521 passed and 22 optional skips in the SDK backend;
  the separate Python OpenCV 4.10 research subset passes 21/21. All three
  affected application checks and all seven native sonar-clock tests pass.
- A native before/after boundary control accepts 0/2 versus 2/2 legitimate
  noisy near/far samples. Both versions reject both physically out-of-range
  latent points. The final native geometry probe also passes.

![Actual acoustic loss and native auxiliary continuity](continuity.png)

The final isolated worker processes all 192 original-time WGPU arrivals with
**synthetic stationary IMU/DVL/pressure**, not measured vehicle motion or the
default physical auxiliary rates. After the required one-second IMU startup
history, it emits 188 states and acknowledges 187 DVL and 187 pressure
optimizations through the exact final 38.6 s input stamp, including the empty
interval. No qualified map or navigation result is produced. Public-client
shutdown requests SIGTERM (exit −15), without SIGKILL; threads are joined.
The installed SDK is unchanged. Worker SHA-256:
`38ec4ecc26aba87e55ee4abe60446196ed6bc6983fa335fb4461dac105e1e813`.

## Existing VIO regression qualification

The final source/worker reruns five existing original-time camera/IMU recovery
profiles: DVL transport loss, invalid bottom lock, pressure loss without Ping,
pressure loss with fused Ping, and pressure loss with receive-only Ping.
The independent library retains the historical combined-sensor controls;
application DVL profiles continue to omit Ping.

All five conditions pass. Ten full exports contain 14,063 vertices in total, including 27 above 10 cm. Invalid bottom lock keeps the same estimator/map frame; the returned export includes prior points. The other four source-matching reset/recovery profiles prepare a development-only provider descriptor. The invalid-lock continuity case is kept as a separate control and is not mislabeled as a new-frame recovery proof.

| Profile / phase | Points | Surface RMSE (cm) | Maximum (cm) | >10 cm |
| --- | ---: | ---: | ---: | ---: |
| dvl-transport / initial | 1425 | 1.137 | 15.738 | 3 |
| dvl-transport / recovered | 1459 | 1.197 | 12.233 | 1 |
| dvl-lock / initial | 1425 | 1.137 | 15.738 | 3 |
| dvl-lock / recovered | 2897 | 1.125 | 15.738 | 5 |
| no-ping / initial | 1246 | 1.769 | 16.701 | 3 |
| no-ping / recovered | 994 | 1.732 | 14.584 | 2 |
| ping-fused / initial | 1135 | 1.795 | 23.823 | 2 |
| ping-fused / recovered | 1142 | 1.313 | 14.670 | 3 |
| ping-received / initial | 1346 | 1.942 | 17.033 | 3 |
| ping-received / recovered | 994 | 1.732 | 14.584 | 2 |

Every emitted point is retained and assessed, with matching field/cloud
isometric overviews for initial and recovered segments in `vio/`. Points above
10 cm are reported, not deleted. That distance is a diagnostic cutoff and not
a universal marine accuracy criterion. These are VIO regressions, not evidence
of sonar mapping benefit. Full recordings and failed controls remain once on
ExternalHDD; the mirrored public record contains bounded inputs/results only.

## Reproduce and remaining gates

With NumPy and the matching POSIM-SLAM source installed, run `python verify.py`.
It checks file/payload hashes, every bundled calibrated frame, the saved native
continuity receipt and every complete-map assessment. `plot_continuity.py`
recreates the figure with Matplotlib. Final source hashes, worker identity and
test counts are in `source-identity-final.json`, `native-identity-final.json`
and `checks.json`. Input-only worker/results remain explicitly dated alongside
the final results; they are not substituted for final qualification.

Next establish physical reflector persistence and calibrated range-response
likelihood before adding more map factors. Useful sonar maps, physical
DVL-loss/absence benefit, general initialization uncertainty, installed and
platform acceptance, VRX completion and WAM-V surveying remain open. This is a
sampling/measurement engineering correction, with no academic novelty claim.
No GitHub Artifact download, LIVE/runner restart, installed SDK replacement or
new native provider ABI is involved.
