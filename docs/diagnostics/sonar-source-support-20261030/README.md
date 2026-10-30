# Sonar source support and separated-view geometry

[한국어](README.ko.md)

This record contains complete **unqualified** reconstruction diagnostics from
the [wall/corner input](../sonar-wall-corner-20261030/README.md). No method here
installs sonar factors, admits a map or changes production defaults. The sensor
combination is sonar + IMU + pressure + DVL, without single Ping or camera input.

`BeamPhaseBounds.query_image()` applies the existing conditional magnitude
bounds to every range/beam cell. It bounds its multiplication temporary to
8 MiB. Unknown phases, range sidelobes, elevation and physical reflector
identity remain unresolved; range rows and source bounds are not independent
probabilistic observations. Zero lower bound means unresolved contribution.
The response and amplitude-error intervals belong to this simulator profile,
not a hardware calibration certificate.

## Complete controls

- Angular source bounds retain 4,014 positive-source samples out of 26,802
  image-edge samples. Their distance to any visible return along the same fan
  has p90 3.76 cm; the full set has p90 44.54 cm. This is a conditional subset
  comparison. The positive subset still has global-first-return p90 257.16 cm.
- Curve-only fits have 509 small held-out-beam residuals but 368 mirrored
  solutions. Other-view prediction has median/p90 RMS 14.93/141.66 cm. All
  2,494 conditional model intersections remain, including 1,033 beyond 10 cm
  from the nearest assessed surface. The failed curve code and ten controls
  remain here, outside the installed API.
- An all-frame nominal reconstruction has 353 candidates with 1.50 cm surface
  RMSE, all near one wall. It lacks pose uncertainty and unused validation
  frames, and uses 20,104 assignments for 8,378 distinct timestamp/pixel keys.
- The bounded 32.50–52.50 s native snapshot supplies **joint** pose means and
  covariance. Construction uses 27 frames through 45.50 s; fourteen later
  frames are validation-only. All 1,714 support-selected candidates remain:
  surface RMSE 10.33 cm, p90 17.50 cm, maximum 42.92 cm; 574 exceed 10 cm.
  Median validation-image support is 16.7%. Missing support is unresolved,
  not a free-space measurement.
- A second split uses alternating two-frame blocks. It retains all 1,852
  support-selected candidates, with 10.19 cm surface RMSE and 546 beyond
  10 cm. Median validation-image support is 30.0%.
- Up to sixteen equispaced positive training frames supply the declared
  likelihood rows per candidate; other frames are support checks. The
  unchanged diagnostic scales are 0.0250836 m and 0.00821331 rad. Correlated
  pose uncertainty gives best position 3σ 19.45/19.72 cm for the two splits;
  neither split has a candidate within the existing 10 cm diagnostic bound.
  Association, elevation branches and response-calibration uncertainty remain
  outside this local Gaussian calculation. Treating poses as independent gives
  different, larger uncertainty in these cases; correlation cannot be guessed.
- Greedy, training-only disjoint pixel-support subsets contain 391/399
  candidates. They remain unqualified; this bookkeeping does not establish
  point identity or make the underlying distributed echoes single-scatterer.

## Matched comparison

![Actual RGB and sonar, field context and every unqualified candidate](comparison-heldout.png)

RGB and sonar share the 35.00 s exposure, origin and 20.1° downward pitch.
Orange outlines the sonar's ±10° elevation fan. The acoustic image uses metric
distance, with near at the bottom. Green marks positive source lower bounds.
The simulated sonar uses 15 m range, 256 horizontal beams and 256 vertical rays,
with 120° × 20° field of view. RGB has 1% PBR transmission at 2 m, particles,
sunlight/caustics and a wave surface. The separate field reference disables
haze/absorption and preserves the body pose and other effects; wave phase differs.
The cloud uses its exact camera pose/FOV/perspective, fixed blue-to-red Turbo
height limits and 64 pt² markers. All 1,714 candidates are in view; zero are
qualified map points. Grey geometry and truth are assessment-only inputs.

## Reproduction and limits

From the independent library root, using Python 3.10+ with NumPy:

```sh
PYTHONPATH=src python3 docs/diagnostics/sonar-source-support-20261030/verify.py
PYTHONPATH=src python3 docs/diagnostics/sonar-source-support-20261030/test_curve_model.py
```

The verifier recreates both training selections and pose marginals, compares
every output array, intervenes on held-out images to check construction leakage,
checks disjoint support and reassesses all candidate geometry. It uses public
library code and prepared numeric input only. It does not rerun Gazebo or prove
hardware/model calibration, physical correspondence, live throughput or useful
map coverage. The 30-second full-pose query and an attempted block query exceed
the unchanged native recovery workspace; the complete failed logs stay with
the application controls. The evaluated twenty-second query stays within it.

Raw camera/sonar/IMU/DVL/pressure remains once in the external canonical record.
`data/` contains derived source masks, source bounds and a same-snapshot pose
prior; `results/` retains every candidate and diagnostic subset; `legacy/`
preserves the failed alternative models. Private acquisition, rendering and
replay implementation is not part of this library record. Frozen source and
an outside-installed wheel each run 526 tests without failure, with sixteen
explicit optional skips. The native implementation is unchanged.

Next compare sensor-only correspondence and degeneracy-aware sonar-pair motion
constraints before further optimizer expansion or point promotion. The useful
3D map, navigation benefit, recovery and package requirements remain open.
