# Sensor-local acoustic forward-model control

[한국어](README.ko.md)

The public NumPy `SourceSurfaceObservation` evaluates finite, opaque surfaces
supplied in the sonar frame. It preserves first-hit visibility, float32 range
grids, POSIM's range-derived ray normals, exact sampled Hamming responses and
conditional pixel correlation. Reflectivity selects source power independently
of geometric occlusion. It does not estimate a surface or add a native factor.

## Source and scope

The reference is POSIM `ffecee9be9976f68c9e883d6cf3bbd6d40eb7729`, a separate
current-main integration containing #6 and #31–36. Those PRs remain unmerged.
`RaySurfaceNormals.hh` already returns its third component as ray incidence.
Reading `normal.z` in the acoustic shader is consistent with this convention.
No acoustic shader, installed SDK, ABI, noise, sensor default or admission rule
is changed by this control. The [Apache-2.0 attribution](https://github.com/IOES-Lab/POSIM-SLAM/blob/codex/reference-patch-tracking/THIRD_PARTY.md#posim-acoustic-model-adaptation--posim-음향-모델-이식)
identifies the adapted source.

The utility retains 1.5° angular support, at most 25 range-derived neighbours,
nearest one-sided tangents at discontinuities and the source's conditional
local plane fit. The source computes float32 sine/cosine before promoting to
double geometry. An initial double-trigonometry adaptation failed the original
parity protocol; its complete report remains in
`normal-reference-first-comparison.json`. Preserving source precision repairs
that discrepancy with the original tolerance unchanged.

Ten complete mathematical range grids include a dense 256×256 finite occluder,
planes, spherical range, missing/isolated returns, nonuniform angles and small
fans. The unchanged C++ helper runs against existing OpenCV 4.13.0. Every
zero/nonzero normal mask agrees; corrected maximum component difference is
**3.13×10⁻⁶**, within the predeclared **2×10⁻⁵** source-parity tolerance.
`native-normals.npz` retains every input and native output. This is execution of
the actual native normal function on supplied arrays, not a Gazebo acquisition
or hardware calibration.

## Conditional power moments

`SourceSurfaceObservation` traces the supplied sensor-local triangles, runs the
range-normal adaptation and reuses existing `sonar_speckle` Fourier amplitudes
and moments. It exposes no ideal-normal geometry derivative. Near-clip
intersections are explicitly unsupported. Gaussian diffuse-source moments give
`K = H diag(q) H*`, mean `diag(K)` and power covariance `abs(K)²`. Pixel
averages retain this covariance; pixels are not assumed independent.

The supplied finite-ground/box control uses the 15 m development profile:
256 beams, 256 vertical rays, 599 bins and 29,900 Hz bandwidth. Both 16 and 64
predeclared pixel queries agree with the separate full-FFT operator to relative
L2 error below **1.6×10⁻¹⁴** for means and covariance. One process measured
0.232/3.774 s for existing analytic queries and 2.135/2.589 s for full FFT,
respectively. These are local timings, not a stable speed ranking or native
throughput. No new Fourier approximation is implemented. The ideal-normal
mean differs by 0.0168%/0.802% on these query sets; that is model-fidelity
evidence, not an accuracy improvement on a recorded map.

## Remaining measurement gate

Geometry and reflectivity are supplied hypotheses. Actual rasterization,
multiple paths, source finite-precision/RNG effects and native/hardware
calibration remain outside this operator. Conditional source moments are not
landmark/pose covariance or a calibrated image likelihood. No navigation/DVL
mean is used as independent sonar information. DVL range uncertainty remains
unknown. The earlier [translated-image boundary failure](../sonar-translated-shadow-20261031/README.md)
and complete maps remain unchanged.

Next, qualify original-input power/visibility predictions, nuisance reflectivity,
unresolved elevation and correlated uncertainty before fitting or native fusion.
This already inspected scene is development evidence. Independent scenes and
actual DVL-loss/absence benefit remain required for useful mapping claims.

## Reproduction

Install POSIM-SLAM with NumPy and run the portable check:

```sh
python docs/diagnostics/sonar-source-local-power-20261031/verify.py
```

It checks every saved normal and recomputes both complete moment controls.
`run_controls.py --source POSIM_SOURCE --sdk OPENCV_SDK --output NEW_DIRECTORY`
reexecutes the unchanged C++ reference with the recorded Apple Silicon/OpenCV
layout. It needs a compiler and existing SDK; it performs no download or SDK
mutation. `normal_reference.cc` and `benchmark_moments.py` preserve the reference
and benchmark procedures. The [protocol](protocol.json) predates the new
measurements; the hash manifest binds complete inputs, outputs and public code.
