# Known-plane DVL return-direction calibration

[한국어](README.ko.md)

This record tests a source-profile DVL against a known horizontal plane. It
isolates the range forward model from navigation, sonar features and map
association. The fixture has no SLAM trajectory or reconstructed cloud.

![Complete static-plane comparison](comparison.png)

The plot includes every static configuration. The first three configurations
supply only their first packet for calibration: 12 ranges in total. The other
300 ranges at those configurations are repeat measurements, not additional
independent calibration information. Six larger-distance configurations supply
624 unused ranges. A second native run supplies another 624 ranges at six
previously unused orientations/distances, after the directions were frozen.
All 390 original packets and 1,560 beam ranges are retained.

## Method and observations

An ideal continuous nearest-cone/plane intersection differs from the rendered
ranges by up to 27.33 mm in the reference configurations. An independent nominal
GPU-grid interpretation also fails, with a maximum difference of 28.81 mm.
Its original predictions and sampling code remain in the record as an
unqualified control. It does not establish the installed renderer's sample
positions or the identity of a return on a nonplanar surface.

For a known plane normal `n` and perpendicular distance `H`, a fixed unit return
direction `u` predicts `r = H / (n·u)`. The independent
`posim_slam.range_direction.calibrate_return_directions` helper solves linear
least squares for `H/r`, then normalizes each direction. It changes no emitted
range or physical sensor setting. The frozen means give a maximum residual of
12.04 µm on the unused-distance controls and 7.20 µm on the separately acquired
unused-orientation controls. These values measure static forward-model
agreement. They do not measure hardware precision, stochastic range noise,
SLAM pose uncertainty or map accuracy.

The helper accepts an explicitly supplied full covariance of `H/r`, in
beam-major order. It propagates cross-plane and cross-beam correlation through
the linear solution and unit normalization. This is a first-order conditional
calibration, with known reference geometry. Unknown uncertainty stays
unavailable. Synthetic covariance controls check derivatives, coordinate
changes and preserved cross-beam terms; they do not supply real measurement
uncertainty.

## Sensor and acquisition scope

- Four beams: 30° tilt, 2° aperture, rotations 45°/135°/−45°/−135°.
- Range: 0.1–20 m; resolution setting: 0.01 m at 1 m; rate: 1 Hz.
- Declared Gaussian velocity noise: 0.002 m/s. This is not range noise.
- Reference heights: 0.75, 2 and 4 m; roll/pitch: 0°/0°, 0°/8° and −7°/5°.
- Separate unused configurations: 1.3 m at 8°/0°, 0°/−8°, 5°/−5°, −5°/−5°;
  3.3 m at 4°/−3° and −4°/3°.
- Known static plane, identity fixture mount, original DVL intrinsics and noise
  element; no vehicle mission or camera/sonar capture. Each process exits
  normally without forced termination.

The installed package reports gz-sensors 10.1.1. Exact loaded DVL, sensor and
renderer binary hashes are in `fixtures.json`. The installed build's complete
source patch identity is not established by its package version alone. The
upstream [DVL source](https://github.com/gazebosim/gz-sensors/blob/gz-sensors10_10.1.1/src/DopplerVelocityLog.cc)
and [API header](https://github.com/gazebosim/gz-sensors/blob/gz-sensors10_10.1.1/include/gz/sensors/DopplerVelocityLog.hh)
describe sampling resolution and velocity noise. Bottom-target/beam messages in
this capture supply range means without range variance. Neither the resolution
setting nor velocity covariance is substituted for missing range uncertainty.

The first unused-orientation fixture records zero packets because its XML
writer emits `ns0:type` where the custom sensor parser requires `gz:type`.
`fixture-setup-failure.json` preserves that failure. Correcting the namespace
prefix retains the same expanded XML geometry/settings and frozen directions.
No product-parser or sensor-model patch is inferred from this fixture error.

## Reproduction and remaining gate

From a POSIM-SLAM checkout, install the library with NumPy and run:

```sh
python docs/diagnostics/dvl-plane-calibration-20261031/verify.py
python -m unittest discover -s tests -p test_range_direction.py
```

The verifier checks recorded hashes, every original mean, the 12 calibration
inputs, all continuous/nominal reference predictions, and every frozen
unused-plane prediction. The serialized protobuf SHA values are acquisition
receipts; wire payloads are not included, so they cannot be regenerated from
JSON alone. The complete JSON data use deterministic gzip storage;
`data-identities.json` also preserves each uncompressed byte hash. `render.py`
regenerates the plot with optional Matplotlib.

Live fusion remains disabled for this calibration. A moving, nonplanar
footprint may switch its nearest return. Physical return support and joint
range/plane/pose uncertainty still require validation before using these means
as SLAM constraints. The [matched field/complete-cloud record](../sonar-close-view-20261031/README.md)
and [conditional shadow record](../sonar-shadow-support-20261031/README.md)
retain the actual failed maps and narrow-view limitation. The next operating
test needs lateral translation that keeps the target and shadow visible.

The calibration code and record are Apache-2.0. The nominal-grid control is an
independent implementation referencing OSRF's Apache-2.0 gz-sensors source.
