# Exact native sonar input recording: preflight controls

[한국어](README.ko.md)

The selected acoustic base is POSIM `ffecee9b`, containing the original #6 and
#31–36 heads. The separate diagnostic source adds opt-in bounded recording at
the actual WGPU call, with render-start/callback node poses and the captured
stamp carried with the cloned depth snapshot. Original PRs remain unmerged.
The source and binary identities are in [source-identity.json](source-identity.json).

The linked production adapter calls its kernel directly inside the same image.
This recorder therefore observes the call explicitly instead of assuming an
external interposer captured it. [Native binding evidence](native-call-binding.json)
and the [source snapshot](source/sonar_native_trace.hh) remain available.

## Complete byte controls

The existing assessment-only reference grid is reused without copying original
field recordings. Its poses are synthetic replay metadata, explicitly labeled
`offline_supplied_grid`; these are **not actual renderer captures**.

- Six complete 256 × 599 complex images are byte-identical with recording off
  and on. Fresh calls to the retained unmodified native library match all six
  complete images, not only selected pixels or mean powers.
- All five captured input tensors and all ten float32 call parameters match
  the supplied input bytes. Dimensions, frame IDs, seed and metadata also match.
- Exactly four records retain **11,210,144 uncompressed bytes** in total. Every
  complex output value matches the independently computed image hash. Gzip
  preserves complete original record bytes and footer hashes.
- Sixteen isolated C++ cases cover disabled/missing context, quotas, invalid
  configuration, oversized input, reversed timestamps, concurrent calls,
  existing-file preservation, null output, propagated compute exceptions and
  nonfinite IEEE values. Three malformed-record controls are rejected.
- The actual Release-mode CMake test `bounded_native_trace` passes. Original
  compute arguments, return pointer, exceptions and thread-local scope remain
  intact. An exception closes an incomplete diagnostic without admitting it.

[All native comparisons](native-verification-final.json),
[unmodified-library comparison](unmodified-native-comparison.json),
[compiled source identity](build-receipt-native-final.json).

## Independent verification

With NumPy, run:

```sh
python verify.py
```

The only shared input is
`../sonar-reflector-birth-20261031/native-reference/inputs.npz` at its frozen
hash. No WWW-POSIM module is imported. To compile and run all standard-C++17
controls with an available `c++` compiler:

```sh
python verify.py --native-unit
```

Native acquisition helpers are retained for optional platform-specific rebuilds.
They require the selected public POSIM source, an already qualified native
dependency build and its CMake link inputs. Installed SDKs are read-only inputs.
Native libraries and SDK archives are not copied into this portable record.

## Next operational gate

Enable the diagnostic only for a bounded real capture, then join every complete
complex output to its published raw image and captured timestamp. Compare actual
renderer ranges with analytic scene first hits at the recorded rendering-node
poses. Separate angular/raster sampling, pose/time and normal sensitivity before
changing the acoustic producer or adding another map factor.

No new world, renderer, SDK replacement, native pose update or map admission is
performed by these replay controls. The previous false reflector and unresolved
field-image disagreement remain in the [complete failed-map record](../sonar-reflector-birth-20261031/README.md).
Initialization, useful mapping, actual DVL loss/absence, installed-platform,
VRX and subsequent WAM-V surveying requirements remain open.
