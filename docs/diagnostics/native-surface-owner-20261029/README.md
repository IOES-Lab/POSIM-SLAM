# Native joint marginal ownership

`SurfaceMarginalOwner` owns a complete square-root marginal prior over actual
Basalt states, unknown planes and the pressure datum. `adopt` transfers the
existing Basalt and pressure-prior information, then clears those legacy owners.
Active IMU, DVL and pressure observations remain in their original stores.

`snapshot` binds immutable information, current coordinates and owner revision.
`prepareLate` accepts only the new live likelihood rows, original reading keys,
source coordinates and the complete changed-conditional closure. New planes
start without prior information. `install` swaps the prepared prior, planes,
conditional archive and ledger after every validation/allocation succeeds.
The folded whole-system verification prior is not an installation argument.

The optional final argument of `optimizePingGroups` includes this joint prior
in the existing native LM cost, state/plane/pressure increments and rollback.
`marginalize` consumes only retiring IMU/DVL/pressure rows, keeps unknown planes
and datum correlations, and captures the same consumed Gaussian conditional.
An incoming seventeenth inertial state can retire into the unchanged sixteen-
state persistent budget. The archive rejects more than 256 nodes or 32 MiB;
stream retention/admission deadlines still need a production lifecycle policy.
This path requires camera-free ownership with no independent acoustic point or
Ping groups. Other estimator combinations retain their existing path.

Ten compiled component controls use actual Basalt state containers, marginal
rows, continuous-noise IMU intervals, DVL/pressure stores and the existing trial
and optimizer classes. They compare complete information, gradient, cost,
covariance and the optimized mode against independent dense/reference systems.
They cover adoption of pressure cross terms, retired-pose late ranges, a
persistent plane through a second retirement, retained versus consumed IMU,
DVL inertial downgrade, free pressure height, duplicate keys, incomplete or
invalid conditionals, stale snapshots, trial rejection and legacy duplication.
Two complete request/response traces are retained compressed.

Maximum differences are 6.11e-13 for information, 1.62e-13 for gradient,
2.84e-14 for squared residual, 1.23e-15 for covariance and 6.71e-10 for the
optimized mode. These synthetic linearization-coordinate differences are not
marine position or map accuracy. Five freshly compiled persistent-prior checks
and the actual DVL/pressure native controls also pass. Independent frozen source
and an outside-installed wheel each run 487 cases: 471 pass and sixteen optional
legacy native/decoder controls are skipped explicitly.

Run `python3 verify.py` to inspect this dated record. With the exact dependency
SDK, build CMake target `basalt_surface_owner_probe`, then run
`POSIM_SLAM_SURFACE_OWNER_PROBE=/absolute/basalt_surface_owner_probe python
native/basalt/test_surface_owner.py`. `tools/build_basalt.py` includes that
qualification in a research build. Recorded compiler commands reuse existing
pinned Basalt objects and SDK libraries; this is not a fresh SDK rebuild.

The streaming frontend is not connected to this owner yet. It needs an
increment-only reference interface bound to each native snapshot: the existing
`prepare_delayed_ranges` whole-system verification digest cannot simply be
reused after native optimization/retirement. Do not overwrite its private
digest or install a folded prior alongside active observations. Native transport,
bounded archive expiry and original 15 m physical-input replay remain required,
followed by support/echo/noise and useful-map qualification. No sensor record,
installed SDK, live worker, default mode or accepted physical map changes here.
