# Pinned visual dependency build

`sources.json` identifies exact upstream Git revisions. `eigen-solvers.patch`
removes CSparse linkage and selects g2o Eigen solvers. All Eigen compilation uses
`EIGEN_MPL2_ONLY`; g2o LGPL/CHOLMOD/CSparse components and GPL apps are disabled.

From a POSIM-SLAM checkout, `python tools/build_visual.py --sources <checkouts>
--engine <matching-sdk> --output <new-provider>` builds the isolated macOS
profile. The source directory contains Git checkouts at the names in the pin
file, including FBoW/tinycolormap submodule checkouts. Source archives and patches
are retained in the build output. No installed engine or source checkout is
modified. A successful build/ABI load is not tracking or archive qualification.

Original notices are in `../notices`; the application distributing a compiled
provider must also retain the matching dependency source and SDK notices.
