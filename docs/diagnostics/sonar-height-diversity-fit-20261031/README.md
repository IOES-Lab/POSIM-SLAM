# Height-diverse views and scene-support controls

[한국어](README.ko.md)

![Same retained native camera and every model sample](full-cloud-comparison.png)

Adding the existing height-diverse views does **not** improve this fixed-pose,
one-prism/plane reconstruction. Starting from the preceding sealed fit, the
same four-sweep search schedule gives complete surface RMSE **5.32 cm** using
the original three views and **8.76 cm** using five views. The initial is
5.65 cm. Every state retains all **548 model surface samples**, including unseen
faces: [initial](initial_prism-complete.ply), [three-view control](three_view_control-complete.ply),
[five-view fit](height_diverse_fit-complete.ply). They are model samples, not
independently observed landmarks or a qualified SLAM map. No pose update occurs.

The five-view fit improves its mean composite score from 1846.007 to 1780.432
and the untouched temporal-view score from 1152.789 to 1121.144. The same-
schedule three-view control improves original-view score to 1005.652 but worsens
temporal score to 1168.634. Image-score improvement alone does not establish
geometry improvement. The bounded search does not establish a global optimum.

## Frozen operating-view comparison

![All eight original sonar images and fixed queries](raw-sonar-queries.png)

The [protocol](protocol.json) precedes raw inspection/optimization. Original
fitting indices 49/88/151 have only 0.18 m vertical diversity. Existing source-
matched frames 27/81 add about 1.31 m, selected from navigation z-extrema whose
sealed shape centre is predicted in the original fan. This is a shape-
conditioned visibility test, not verified reflector identity. The same initial
ten parameters, bounds and coordinate schedule are used in both fits; actual
wall time/evaluation count need not match when bounds skip proposals. All 165
evaluation calls and the 520.7 s execution remain in the trace/report.

The high view has 471 predicted object-hit rays and only 1.36% of selected mean
power assigned to the supplied object, versus 5,893 rays/62.10% in the low view.
These are model predictions, not measured object energy or information ranks.
Both original raw images are retained; the six earlier images are shared by
hash instead of copied again. Each view keeps its 48 frozen native queries and
all 599×256 original values. No noise, clipping, freshness, map-admission or
physical sensor setting changes.

## Why scene support matters

![Reference-support and pose comparisons after the fit was sealed](scene-support-comparison.png)

A separate [assessment-only forward audit](field-forward-audit.json) uses the
complete retained reference scene at three fixed views. At the **same navigation
poses**, normalized residual RMS changes from **5.048/3.557/3.042** for the sealed
finite fit to **1.594/1.202/1.787** for the complete reference scene. Gains change
from 28.07/2.50/4.93 to 1.24/1.11/1.81. Reference geometry never enters inference.
An additional full-scene/interpolated-true-pose arm retains 1.559/2.202/1.142
RMS. Original truth samples bracket each acquisition within a 40 ms interval;
translation/Slerp interpolation is explicit, without a fitted time offset.
These single-image controls do not certify hardware, rasterization, exact time
alignment or a calibrated image likelihood.

The adaptive [support ablation](scene-support-ablation.json) then holds pose,
uniform reflectivity, operator and queries fixed. At frames 27/81, reference
ground plus the same reference target gives scores 215.717/327.686, versus
336.111/378.984 with the fitted plane plus that target. At frame 88, ground plus
target scores **452.764**; the complete scene scores **358.840**. The wrong flat
background scores **417.134**, partly compensating for missing reflectors.
Surface removal changes first-hit occlusion; zero reflectivity is not used as
surface removal. The reference controls are chosen after sealing and remain
assessment-only, without independent-scene or universal-error claims.

This rejects height-extrema selection as a repair for this incomplete family.
Next, model measured local ground support and multiple reflectors together,
with unknown support and shared pose uncertainty, before another native factor.
Do not tune sensor noise or convert each sample to an independent XYZ reading.
All failed hypotheses and earlier maps remain; independent scenes and actual
DVL-loss/absence benefit still gate operational admission.

## Source, references and reproduction

The source remains POSIM **ffecee9b**, containing unmerged #6/#31–36, with the
15 m/599-bin/256-beam/256-vertical-ray profile. IMU, pressure and DVL are original;
Ping is absent. The explicit 35° inspection mount is unchanged. Actual estimator
RGB keeps 2 m visibility. The exact native context-camera projection/crop,
64 pt² markers, complete common Turbo heights and vehicle/sensor directions are
shared by all overview panels. Context RGB disables only haze/absorption;
lighting, caustics, particles and waves remain at another phase. Scene/RGB/pose
truth are assessment-only. No field capture, SDK mutation or GitHub Actions
Artifacts is needed.

The independent public producer reuses the preceding finite-surface search,
assessment and plotting helpers. Only the helper interfaces for variable
families/shared raw records change; prior defaults and scientific outputs stay
intact. With the library's `research,diagnostics` dependencies:

```sh
python docs/diagnostics/sonar-height-diversity-fit-20261031/verify.py
```

The verifier recomputes every final power/covariance, full PLY/assessment and
paired scene-support forward control, without rerunning optimization. New
`run_fit.py prepare`/`support`/`fit` commands require a fresh folder with producer
and protocol and accept `--records`/`--raw`; existing traces cannot be overwritten.
`assess`/`render` operate only after sealing. Native/source controls and installed
acceptance remain separate. No WWW-POSIM implementation is imported.

Scene/pose optimization is established research: [Qadri et al. 2023](https://www.cs.cmu.edu/~kaess/pub/Qadri23icra.pdf)
models implicit acoustic surfaces; [Lin et al. 2025](https://www.cs.cmu.edu/~kaess/pub/Lin25iros.html)
optimizes scene and poses together. This diagnostic claims no new paper method.
The [NeuSIS implementation](https://github.com/rpl-cmu/neusis) has an
[MIT license](https://github.com/rpl-cmu/neusis/blob/main/LICENSE); its published
training workflow uses an NVIDIA 3090. It is an offline baseline to assess, not
an installed dependency or a demonstrated live mapper here.
