# Visual patch diagnostics / 시각 패치 진단

These optional native targets inspect original gray8 images. They leave the
estimator, landmark archive and frontend selection unchanged. Build them from a
configured Basalt research build:

```sh
cmake --build BUILD --target basalt_patch_information_probe basalt_reference_photometry_probe basalt_reference_warp_probe --parallel 2
```

Each input packet is a UTF-8 JSON line followed immediately by packed gray8
image bytes, with no padding or separator. Output is one JSON line per packet.
Widths and heights are at most 1920×1080. Optional `processed_width` and
`processed_height` select the same OpenCV area resize and uint16 conversion as
the native camera boundary. Pixel coordinates refer to the processed image.

- `basalt_patch_information_probe`: header fields are `width`, `height` and
  `points`, each with `id` and `pixel: [x,y]`; one image follows. It evaluates
  the pinned Pattern51 normalized SE2 Jacobian, its Hessian, and translation
  information after eliminating rotation. The eigenvalue ratio describes
  directional conditioning. It is not a calibrated pixel covariance.
- `basalt_reference_photometry_probe CALIBRATION ARCHIVE`: calibration selects
  `optical_flow_type: "patch"`; archive JSON contains unique nonnegative `ids`
  (at most 10,000). Each header is `{"stamp_ns": 0}`, followed by one image at
  calibration resolution. Supply every original camera exposure, including
  pre-initialization images. Times start at zero or later and strictly increase.
  The tracker processes all features; only selected IDs appear in the output.
  Their affine transforms, normalized birth-reference RMSE and valid sample
  counts accompany the reference-cache size. No IMU or pose is estimated.
- `basalt_reference_warp_probe`: header fields are `width`, `height`, `id`,
  `birth: [x,y]`, initial target `pixel: [x,y]`, and initial `linear: [[a,b],[c,d]]`.
  The birth and target images follow in that order. Both local fits use the
  same fixed visible Pattern51 samples and gain normalization. Inverse
  compositional SE2 and six-parameter affine fits use SVD and decreasing-cost
  updates for at most 30 iterations. Output includes rank, accepted steps,
  RMSE, centre and linear warp. This is local refinement around a supplied
  match; it is not a complete multiscale affine tracker or a physical identity
  test. Black or unsupported references report `valid: false`.

Image alignment follows the established
[Baker–Matthews framework](https://www.ri.cmu.edu/pub_files/pub3/baker_simon_2004_1/baker_simon_2004_1.pdf).
The tools use the pinned Basalt image, gradient and sampling implementations.
Photometric residuals, Hessian conditioning and lower fitting costs do not
certify that a feature refers to one static physical point. Keep their evidence
separate from native estimator, ROS recovery, map and package acceptance.

## 한국어

선택형 네이티브 대상은 원본 gray8 영상을 검사합니다. 추정기·지도 보관·추적기
선택은 바꾸지 않으며 위 명령으로 별도 빌드합니다.

입력은 UTF-8 JSON 한 줄과 바로 이어지는 연속 gray8 영상 바이트입니다.
패딩이나 구분 바이트는 없습니다. 패킷마다 JSON 한 줄을 출력합니다. 영상은
최대 1920×1080이며 `processed_width`, `processed_height`를 지정하면 네이티브
카메라 경계와 같은 OpenCV 면적 축소·uint16 변환을 사용합니다. 좌표는 처리
영상 기준입니다.

- `basalt_patch_information_probe`: `width`, `height`, `id`·`pixel: [x,y]`를
  가진 `points`와 영상 한 장을 받습니다. 고정 Pattern51의 정규화 SE2
  Jacobian·Hessian, 회전을 소거한 위치 정보를 구합니다. 고유값 비율은 방향별
  조건을 나타내며 보정된 픽셀 공분산을 뜻하지 않습니다.
- `basalt_reference_photometry_probe CALIBRATION ARCHIVE`: 보정 파일은
  `optical_flow_type: "patch"`, 보관 JSON은 중복 없는 음이 아닌 `ids`를
  최대 10,000개 지정합니다. `{"stamp_ns": 0}` 뒤 보정 해상도의 영상 한 장을
  보냅니다. 초기화 전 영상을 포함한 모든 원본 노출을 원래 순서로 보내고,
  시각은 0 이상에서 증가해야 합니다. 모든 특징을 추적하되 선택한 ID의 변환,
  최초 참조와의 정규화 RMSE·유효 표본 수와 참조 수만 출력합니다. IMU·자세
  추정은 하지 않습니다.
- `basalt_reference_warp_probe`: `width`, `height`, `id`, 최초 좌표 `birth`,
  대상 초기 좌표 `pixel`, 초기 2×2 행렬 `linear` 뒤 최초·대상 영상 두 장을
  보냅니다. 동일한 고정 가시 표본과 밝기 배율 정규화를 사용하여 SE2와
  6변수 affine 국소 정합을 비교합니다. SVD와 비용이 감소하는 갱신을 최대
  30회 적용하고 계수·갱신 수·RMSE·중심·변환을 출력합니다. 주어진 대응점
  주변의 국소 검사이며 전체 다중 해상도 affine 추적기나 물리적 ID 검사는
  아닙니다. 검거나 지원되지 않는 참조는 `valid: false`를 보고합니다.

영상 정합은 위 Baker–Matthews 문헌의 확립된 방법을 따르며, 영상·기울기·표본은
고정 Basalt 구현을 사용합니다. 광도 잔차·Hessian 조건·낮은 정합 비용만으로
같은 정적 물리점을 본다고 보장할 수 없습니다. 추정기·ROS 복구·지도·배포
검증과 구분해 보존합니다.
