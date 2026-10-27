# Multiview image support / 다중 시점 영상의 관측 지지

`posim_slam.image_neighbors.image_neighbors` selects neighbouring images from
actual shared feature IDs and estimated landmark positions in one coordinate
frame. It uses the conventional OpenSfM rule: more than 20 common tracks with
parallax strictly between 3° and 30°, ranked by track count, with up to seven
neighbours. Camera timestamps break score ties. NumPy is its only dependency.
The Apache-2.0 implementation is independent of the OpenSfM native depth kernel.
It returns every supplied image ID, including images with no qualified neighbours.

`posim_slam.image_neighbors.image_neighbors`는 같은 좌표계의 실제 공통 특징 ID와
추정 랜드마크 위치로 인접 영상을 선택합니다. OpenSfM의 기존 규칙에 따라 시차가
3°–30°인 공통 특징이 20개를 초과하는 영상을 특징 수순으로 최대 7개 선택합니다.
동점이면 카메라 시각순으로 정렬합니다. 의존성은 NumPy이며 자체 Apache-2.0
구현입니다. OpenSfM 네이티브 깊이 커널과는 별개입니다. 조건을 만족하는 인접
영상이 없는 경우도 포함해 입력한 모든 영상 ID를 반환합니다.

Inputs are camera centres in metres, observed inlier track IDs at the exact
image times, and known estimated landmark positions. Unknown positions do not
contribute to the score. Missing image-time observations, non-finite geometry
and invalid policies raise `ValueError`. This utility constructs a view graph;
it does not estimate poses or depths, qualify pixel-level visibility or certify
a complete map. Use it with recorded or estimated sensor data, keeping simulator
geometry in the separate assessment process.

입력은 미터 단위 카메라 중심, 정확한 영상 시각의 인라이어 특징 ID, 추정된
랜드마크 위치입니다. 위치를 모르는 특징은 점수에 포함하지 않습니다. 영상 시각의
관측 누락·유효하지 않은 형상·잘못된 설정은 `ValueError`로 거부합니다. 이 함수는
뷰 그래프를 구성하며 자세·깊이를 추정하거나 픽셀 가시성·지도 완성도를 판정하지
않습니다. 시뮬레이터 형상은 별도 평가 과정에서만 사용합니다.

```python
from posim_slam.image_neighbors import image_neighbors

graph = image_neighbors(camera_centers, observed_inlier_ids, estimated_points)
for image_stamp_ns, neighbours in graph.items():
    matching_images = [item.view_id for item in neighbours]
    # Keep the reference image and its selected matching images together.
    # 기준 영상과 선택한 대응 영상을 함께 사용합니다.
```

Reconstruction inputs retain the observations needed for repeated views of the
same surface. A fixed temporal stride can remove that support even when a single
image shows the object clearly. The viewer's 1,000/2,000/5,000/all-point selection
controls rendering. Recorded observations and reconstructed map storage use
their own explicit resource policies. Sparse VIO landmarks and photometric
surface reconstruction remain separate outputs.

복원 입력에는 같은 표면을 여러 시점에서 관측하는 데 필요한 영상이 남아 있어야
합니다. 고정 시간 간격으로 영상을 줄이면 한 영상에 물체가 선명하게 보여도 이
관측 지지가 사라질 수 있습니다. 뷰어의 1,000·2,000·5,000·전체 선택은 표시량을
조절합니다. 기록 관측과 복원 지도 저장에는 별도의 명시적인 자원 정책을 적용합니다.
희소 VIO 랜드마크와 광도 기반 표면 복원은 별도 출력입니다.

Rule reference: [OpenSfM `dense.py`, revision 65571c8](https://github.com/mapillary/OpenSfM/blob/65571c8ae9ac6a1d0bcd3fdc1ea5fff97b1b5611/opensfm/dense.py).
The [recorded reference experiments](../validation/visual-dense-reference-20261027/README.md)
retain the native kernel's BSD-2-Clause notice and failed map cases. Geometric
unit controls and an exact comparison against the frozen 490-image recorded
graph verify this selector; native depth integration and installed-package
acceptance require separate evidence.

규칙은 위 OpenSfM 고정 리비전을 참고합니다. 기록 기반 실험은 커널의 BSD-2-Clause
고지와 실패 사례를 보존합니다. 기하 제어 사례와 고정된 실제 490영상 그래프의
정확한 일치를 확인합니다. 네이티브 깊이 복원 통합과 설치 패키지 검증은 별도입니다.

The full recorded-rate reference retains 5,023,301 points from 490 RGB images
with fixed RGB / IMU / DVL estimated poses. All three target roofs reach 100%
coverage within 5 cm, while a 29.75 m maximum surface error and missing wall
coverage leave the complete map unqualified. Its matching-view RGB comparison,
all-point statistics and estimated-landmark depth-range audit remain in the
[reference record](../validation/visual-dense-reference-20261027/README.md#full-recorded-image-rate--기록-영상-전체-주기).

기록 전체 주기의 기준실험은 RGB 영상 490개와 RGB·IMU·DVL 추정 자세를
고정해 5,023,301점을 보존합니다. 세 물체 상부의 5 cm 이내 복원 비율은
모두 100%이지만 최대 표면 오차 29.75 m와 누락된 벽 때문에 완전한 지도는
불합격입니다. 동일 시점 RGB 비교, 모든 점의 통계와 추정 랜드마크 깊이
범위 검사는 위 기록에 보존합니다.
