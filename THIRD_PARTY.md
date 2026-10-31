# Third-party code and notices / 제3자 코드와 고지

## Documentation imagery / 문서 이미지

Diagnostic RGB figures render the BlueROV vehicle and textured seafloor from
[Clyde McQueen's Orca5](https://github.com/clydemcqueen/orca5/tree/5f7cdc1f044926ebd56f2ce8a1fc4a82990cfa5e),
revision `5f7cdc1f044926ebd56f2ce8a1fc4a82990cfa5e`. The original
[MIT notice](docs/diagnostics/notices/orca5-MIT.txt) is retained verbatim.
Figures adapt the scene with a 3× horizontal terrain footprint, underwater
rendering effects and added physical test structures. These rendered images
do not include Orca5's SLAM executable or ORB-SLAM code.

진단 RGB 그림은 위 Orca5 리비전의 BlueROV 차량과 사진을 입힌 해저를 렌더링합니다.
원본 MIT 고지를 그대로 보존합니다. 지형의 수평 범위를 3배로 확장하고 수중 효과와
물리적 시험 구조물을 추가합니다. 그림에는 Orca5의 SLAM 실행 파일이나 ORB-SLAM
코드를 포함하지 않습니다.

Diagnostic height figures use the [Google Turbo colormap](https://gist.github.com/mikhailov-work/ee72ba4191942acecc03fe6da94fc73f),
Copyright 2019 Google LLC, author Anton Mikhailov, under Apache-2.0. The
[retained attribution and license](docs/diagnostics/notices/turbo-Apache-2.0.txt)
identify the original source revision. This notice also applies to the visual
depth-support comparison figures.

진단 높이 그림은 Google Turbo 색상표를 사용합니다. 저작권자·저자와 Apache-2.0
라이선스, 원본 소스 리비전을 위 고지 파일에 보존합니다. 시각 지도의 깊이 지지
비교 그림에도 같은 고지가 적용됩니다.

## POSIM acoustic-model adaptation / POSIM 음향 모델 이식

`sonar_source_normals.py` adapts
[POSIM RaySurfaceNormals.hh](https://github.com/IOES-Lab/POSIM/blob/ffecee9be9976f68c9e883d6cf3bbd6d40eb7729/gazebo/posim_gz_multibeam_sonar/multibeam_sonar/RaySurfaceNormals.hh),
revision `ffecee9be9976f68c9e883d6cf3bbd6d40eb7729`, under Apache-2.0.
The NumPy adaptation preserves the spherical range geometry, one-sided
tangents, 1.5° support and local plane-fit rule. The source SPDX notice and
[Apache-2.0 license](LICENSE) are retained. This utility adds no native POSIM or
OpenCV binary to the Python wheel.

`sonar_source_normals.py`는 위 POSIM 고정 리비전의 Apache-2.0 함수를 NumPy로
이식합니다. 구면 거리 격자, 한쪽 접선 선택, 1.5° 이웃 범위와 국소 평면 적합
규칙을 유지합니다. 원본 SPDX 고지와 Apache-2.0 라이선스를 보존합니다.
이 유틸리티는 Python wheel에 네이티브 POSIM·OpenCV 바이너리를 추가하지 않습니다.

## Estimator dependencies / 추정기 의존성

The offline [CPU depth reference](docs/validation/visual-dense-reference-20261027/README.md)
uses Mapillary OpenSfM `65571c8ae9ac6a1d0bcd3fdc1ea5fff97b1b5611`, under
[BSD-2-Clause](docs/diagnostics/notices/opensfm-BSD-2-Clause.txt).
Its native kernel is not distributed in the POSIM-SLAM wheel. The independent
`image_neighbors` NumPy utility follows its published shared-track/parallax
selection rule and uses original Apache-2.0 source.

오프라인 CPU 깊이 기준실험은 위 OpenSfM 고정 리비전과 BSD-2-Clause 고지를
사용합니다. 네이티브 커널은 POSIM-SLAM wheel에 포함하지 않습니다. 별도
`image_neighbors` 유틸리티는 공개된 공통 특징·시차 규칙을 자체 Apache-2.0
NumPy 코드로 구현합니다.

The [temporal-stereo reference](docs/validation/visual-temporal-stereo-20261027/README.md)
also uses OpenCV 4.13.0 rectification and StereoSGBM under the OpenCV Apache-2.0
notice below, followed by the same OpenSfM BSD-2-Clause depth cleaner. These
offline native reference tools are not included in the default wheel.

시간차 스테레오 기준실험은 아래 OpenCV 4.13.0의 Apache-2.0 고지와 같은
OpenSfM BSD-2-Clause 고지를 유지합니다. 오프라인 네이티브 기준 도구는
기본 wheel에 포함하지 않습니다.

POSIM-SLAM's original Python/C++ source is Apache-2.0. Dependencies retain the
licenses below. A license for the main estimator does not cover every optional
solver, vocabulary, dataset, model or runtime dependency.

자체 Python/C++ 코드는 Apache-2.0입니다. 의존성에는 아래 개별 라이선스가 적용됩니다.
주 추정기의 라이선스가 선택형 솔버·사전·데이터·모델·실행 의존성을 대신하지 않습니다.

| Component | Exact selected version/source | License and retained notice |
|---|---|---|
| Stella VSLAM | 0.7.0; `e445b5452535e781fdf6777ec33a06f5f8f5e416` | BSD-2-Clause: [original](src/posim_slam/data/notices/stella-original.txt), [fork](src/posim_slam/data/notices/stella-fork.txt) |
| g2o selected core/Eigen solver | `e8df2004e07ea8f5b8e6a8b9f2dc067b45b45036` | BSD-2-Clause: [notice](src/posim_slam/data/notices/g2o-bsd.txt); included Ceres headers retain [BSD-3-Clause](src/posim_slam/data/notices/g2o-ceres.txt) |
| Eigen | 3.4.0; `3147391d946bb4b6c68edd901f2add6ac1f31f8c` | [MPL-2.0](src/posim_slam/data/notices/eigen-mpl2.txt) and per-file permissive notices; [upstream guidance](src/posim_slam/data/notices/eigen-readme.txt) |
| FBoW | `c6e3c29e3332a0b0834021797e2aa4e8eb66a3c1` | [MIT](src/posim_slam/data/notices/fbow.txt) |
| tinycolormap | `67198d2b2b48ca5e97600c83b5c0f2310cfd4c03` | [MIT](src/posim_slam/data/notices/tinycolormap.txt) |
| OpenCV | Native qualified profile: 4.13.0 | [Apache-2.0](https://github.com/opencv/opencv/blob/4.13.0/LICENSE); OpenCV's own third-party notices also apply |
| NumPy | Python dependency; native qualified SDK profile: 2.5.3 | [BSD-3-Clause and bundled notices](https://numpy.org/doc/stable/license.html); wheels/SDKs supply their own exact build notices |
| SciPy | Optional volume-diagnostic tool; tested SDK profile: 1.18.1 | [BSD-3-Clause](https://github.com/scipy/scipy/blob/main/LICENSE.txt); installed wheels/SDKs retain their bundled notices; the POSIM-SLAM wheel does not redistribute these binaries |
| ROS 2, yaml-cpp, spdlog, fmt and native runtime libraries | Supplied by the selected development SDK | Package-specific notices and versions in that SDK; the Python package does not redistribute these binaries |

The selected visual build defines `EIGEN_MPL2_ONLY`, disables g2o LGPL/CSparse/
CHOLMOD libraries and GPL applications, and applies the supplied Eigen-solver
patch. This statement applies to that build configuration, not all possible g2o
or Eigen configurations. The source [pin file](src/posim_slam/data/visual/sources.json)
and original notice [hash manifest](src/posim_slam/data/notices/manifest.json) are
authoritative for this package. Additional permissive Eigen notices are retained
beside the MPL notice.

시각 빌드는 `EIGEN_MPL2_ONLY`를 정의하고 g2o의 LGPL/CSparse/CHOLMOD 라이브러리와
GPL 앱을 끈 뒤 포함된 Eigen 솔버 패치를 적용합니다. 이 설명은 선택한 빌드 구성에
해당합니다. 소스 버전 파일과 원본 고지 해시 목록이 패키지의 기준입니다.

## Source availability / 소스 제공

The visual dependency source is pinned to the commits above. Obtain the exact
upstream source from [Stella](https://github.com/stella-cv/stella_vslam/tree/e445b5452535e781fdf6777ec33a06f5f8f5e416),
[g2o](https://github.com/RainerKuemmerle/g2o/tree/e8df2004e07ea8f5b8e6a8b9f2dc067b45b45036),
and [Eigen](https://gitlab.com/libeigen/eigen/-/tree/3147391d946bb4b6c68edd901f2add6ac1f31f8c).
Apply the patches in `src/posim_slam/data/visual` when reproducing the selected
provider. The Python package includes adapter sources, pins and notices; it does
not include complete upstream dependency trees or compiled providers.

When distributing a compiled visual provider, preserve the copyright/license
notices and provide the applicable MPL-covered Eigen source, including any
changes, with clear instructions for obtaining it. WWW-POSIM's optional visual
bundle stores matching dependency source in
`www-posim/practice/visual-slam/dependency-sources`, with its build receipt and
patches. MPL obligations apply to covered files; they do not require unrelated
WWW-POSIM application files to be published. See the [Mozilla MPL FAQ](https://www.mozilla.org/en-US/MPL/2.0/FAQ/).

빌드한 공급자를 배포할 때 저작권·라이선스 고지와 MPL 적용 Eigen 소스 및 변경분의
취득 경로를 제공해야 합니다. WWW-POSIM의 선택형 시각 번들은 위 경로에 의존성
소스·빌드 기록·패치를 함께 보존합니다. Python 패키지 자체에는 어댑터 소스·버전·
고지가 들어 있으며 전체 외부 소스 트리나 빌드한 공급자는 포함하지 않습니다.
MPL 적용 파일의 의무가 관련 없는 WWW-POSIM 애플리케이션 파일 공개를 요구하지는
않습니다.

No ORB-SLAM3 code, vocabulary or learned weights are included. The opt-in native
Basalt backend has the separate experimental dependency closure below.
Kimera-VIO is not a selected backend. Top-level licenses do not approve a future
dependency closure. In particular, Kimera's
default build links Triangle, whose source notice requires a direct arrangement
for commercial-system distribution.

ORB-SLAM3 코드·사전·학습 가중치는 포함하지 않습니다. 선택형 네이티브 Basalt
백엔드는 아래에 별도 실험용 의존성 구성이 있습니다. Kimera-VIO는 선택한 백엔드가
아닙니다. 최상위 라이선스만으로 향후 의존성 전체를 승인하지 않습니다.
Kimera 기본 빌드의 Triangle 고지에는 상업 시스템 배포 시 저자와의 별도
협의 조건이 있습니다.

## Experimental Basalt research recipe / 실험적 Basalt 연구

The original hydrostatic-height factor and unknown-datum marginalization in
`native/basalt/pressure_factors.hpp` are POSIM-SLAM Apache-2.0 code. They add no
third-party dependency; the Basalt and SDK notices below still apply.

`native/basalt/pressure_factors.hpp`의 자체 정수압 높이 제약과 미지 기준값
주변화 코드는 POSIM-SLAM Apache-2.0 코드입니다. 새 외부 의존성은 없으며,
아래 Basalt·SDK 고지는 계속 적용됩니다.

The opt-in [headless target](native/basalt/README.md) has its own exact
[source pins](native/basalt/sources.json): Basalt/basalt-headers and OpenGV
(BSD-3-Clause), Sophus (MIT), cereal (BSD-3-Clause), magic_enum (MIT), and
nlohmann JSON (MIT). The recipe does not redistribute these source trees in
the Python wheel. The builder verifies clean pinned checkouts, retains them
and copies original license files beside its experimental binary receipt.
SDK Eigen **5.0.1** (MPL-2.0 and per-file notices), OpenCV, TBB and fmt have
their own SDK notices and source obligations. The existing Stella Eigen 3.4.0
notices do not substitute for the experimental SDK's Eigen 5.0.1 notices.
No GUI/AprilTag/camera-driver target is linked. This research recipe does not
qualify another dependency closure for application redistribution.

The build-local IMU interval extension preserves the pinned Basalt header's
BSD-3-Clause notice. Its source and preparation script are included under
`native/basalt/imu_continuous.inc` and `prepare_joint_headers.py`; it introduces
no additional dependency.

The build-local optical-flow overlays preserve Basalt's BSD-3-Clause headers.
`prepare_joint_headers.py` verifies their pinned hashes, adds reference-count
diagnostics and retires unobserved birth-frame patches using the Apache-2.0
`reference_patches.hpp`. The original dependency checkout remains unchanged.
Both tracking algorithms originate in Basalt; this adds no dependency.

선택형 연구 대상의 고정 버전과 고지는 위 파일에서 관리합니다. 외부 소스는 Python
wheel에 포함하지 않으며 빌더가 정확한 원본 체크아웃을 검사하고 고지를 보존합니다.
SDK의 Eigen 5.0.1·OpenCV·TBB·fmt 고지와 해당 소스 의무는 별도로 적용됩니다.
Stella용 Eigen 3.4.0 고지를 이 연구 대상에 대신 적용하지 않습니다. GUI·AprilTag·
카메라 드라이버 대상은 연결하지 않습니다. 앱의 기본 실행 백엔드는 바뀌지 않습니다.

빌드 사본의 IMU 구간 확장은 고정 Basalt 헤더의 BSD-3-Clause 고지를 보존합니다.
확장 소스와 생성 스크립트를 위 경로로 제공하며 추가 의존성은 없습니다.

빌드 사본의 광학 흐름 변경은 Basalt의 BSD-3-Clause 헤더를 보존합니다.
`prepare_joint_headers.py`가 고정한 원본 해시를 검사하고 참조 수 진단값과
관측이 끝난 최초 프레임 패치의 해제를 추가합니다. 해제 함수
`reference_patches.hpp`는 Apache-2.0 코드입니다. 원본 의존성 체크아웃은
바꾸지 않습니다. 두 추적 알고리즘은 Basalt에서 제공하며 새 의존성은 없습니다.
