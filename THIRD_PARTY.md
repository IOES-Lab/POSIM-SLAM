# Third-party code and notices / 제3자 코드와 고지

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

No ORB-SLAM3 code, vocabulary or learned weights are included. Basalt, Kimera-VIO
and their dependencies are not selected runtime backends. Their top-level
licenses do not approve a future dependency closure. In particular, Kimera's
default build links Triangle, whose source notice requires a direct arrangement
for commercial-system distribution.

ORB-SLAM3 코드·사전·학습 가중치는 포함하지 않습니다. Basalt와 Kimera-VIO는 현재
실행 백엔드가 아닙니다. 후보의 최상위 라이선스만으로 향후 의존성 전체를 승인하지
않습니다. Kimera 기본 빌드의 Triangle 고지에는 상업 시스템 배포 시 저자와의 별도
협의 조건이 있습니다.

## Experimental Basalt research recipe / 실험적 Basalt 연구

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

선택형 연구 대상의 고정 버전과 고지는 위 파일에서 관리합니다. 외부 소스는 Python
wheel에 포함하지 않으며 빌더가 정확한 원본 체크아웃을 검사하고 고지를 보존합니다.
SDK의 Eigen 5.0.1·OpenCV·TBB·fmt 고지와 해당 소스 의무는 별도로 적용됩니다.
Stella용 Eigen 3.4.0 고지를 이 연구 대상에 대신 적용하지 않습니다. GUI·AprilTag·
카메라 드라이버 대상은 연결하지 않습니다. 앱의 기본 실행 백엔드는 바뀌지 않습니다.
