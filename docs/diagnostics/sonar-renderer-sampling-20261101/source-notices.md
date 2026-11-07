# Source notices / 소스 고지

Original diagnostic code is Copyright 2026 IOES-Lab, Apache-2.0; see [LICENSE](LICENSE).

`sampling.py` adapts the cube-face coordinate mapping and nearest-texel sampling
from Gazebo Rendering 10.0.2 `Ogre2GpuRays.cc`, Copyright 2018 Open Source Robotics
Foundation, Apache-2.0. The original source is pinned in
[sampling-identity.json](sampling-identity.json). No Gazebo executable or binary
is distributed in this record.

The original normal helper is POSIM `RaySurfaceNormals.hh` at
`8d131df47fd745815bd15f36365a4bd49aa13e47`, Apache-2.0. Its hash and full-tensor
native comparison remain in [native-normal-control.json](native-normal-control.json).
`raw_to_db.cc` reproduces POSIM's raw publication expression under Apache-2.0.

RGB and reference scene imagery use Orca5 assets at
`5f7cdc1f044926ebd56f2ce8a1fc4a82990cfa5e`; the original [MIT notice](Orca5-MIT.txt)
is retained. The field uses a 3× horizontal seabed footprint and added physical
test structures. ORB-SLAM and its data are not included.

자체 진단 코드는 Apache-2.0입니다. Gazebo Rendering과 POSIM의 고정 소스·
저작권·라이선스를 유지합니다. RGB·기준 장면은 위 Orca5 자산과 MIT 고지를
사용하며 해저 수평 범위를 3배로 확장하고 시험 구조물을 추가합니다.
이 기록은 Gazebo 바이너리나 ORB-SLAM 코드를 배포하지 않습니다.
