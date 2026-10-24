// SPDX-License-Identifier: Apache-2.0
#include "calibration.hpp"
#include "ping_patch.hpp"
#include <iostream>

int main() {
  try {
    using namespace posim_slam;
    Eigen::MatrixXd rays(25,3);int row=0;
    for(int y=-2;y<=2;++y)for(int z=-2;z<=2;++z) {
      const double h=y*.074175,v=z*.074175;
      rays.row(row++)=Vec3(std::cos(v)*std::cos(h),std::cos(v)*std::sin(h),std::sin(v)).transpose();
    }
    std::vector<PingPatchPoint> points;size_t id=0;
    for(int y=-8;y<=8;++y)for(int z=-8;z<=8;++z) {
      const double py=y*.08,pz=z*.08;points.push_back({id++,Vec3(3+.1*py+.2*pz,py,pz)});
    }
    const auto patch=selectPingPatch(points,Sophus::SE3d(),Sophus::SE3d(),rays,.2,50.);
    if(!patch)throw std::runtime_error("tilted_patch_not_selected");
    const Vec3 expected=Vec3(1,-.1,-.2).normalized();
    const double plane_error=(patch->normal-expected).norm()+std::abs(patch->distance-3./std::sqrt(1.05));
    std::reverse(points.begin(),points.end());
    const auto reordered=selectPingPatch(points,Sophus::SE3d(),Sophus::SE3d(),rays,.2,50.);
    const bool order_independent=reordered && reordered->ids==patch->ids;
    Sophus::SE3d transform(Sophus::SO3d::exp(Vec3(.3,-.2,.1)),Vec3(4,-2,1));
    auto shifted=points;for(auto& point:shifted)point.point=transform*point.point;
    const auto moved=selectPingPatch(shifted,transform,Sophus::SE3d(),rays,.2,50.);
    const double transform_error=moved ? std::abs(moved->predicted_range-patch->predicted_range) : 1.;
    std::vector<PingPatchPoint> hole;
    for(const auto& point:points)if(point.point.tail<2>().norm()>.3)hole.push_back(point);
    const bool hole_rejected=!selectPingPatch(hole,Sophus::SE3d(),Sophus::SE3d(),rays,.2,50.);
    auto occluded=points;
    for(int k=0;k<6;++k)occluded.push_back({id++,Vec3(2.,.01*k,.01*(k%2))});
    const bool occlusion_rejected=!selectPingPatch(occluded,Sophus::SE3d(),Sophus::SE3d(),rays,.2,50.);
    std::vector<PingPatchPoint> line;
    for(int k=-8;k<=8;++k)line.push_back({id++,Vec3(3.,k*.05,0.)});
    const bool line_rejected=!selectPingPatch(line,Sophus::SE3d(),Sophus::SE3d(),rays,.2,50.);
    std::vector<PingPatchPoint> sparse,sparse_hole;
    for(int y=-3;y<=3;++y)for(int z=-3;z<=3;++z) {
      PingPatchPoint point{id++,Vec3(3.+.1*y*.2+.2*z*.2,y*.2,z*.2)};
      sparse.push_back(point);
      if(std::hypot(y*.2,z*.2)>.4)sparse_hole.push_back(point);
    }
    const auto sparse_patch=selectPingPatch(sparse,Sophus::SE3d(),Sophus::SE3d(),rays,.2,50.);
    const bool sparse_hole_rejected=!selectPingPatch(sparse_hole,Sophus::SE3d(),Sophus::SE3d(),rays,.2,50.);
    bool duplicate_rejected=false;points.push_back(points[0]);
    try {selectPingPatch(points,Sophus::SE3d(),Sophus::SE3d(),rays,.2,50.);}catch(const std::invalid_argument&){duplicate_rejected=true;}
    bool passed=sparse_patch.has_value() && sparse_hole_rejected && plane_error<1e-10 && transform_error<1e-10 && order_independent && hole_rejected && occlusion_rejected && line_rejected && duplicate_rejected && patch->ids.size()<=64;
    std::cout<<Json({{"sparse_patch_selected",sparse_patch.has_value()},{"sparse_hole_rejected",sparse_hole_rejected},{"passed",passed},{"selected",patch->ids.size()},{"plane_error",plane_error},{"rigid_transform_range_error",transform_error},
      {"order_independent",order_independent},{"hole_rejected",hole_rejected},{"foreground_occlusion_rejected",occlusion_rejected},
      {"collinear_rejected",line_rejected},{"duplicate_rejected",duplicate_rejected}}).dump()<<'\n';return passed?0:1;
  }catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}
}
