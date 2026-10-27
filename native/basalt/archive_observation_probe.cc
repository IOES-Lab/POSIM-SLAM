// SPDX-License-Identifier: Apache-2.0
#include "visual_support.hpp"
#include <cassert>
#include <iostream>

int main(int argc,char** argv) {
  if(argc!=2)return 2;
  const auto c=input_calibration(argv[1]);
  basalt::VioVisualizationData data;data.states.push_back(Sophus::SE3d());
  const Sophus::SE3d current(Sophus::SO3d::exp(Eigen::Vector3d(.1,-.2,.05)),
      Eigen::Vector3d(1.,2.,-3.));
  data.states.push_back(current);
  data.opt_flow_res=std::make_shared<basalt::OpticalFlowResult>();
  data.opt_flow_res->observations.resize(1);
  const Eigen::Vector3d world=current*c.cal.T_i_c[0]*Eigen::Vector3d(.1,.2,4.);
  LandmarkArchive::Point point{float(world.x()),float(world.y()),float(world.z())};
  Eigen::Vector2d pixel;
  assert(c.cal.intrinsics[0].project(Eigen::Vector3d(.1,.2,4.),pixel));
  Eigen::AffineCompact2f observation=Eigen::AffineCompact2f::Identity();
  observation.translation()=pixel.cast<float>();
  LandmarkArchive archive;
  for(int t=1;t<=3;++t)archive.update(t,{7},{point},{7});
  const auto query=[&] {return archive_observation_outliers(archive,data,c.cal,c.processed_width,c.processed_height,1.);};
  assert(query().empty());  // No received feature; retain the map.
  data.opt_flow_res->observations[0][7]=observation;
  assert(query().empty());
  assert(data.point_ids.empty());  // Retired optimizer point is still tested.
  data.opt_flow_res->observations[0][7].translation().x()+=4.;
  assert(query()==std::set<int>{7});
  archive.withdraw_observed_outliers(query());
  assert(archive.points().empty() && archive.withdrawn()==1);
  for(int t=4;t<=5;++t)archive.update(t,{7},{point},{7});
  assert(archive.points().empty());
  archive.update(6,{7},{point},{7});assert(archive.points().size()==1);
  data.opt_flow_res->observations[0].clear();
  data.opt_flow_res->observations[0][8]=observation;
  assert(query().empty() && archive.points().size()==1);
  data.opt_flow_res->observations[0][7]=observation;
  const Eigen::Vector3d behind=current*c.cal.T_i_c[0]*Eigen::Vector3d(0,0,-4.);
  archive.update(7,{7},{{float(behind.x()),float(behind.y()),float(behind.z())}},{7});
  assert(query()==std::set<int>{7});
  std::cout<<"archive observation, retirement, occlusion and fresh admission controls passed\n";
}
