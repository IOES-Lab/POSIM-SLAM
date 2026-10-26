// SPDX-License-Identifier: Apache-2.0
#include "visual_support.hpp"
#include <cassert>

int main() {
  basalt::VioVisualizationData data;
  assert(visual_track_trace(data,100,100,1.,{}).empty());
  data.opt_flow_res=std::make_shared<basalt::OpticalFlowResult>();
  data.opt_flow_res->observations.resize(1);data.projections.resize(1);
  for(int id=1;id<=3;++id) {
    auto& obs=data.opt_flow_res->observations[0][id];obs.setIdentity();obs.translation()<<10*id,20;
  }
  data.point_ids={2,3};data.points.emplace_back(1.,2.,3.);data.points.emplace_back(4.,5.,6.);
  data.projections[0].emplace_back(20.2,20.,1.,2.);
  data.projections[0].emplace_back(32.,20.,1.,3.);
  const auto rows=visual_track_trace(data,100,100,1.,{3});
  assert(rows.size()==3 && rows[0]["point"].is_null());
  assert(rows[1]["full_weight"]==true && rows[1]["archived"]==false);
  assert(rows[2]["full_weight"]==false && rows[2]["archived"]==true);
  assert(rows[1]["point"]==Json({1.,2.,3.}));
  assert(rows[2]["pixel"]==Json({30.,20.}));
  bool missing=false;
  try {unqualified_visual_points(data);}catch(const std::runtime_error& error) {
    missing=std::string(error.what())=="visual_geometry_identity_missing";
  }
  assert(missing);
  data.map_quality[2].eligible=true;
  data.map_quality[3].eligible=false;
  assert(unqualified_visual_points(data)==std::set<int>({3}));
}
