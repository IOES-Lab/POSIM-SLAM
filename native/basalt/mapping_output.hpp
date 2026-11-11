// SPDX-License-Identifier: Apache-2.0
#pragma once
#include "mapping_graph.hpp"
#include <array>

namespace posim_slam {
struct PersistentMapCandidate {
  int64_t id;
  int64_t native_id=-1;
  Eigen::Vector3d position;
  bool eligible=false, finite=false, observed=false, mature=false;
  double maximum_sigma_m=std::numeric_limits<double>::infinity();
};
struct MappingOutput {
  size_t observations=0,factors=0,observed=0,nonfinite=0;
  std::vector<PersistentMapCandidate> candidates;
  std::vector<std::array<float,3>> points;
  std::vector<int> ids;
};

// Preserve every candidate, including weak two-view and infinite landmarks.
// Only the shared certificate selects downloadable/displayable finite points.
// Quality and graph must belong to this exclusively owned mapper state.
inline MappingOutput mappingOutput(basalt::NfrMapper& mapper,
    const Sophus::SE3d& gauge,const MappingInformationResult& quality,
    const MappingGraphRegistry::Snapshot& graph) {
  if(!graph || !gauge.matrix().allFinite())
    throw std::invalid_argument("mapping_output_graph_mismatch");
  MappingOutput out;out.observations=quality.observations;out.factors=quality.pose_factors;
  for(const auto& [id,landmark]:mapper.lmdb.getLandmarks()) {
    const auto host=mapper.getFramePoses().at(landmark.host_kf_id.frame_id).getPose()
                      *mapper.calib.T_i_c.at(landmark.host_kf_id.cam_id);
    auto point=basalt::StereographicParam<double>::unproject(landmark.direction);point[3]=landmark.inv_dist;
    const auto world=(host*point).eval();PersistentMapCandidate candidate;
    candidate.native_id=id;candidate.id=graph->stable_ids.at(id);
    candidate.position=gauge*(world.head<3>()/world[3]);
    candidate.finite=candidate.position.allFinite() && landmark.inv_dist>0;
    out.nonfinite+=!candidate.finite;
    const auto found=quality.points.find(id);
    if(found!=quality.points.end()) {
      candidate.eligible=found->second.eligible;candidate.observed=found->second.world.observed;
      candidate.mature=found->second.mature;candidate.maximum_sigma_m=found->second.world.maximum_sigma_m;
    }
    out.observed+=candidate.observed;
    if(candidate.eligible) {
      if(!candidate.finite || id>std::numeric_limits<int>::max() ||
          candidate.position.cwiseAbs().maxCoeff()>std::numeric_limits<float>::max())
        throw std::runtime_error("mapping_certified_point_invalid");
      out.points.push_back({float(candidate.position.x()),float(candidate.position.y()),float(candidate.position.z())});
      out.ids.push_back(int(candidate.id));
    }
    out.candidates.push_back(std::move(candidate));
  }
  return out;
}
} // namespace posim_slam
