// SPDX-License-Identifier: Apache-2.0
#pragma once
#include "ping_joint.hpp"
#include <basalt/utils/imu_types.h>
#include <algorithm>
#include <set>

namespace posim_slam {
struct MappingHandoffRequest {
  std::set<int64_t> keyframes,retired;
  std::vector<basalt::OpticalFlowResult::Ptr> images;
  // Assigned only after the matching marginal-prior transaction commits.
  basalt::MargData::Ptr committed;
};

// Preserve the complete linearized joint information, including any pressure
// datum column. Keep keyframe poses and eliminate every other coordinate using
// the same rank-aware square-root operation used by joint factor retirement.
// Images are copied so subsequent estimator/frontend mutations cannot alter the
// mapper's snapshot. No world reference, damping or observation is added.
template<class Owner> basalt::MargData::Ptr prepareMappingHandoff(Owner& owner,
    const basalt::AbsOrderMap& order,const Eigen::MatrixXd& jacobian,
    const Eigen::VectorXd& residual,const MappingHandoffRequest& request) {
  if(jacobian.rows()<1 || jacobian.rows()>4096 || jacobian.cols()<1 ||
      jacobian.cols()>kMaxJointColumns || residual.size()!=jacobian.rows() ||
      !jacobian.allFinite() || !residual.allFinite() ||
      request.committed || request.retired.empty() || request.keyframes.size()<2 ||
      request.keyframes.size()*6>size_t(kMaxJointColumns) ||
      !std::includes(request.keyframes.begin(),request.keyframes.end(),request.retired.begin(),request.retired.end()) ||
      request.images.size()!=request.keyframes.size() ||
      (jacobian.cols()!=Eigen::Index(order.total_size) &&
       jacobian.cols()!=Eigen::Index(order.total_size+1)))
    throw std::invalid_argument("invalid_mapping_handoff_request");
  auto out=std::make_shared<basalt::MargData>();
  out->use_imu=true;out->kfs_all=request.keyframes;out->kfs_to_marg=request.retired;
  std::vector<int> kept;std::vector<bool> used(jacobian.cols(),false);
  for(int64_t stamp:request.keyframes) {
    const auto found=order.abs_order_map.find(stamp);
    if(found==order.abs_order_map.end() || (found->second.second!=6 && found->second.second!=15) ||
        found->second.first+found->second.second>order.total_size)
      throw std::invalid_argument("missing_mapping_keyframe_coordinates");
    for(int i=0;i<6;++i) {
      const int column=found->second.first+i;
      if(used.at(column))throw std::invalid_argument("overlapping_mapping_coordinates");
      used[column]=true;kept.push_back(column);
    }
    out->aom.abs_order_map.emplace(stamp,std::make_pair(out->aom.total_size,6));
    out->aom.total_size+=6;++out->aom.items;
    out->frame_poses.emplace(stamp,basalt::PoseStateWithLin<double>(stamp,owner.getPoseStateWithLin(stamp).getPose()));
  }
  Eigen::MatrixXd reordered(jacobian.rows(),jacobian.cols());Eigen::Index index=0;
  for(int column:kept)reordered.col(index++)=jacobian.col(column);
  for(Eigen::Index column=0;column<jacobian.cols();++column)
    if(!used[column])reordered.col(index++)=jacobian.col(column);
  const auto reduced=splitJointInformation(reordered,residual,kept.size(),false).reduced;
  out->abs_H=reduced.jacobian.transpose()*reduced.jacobian;
  out->abs_b=reduced.jacobian.transpose()*reduced.residual;
  std::set<int64_t> images;size_t total_pixels=0;
  for(const auto& flow:request.images) {
    if(!flow || !flow->input_images || flow->input_images->t_ns!=flow->t_ns ||
        !request.keyframes.count(flow->t_ns) || !images.insert(flow->t_ns).second ||
        flow->input_images->img_data.empty() || flow->input_images->img_data.size()>4)
      throw std::invalid_argument("invalid_mapping_image_identity");
    auto copy=std::make_shared<basalt::OpticalFlowResult>(*flow);
    copy->input_images=std::make_shared<basalt::OpticalFlowInput>(*flow->input_images);
    for(auto& image:copy->input_images->img_data) {
      if(!image.img || image.img->w<1 || image.img->h<1 || image.img->w>1920 || image.img->h>1080)
        throw std::invalid_argument("invalid_mapping_image_shape");
      total_pixels+=size_t(image.img->w)*image.img->h;
      if(total_pixels>16*1024*1024)throw std::invalid_argument("mapping_snapshot_image_budget");
      auto owned=std::make_shared<basalt::ManagedImage<uint16_t>>(image.img->w,image.img->h);
      owned->CopyFrom(*image.img);
      image.img=std::move(owned);
    }
    out->opt_flow_res.push_back(std::move(copy));
  }
  return out;
}
} // namespace posim_slam

