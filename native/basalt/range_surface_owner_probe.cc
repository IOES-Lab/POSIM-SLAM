// SPDX-License-Identifier: Apache-2.0
// Real Basalt owner, additive state updates and existing marginalization.
#include "range_surface_group.hpp"
#include "range_surface_probe_io.hpp"
#include "ping_optimizer.hpp"
#include "ping_prior.hpp"
#include "ping_commit.hpp"
#include <iostream>
namespace probe=posim_slam::range_surface_probe;
using namespace posim_slam;
int main() {
  try {
    probe::Json input;std::cin>>input;const auto parsed=probe::read(input);
    basalt::BundleAdjustmentBase<double> owner;owner.huber_thresh=1.;owner.obs_std_dev=1.;
    basalt::AbsOrderMap order;std::vector<int64_t> stamps;
    for(const auto& [stamp,p]:parsed.poses) {
      const auto& s=input.at("states").at(std::to_string(stamp));
      const Mat3 reference=s.contains("rotation_reference") ? probe::rotation(s.at("rotation_reference")) : p.attitude;
      const Sophus::SE3d pose(reference,p.position);
      owner.frame_poses.emplace(stamp,basalt::PoseStateWithLin<double>(stamp,pose,true));
      if(s.contains("rotation_delta")) {
        Eigen::Matrix<double,6,1> delta=Eigen::Matrix<double,6,1>::Zero();delta.tail<3>()=probe::vector<3>(s.at("rotation_delta"));
        owner.frame_poses.at(stamp).applyInc(delta);
      }
      order.abs_order_map[stamp]={order.total_size,6};order.total_size+=6;++order.items;stamps.push_back(stamp);
    }
    std::map<int,RangeSurfaceSupport> support;
    for(const auto& [id,p]:parsed.planes) {
      const auto& s=input.at("planes").at(std::to_string(id));
      const Vec3 reference=s.contains("normal_reference") ? probe::vector<3>(s.at("normal_reference")) : p.normal;
      owner.posim_ping_planes.set(id,reference,p.distance);
      if(s.contains("normal_delta")) {
        Vec3 delta=Vec3::Zero();delta.head<2>()=probe::vector<2>(s.at("normal_delta"));owner.posim_ping_planes.apply(id,delta);
      }
      support.emplace(id,RangeSurfaceSupport{p.support_keys,p.max_plane_deviation,p.max_support_distance});
    }
    const auto group=makeOwnedRangeSurfaces(owner,order,support,parsed.observations,parsed.covariance,parsed.uncertainty,parsed.require_support);
    const auto raw=group.build({});const auto reduced=eliminateJointPatch(raw.jacobian,raw.residual,order.total_size);
    probe::Json result={{"jacobian",probe::jsonMatrix(raw.jacobian)},{"residual",probe::jsonVector(raw.residual)},
      {"reduced_H",probe::jsonMatrix(reduced.jacobian.transpose()*reduced.jacobian)},
      {"reduced_g",probe::jsonVector(reduced.jacobian.transpose()*reduced.residual)},
      {"nuisance_rank",reduced.nuisance_rank},{"observation_stamps",raw.observation_stamps}};
    const auto ids=group.planeIds();constexpr double epsilon=1e-6;
    Eigen::MatrixXd numerical(raw.jacobian.rows(),raw.jacobian.cols());
    for(Eigen::Index column=0;column<numerical.cols();++column) {
      Eigen::VectorXd values[2];
      for(int sign=0;sign<2;++sign) {
        owner.backup();const double step=(sign ? 1. : -1.)*epsilon;
        if(column<Eigen::Index(order.total_size)) {
          Eigen::Matrix<double,6,1> delta=Eigen::Matrix<double,6,1>::Zero();delta[column%6]=step;
          owner.frame_poses.at(stamps[column/6]).applyInc(delta);
        } else {
          const auto index=column-order.total_size;Vec3 delta=Vec3::Zero();delta[index%3]=step;
          owner.posim_ping_planes.apply(ids[index/3],delta);
        }
        values[sign]=group.build({}).residual;owner.restore();
      }
      numerical.col(column)=(values[1]-values[0])/(2*epsilon);
    }
    result["maximum_derivative_error"]=(raw.jacobian-numerical).cwiseAbs().maxCoeff();
    if(input.contains("prior_sqrt")) {
      basalt::MargLinData<double> prior;prior.is_sqrt=true;prior.order=order;prior.H=probe::matrix(input.at("prior_sqrt"));
      prior.b=Eigen::VectorXd::Zero(prior.H.rows());
      if(prior.H.cols()!=Eigen::Index(order.total_size))throw std::invalid_argument("probe_prior_shape");
      const Vec3 gravity(0,0,-9.80665),weight=Vec3::Ones();const basalt::ImuLinData<double> imu{gravity,weight,weight,{}};
      const auto prepared=preparePingPrior(owner,order,{group},prior,imu,{stamps.front()},{},{});
      result["marginal_H"]=probe::jsonMatrix(prepared.prior.H.transpose()*prepared.prior.H);
      result["retired_stamps"]=prepared.retirement.observations;result["retired_planes"]=prepared.retirement.planes;
      const auto optimized=optimizePingGroups(owner,order,{group},prior,imu,{20,8,.001});
      result["optimization"]={{"accepted",optimized.accepted},{"initial_cost",optimized.initial_cost},
        {"final_cost",optimized.final_cost},{"termination",optimized.termination}};
      // Rebuild the marginal prior from the accepted geometry before committing.
      const auto final=preparePingPrior(owner,order,{group},prior,imu,{stamps.front()},{},{});
      auto groups=std::vector<JointPingGroupSpec>{group};std::map<int64_t,basalt::IntegratedImuMeasurement<double>> consumed;
      installPingMarginalPrior(owner,prior,groups,consumed,final.retirement,final.layout,{stamps.front()},{},final.prior);
      bool replay_rejected=false;
      try {installPingMarginalPrior(owner,prior,groups,consumed,final.retirement,final.layout,{stamps.front()},{},final.prior);}
      catch(const std::exception&){replay_rejected=true;}
      bool retired_source_rejected=false;
      try {group.build({});}catch(const std::exception&){retired_source_rejected=true;}
      result["installation"]={{"remaining_planes",owner.posim_ping_planes.size()},{"remaining_groups",groups.size()},
        {"remaining_poses",owner.frame_poses.size()},{"replay_rejected",replay_rejected},{"retired_source_rejected",retired_source_rejected}};
    }
    std::cout<<result.dump()<<'\n';return 0;
  } catch(const std::exception& error) {std::cerr<<error.what()<<'\n';return 2;}
}
