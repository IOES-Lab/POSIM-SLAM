// SPDX-License-Identifier: Apache-2.0
// Bounded cross-language qualification; no live transport or terrain truth input.
#include "sonar_ground_group.hpp"
#include "ping_optimizer.hpp"
#include "ping_prior.hpp"
#include <nlohmann/json.hpp>
#include <iostream>
using namespace posim_slam;
using Json=nlohmann::json;
Eigen::MatrixXd matrix(const Json& value) {
  if(!value.is_array() || value.empty() || value.size()>1024 ||
     !value[0].is_array() || value[0].empty() || value[0].size()>1024)
    throw std::invalid_argument("probe_matrix_shape");
  Eigen::MatrixXd m(value.size(),value[0].size());
  for(Eigen::Index i=0;i<m.rows();++i) {
    if(value[i].size()!=size_t(m.cols()))throw std::invalid_argument("probe_matrix_shape");
    for(Eigen::Index k=0;k<m.cols();++k)m(i,k)=value[i][k].get<double>();
  }
  return m;
}
Vec3 vector3(const Json& value) {
  if(value.size()!=3)throw std::invalid_argument("probe_vector_shape");
  return Vec3(value[0],value[1],value[2]);
}
Json jsonMatrix(const Eigen::MatrixXd& value) {
  Json out=Json::array();
  for(Eigen::Index i=0;i<value.rows();++i) {
    Json row=Json::array();for(Eigen::Index k=0;k<value.cols();++k)row.push_back(value(i,k));out.push_back(row);
  }
  return out;
}
int main() {try {
  Json input;std::cin>>input;
  basalt::BundleAdjustmentBase<double> owner;owner.huber_thresh=1.;owner.obs_std_dev=1.;
  basalt::AbsOrderMap order;std::vector<GroundEntryObservation> observations;
  for(const auto& item:input.at("observations")) {
    const int64_t stamp=item.at("stamp_ns");const auto& state=input.at("states").at(std::to_string(stamp));
    const Eigen::Matrix3d rotation=matrix(state.at("attitude"));
    if(!rotation.allFinite() || (rotation.transpose()*rotation-Mat3::Identity()).norm()>1e-8 || std::abs(rotation.determinant()-1)>1e-8)
      throw std::invalid_argument("probe_rotation");
    const Sophus::SE3d pose(rotation,vector3(state.at("position")));
    if(!owner.frame_poses.emplace(stamp,basalt::PoseStateWithLin<double>(stamp,pose,true)).second)
      throw std::invalid_argument("probe_duplicate_state");
    if(state.contains("delta")) {
      const auto values=state.at("delta").get<std::vector<double>>();
      if(values.size()!=6)throw std::invalid_argument("probe_pose_delta");
      owner.frame_poses.at(stamp).applyInc(Eigen::Map<const Eigen::Matrix<double,6,1>>(values.data()));
    }
    order.abs_order_map[stamp]={order.total_size,6};order.total_size+=6;++order.items;
    const auto ranges=item.at("range_m").get<std::vector<double>>(),az=item.at("azimuth_rad").get<std::vector<double>>();
    observations.push_back({stamp,item.at("beam_ids").get<std::vector<int>>(),
      Eigen::Map<const Eigen::VectorXd>(az.data(),az.size()),Eigen::Map<const Eigen::VectorXd>(ranges.data(),ranges.size()),item.at("boundary_sign")});
  }
  GroundEntryCalibration c{Sophus::SE3d(Mat3(matrix(input.at("sonar_to_imu"))),vector3(input.at("imu_to_sonar"))),
    input.at("vertical_fov"),input.at("range_min"),input.at("range_max")};
  owner.posim_ping_planes.set(0,vector3(input.at("plane_normal")),input.at("plane_distance"));
  if(input.contains("plane_delta"))owner.posim_ping_planes.apply(0,vector3(input.at("plane_delta")));
  const auto group=makeOwnedGroundGroup(owner,order,0,c,observations,matrix(input.at("measurement_covariance")));
  const auto raw=group.build({});const auto reduced=eliminateJointPatch(raw.jacobian,raw.residual,order.total_size);
  Json output={{"jacobian",jsonMatrix(raw.jacobian)},{"residual",jsonMatrix(raw.residual)},
    {"reduced_H",jsonMatrix(reduced.jacobian.transpose()*reduced.jacobian)},
    {"reduced_g",jsonMatrix(reduced.jacobian.transpose()*reduced.residual)},
    {"nuisance_rank",reduced.nuisance_rank}};
  if(input.value("check_derivatives",false)) {
    constexpr double epsilon=1e-6;
    Eigen::MatrixXd numerical(raw.jacobian.rows(),raw.jacobian.cols());
    for(Eigen::Index k=0;k<numerical.cols();++k) {
      Eigen::VectorXd values[2];
      for(int sign=0;sign<2;++sign) {
        owner.backup();const double step=(sign?1.:-1.)*epsilon;
        if(k<Eigen::Index(order.total_size)) {
          Eigen::Matrix<double,6,1> delta=Eigen::Matrix<double,6,1>::Zero();delta[k%6]=step;
          owner.frame_poses.at(observations[k/6].stamp).applyInc(delta);
        } else {
          Vec3 delta=Vec3::Zero();delta[k-order.total_size]=step;owner.posim_ping_planes.apply(0,delta);
        }
        values[sign]=group.build({}).residual;owner.restore();
      }
      numerical.col(k)=(values[1]-values[0])/(2*epsilon);
    }
    output["numerical_jacobian"]=jsonMatrix(numerical);
    output["maximum_derivative_error"]=(numerical-raw.jacobian).cwiseAbs().maxCoeff();
  }
  if(input.contains("prior_sqrt")) {
    basalt::MargLinData<double> prior;prior.is_sqrt=true;prior.order=order;
    prior.H=matrix(input.at("prior_sqrt"));prior.b=Eigen::VectorXd::Zero(prior.H.rows());
    if(prior.H.cols()!=Eigen::Index(order.total_size))throw std::invalid_argument("probe_prior_shape");
    const Vec3 gravity(0,0,-9.80665),weight=Vec3::Ones();
    const basalt::ImuLinData<double> imu{gravity,weight,weight,{}};
    const auto prepared=preparePingPrior(owner,order,{group},prior,imu,{observations.front().stamp},{},{});
    output["retired_stamps"]=prepared.retirement.observations;
    output["marginal_H"]=jsonMatrix(prepared.prior.H.transpose()*prepared.prior.H);
    output["marginal_g"]=jsonMatrix(prepared.prior.H.transpose()*prepared.prior.b);
    const auto result=optimizePingGroups(owner,order,{group},prior,imu,{20,8,.001});
    output["optimization"]={{"accepted",result.accepted},{"initial_cost",result.initial_cost},
      {"final_cost",result.final_cost},{"termination",result.termination}};
    for(const auto& o:observations) {
      const auto pose=owner.frame_poses.at(o.stamp).getPose();
      output["states"][std::to_string(o.stamp)]={{"position",jsonMatrix(pose.translation())},
        {"attitude",jsonMatrix(pose.rotationMatrix())}};
    }
    output["plane_normal"]=jsonMatrix(owner.posim_ping_planes.at(0).normal);
    output["plane_distance"]=owner.posim_ping_planes.at(0).distance;
  }
  std::cout<<output.dump()<<'\n';
}catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}}
