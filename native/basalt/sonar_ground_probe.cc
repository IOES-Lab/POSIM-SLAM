// SPDX-License-Identifier: Apache-2.0
// Bounded cross-language qualification; no live transport or terrain truth input.
#include "sonar_ground_group.hpp"
#include "sonar_plane_group.hpp"
#include "ground_predictive.hpp"
#include "ground_surface_noise.hpp"
#include "ping_optimizer.hpp"
#include "ping_prior.hpp"
#include "ping_commit.hpp"
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
  if(input.contains("surface_points")) {
    const auto c=surfaceRangeCovariance(matrix(input.at("surface_points")),matrix(input.at("surface_incidence")),
      input.at("surface_sigma_m"),input.at("surface_length_m"));
    std::cout<<Json({{"surface_covariance",jsonMatrix(c)}}).dump()<<'\n';return 0;
  }
  if(input.contains("association_jacobian")) {
    const auto check=groundPredictiveCheck(matrix(input.at("association_jacobian")),
      matrix(input.at("association_residual")),input.at("association_old_rows"),input.at("association_sigma"));
    std::cout<<Json({{"score",check.score},{"limit",check.limit},{"new_rows",check.new_rows}}).dump()<<'\n';return 0;
  }
  basalt::BundleAdjustmentBase<double> owner;owner.huber_thresh=1.;owner.obs_std_dev=1.;
  basalt::AbsOrderMap order;std::vector<GroundEntryObservation> observations;
  std::vector<SonarPlaneFrame> plane_frames;std::vector<int64_t> stamps;
  const bool general=input.contains("planes");
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
    stamps.push_back(stamp);
    if(general) {
      SonarPlaneFrame frame{stamp,{}};
      for(const auto& row:item.at("returns"))frame.returns.push_back({row.at("source_id"),row.at("plane_id"),row.at("range_m"),matrix(row.at("rays"))});
      plane_frames.push_back(frame);continue;
    }
    const auto ranges=item.at("range_m").get<std::vector<double>>(),az=item.at("azimuth_rad").get<std::vector<double>>();
    observations.push_back({stamp,item.at("beam_ids").get<std::vector<int>>(),
      Eigen::Map<const Eigen::VectorXd>(az.data(),az.size()),Eigen::Map<const Eigen::VectorXd>(ranges.data(),ranges.size()),item.at("boundary_sign")});
  }
  GroundEntryCalibration c{Sophus::SE3d(Mat3(matrix(input.at("sonar_to_imu"))),vector3(input.at("imu_to_sonar"))),
    input.value("vertical_fov",.4),input.at("range_min"),input.at("range_max")};
  if(general) {
    for(const auto& plane:input.at("planes")) {
      int id=plane.at("id");owner.posim_ping_planes.set(id,vector3(plane.at("normal")),plane.at("distance"));
      if(plane.contains("delta"))owner.posim_ping_planes.apply(id,vector3(plane.at("delta")));
    }
  }else owner.posim_ping_planes.set(0,vector3(input.at("plane_normal")),input.at("plane_distance"));
  if(input.contains("plane_delta"))owner.posim_ping_planes.apply(0,vector3(input.at("plane_delta")));
  const auto group=general ? makeOwnedSonarPlanes(owner,order,c.mount,c.minimum,c.maximum,plane_frames,matrix(input.at("measurement_covariance"))) :
    makeOwnedGroundGroup(owner,order,0,c,observations,matrix(input.at("measurement_covariance")));
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
          owner.frame_poses.at(stamps[k/6]).applyInc(delta);
        } else {
          const auto index=k-order.total_size;Vec3 delta=Vec3::Zero();delta[index%3]=step;owner.posim_ping_planes.apply(group.planeIds()[index/3],delta);
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
    const auto prepared=preparePingPrior(owner,order,{group},prior,imu,{stamps.front()},{},{});
    output["retired_stamps"]=prepared.retirement.observations;
    output["retired_planes"]=prepared.retirement.planes;
    output["marginal_H"]=jsonMatrix(prepared.prior.H.transpose()*prepared.prior.H);
    output["marginal_g"]=jsonMatrix(prepared.prior.H.transpose()*prepared.prior.b);
    const auto result=optimizePingGroups(owner,order,{group},prior,imu,{20,8,.001});
    output["optimization"]={{"accepted",result.accepted},{"initial_cost",result.initial_cost},
      {"final_cost",result.final_cost},{"termination",result.termination}};
    for(int64_t stamp:stamps) {
      const auto pose=owner.frame_poses.at(stamp).getPose();
      output["states"][std::to_string(stamp)]={{"position",jsonMatrix(pose.translation())},
        {"attitude",jsonMatrix(pose.rotationMatrix())}};
    }
    for(int id:group.planeIds())output["planes"][std::to_string(id)]={{"normal",jsonMatrix(owner.posim_ping_planes.at(id).normal)},
      {"distance",owner.posim_ping_planes.at(id).distance}};
    output["plane_normal"]=jsonMatrix(owner.posim_ping_planes.at(0).normal);
    output["plane_distance"]=owner.posim_ping_planes.at(0).distance;
    if(input.value("check_install",false)) {
      const auto final=preparePingPrior(owner,order,{group},prior,imu,{stamps.front()},{},{});
      auto groups=std::vector<JointPingGroupSpec>{group};
      std::map<int64_t,basalt::IntegratedImuMeasurement<double>> consumed;
      installPingMarginalPrior(owner,prior,groups,consumed,final.retirement,final.layout,{stamps.front()},{},final.prior);
      bool replay_rejected=false;
      try {installPingMarginalPrior(owner,prior,groups,consumed,final.retirement,final.layout,{stamps.front()},{},final.prior);}
      catch(const std::exception&){replay_rejected=true;}
      output["installation"]={{"remaining_planes",owner.posim_ping_planes.size()},{"remaining_groups",groups.size()},
        {"remaining_poses",owner.frame_poses.size()},{"replay_rejected",replay_rejected}};
    }
  }
  std::cout<<output.dump()<<'\n';
}catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}}
