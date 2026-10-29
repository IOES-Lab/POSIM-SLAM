// SPDX-License-Identifier: Apache-2.0
// Interactive component probe; no simulator truth or application imports.
#include "surface_owner.hpp"
#include "surface_prior_probe_io.hpp"
#include "ping_optimizer.hpp"
#include "surface_protocol.hpp"
#include <iostream>
namespace io=posim_slam::surface_prior_probe;
namespace sp=posim_slam::surface_prior;
using namespace posim_slam;
using Json=nlohmann::json;
Json variables(const std::vector<sp::Variable>& order) {
  Json out=Json::array();for(const auto& v:order)out.push_back({{"kind",v.kind==sp::Kind::State ? "state" : (v.kind==sp::Kind::Plane ? "plane" : "pressure")},
    {"ident",v.id},{"size",v.size},{"reference",io::probe::jsonVector(v.reference)}});return out;
}
Json node(const RetiredPoseConditional& n) {
  return {{"id",n.id},{"variables",variables(n.variables)},{"kept_indices",n.kept},{"removed_indices",n.removed},
    {"regression",io::probe::jsonMatrix(n.regression)},{"offset",io::probe::jsonVector(n.offset)},
    {"noise_factor",io::probe::jsonMatrix(n.noise)},{"unbounded_basis",io::probe::jsonMatrix(n.unbounded)},
    {"marginal_information_relative_error",n.marginal_information_relative_error},
    {"marginal_gradient_relative_error",n.marginal_gradient_relative_error}};
}
RetiredPoseConditional readNode(const Json& v) {
  RetiredPoseConditional n;n.id=v.at("id");n.variables=io::order(v.at("variables"),true);
  n.kept=v.at("kept_indices").get<std::vector<int>>();n.removed=v.at("removed_indices").get<std::vector<int>>();
  n.regression=io::rows(v.at("regression"),n.kept.size());n.offset=io::vector(v.at("offset"));
  n.noise=io::probe::matrix(v.at("noise_factor"));n.unbounded=io::probe::matrix(v.at("unbounded_basis"));return n;
}
void addState(basalt::BundleAdjustmentBase<double>& owner,const sp::Variable& v) {
  if(v.kind!=sp::Kind::State)throw std::invalid_argument("probe_state_required");
  const Eigen::Map<const Eigen::Matrix<double,3,3,Eigen::RowMajor>> r(v.reference.data()+3);
  const Sophus::SE3d pose(r,v.reference.head<3>());
  if(v.size==6)owner.frame_poses.emplace(v.id,basalt::PoseStateWithLin<double>(v.id,pose,true));
  else owner.frame_states.emplace(v.id,basalt::PoseVelBiasStateWithLin<double>(v.id,pose,v.reference.segment<3>(12),
      v.reference.segment<3>(15),v.reference.segment<3>(18),true));
}
int main() {
  basalt::BundleAdjustmentBase<double> owner;owner.huber_thresh=1;owner.obs_std_dev=1;
  SurfaceMarginalOwner surface;basalt::MargLinData<double> legacy;
  std::unique_ptr<PressureFactorStore> pressure;DvlFactorStore dvl(DvlCalibration{});
  std::map<int64_t,basalt::IntegratedImuMeasurement<double>> measurements;
  const Vec3 gravity(0,0,-9.80665),weight=Vec3::Ones();
  std::optional<SurfaceMarginalOwner::Snapshot> saved;
  auto order=[&] {return planPingMarginalLayout(owner,{},{},{},{}).source;};
  auto imu=[&] {basalt::ImuLinData<double> data{gravity,weight,weight,{}};
    for(const auto& [stamp,m]:measurements)data.imu_meas.emplace(stamp,&m);return data;};
  auto intervals=[&](const Json& values) {
    for(const auto& v:values) {
      const int64_t start=v.at(0),end=v.at(1);
      basalt::IntegratedImuMeasurement<double> m(start,Vec3::Zero(),Vec3::Zero());
      basalt::ImuData<double> sample;sample.t_ns=end;sample.accel=Vec3(0,0,9.80665);sample.gyro.setZero();
      m.integrateContinuous(sample,Vec3::Constant(.04),Vec3::Constant(.0001));
      measurements.emplace(start,std::move(m));
    }
  };
  auto result=[&] {
    const auto current=order();const auto coordinates=surface.coordinates(owner,current,*pressure);
    const auto marginal=surface.linearize(owner,current,*pressure);
    const auto depth=pressure->linearize(owner,current);const auto velocity=dvl.linearize(owner,current);
    const auto active_imu=imu();PingLinearization::Options options;
    options.lb_options.huber_parameter=owner.huber_thresh;options.lb_options.obs_std_dev=owner.obs_std_dev;
    basalt::LinearizationAbsQR<double,6> active(&owner,current,options,&legacy,&active_imu);
    bool valid=false;active.linearizeProblem(&valid);if(!valid)throw std::invalid_argument("probe_active_imu_rows");
    active.performQR();Eigen::MatrixXd ji;Eigen::VectorXd ri;active.get_dense_Q2Jp_Q2r(ji,ri);
    Eigen::MatrixXd j=Eigen::MatrixXd::Zero(marginal.jacobian.rows()+depth.jacobian.rows()+velocity.jacobian.rows()+ji.rows(),coordinates.delta.size());
    Eigen::VectorXd r(j.rows());j.topRows(marginal.jacobian.rows())=marginal.jacobian;
    j.block(marginal.jacobian.rows(),0,depth.jacobian.rows(),depth.jacobian.cols())=depth.jacobian;
    j.block(marginal.jacobian.rows()+depth.jacobian.rows(),0,velocity.jacobian.rows(),velocity.jacobian.cols())=velocity.jacobian;
    j.bottomLeftCorner(ji.rows(),ji.cols())=ji;
    r<<marginal.residual,depth.residual,velocity.residual,ri;
    Json nodes=Json::array();for(const auto& [id,n]:surface.state().nodes)nodes.push_back(node(*n));
    Json data={{"prior",io::write(surface.state().prior)},{"order",variables(coordinates.order)},
      {"delta",io::probe::jsonVector(coordinates.delta)},{"joint_jacobian",io::probe::jsonMatrix(j)},
      {"joint_residual",io::probe::jsonVector(r)},{"conditionals",nodes},
      {"pressure_count",pressure->size()},{"dvl_count",dvl.size()},{"imu_count",measurements.size()},
      {"legacy_zero",legacy.H.isZero(0) && legacy.b.isZero(0) && !pressure->prior().active},
      {"owner_revision",owner.posim_ping_state_revision}};
    try {data["information"]=surface_protocol::information(nativeSurfaceInformation(owner,current,legacy,active_imu,&dvl,*pressure,surface));}
    catch(const std::exception& error) {data["information_error"]=error.what();}
    return data;
  };
  Json input;while(std::cin>>input) {
    Json out;
    try {
      const auto operation=input.at("op").get<std::string>();
      if(operation=="init") {
        for(const auto& v:io::order(input.at("states")))addState(owner,v);
        const auto current=order();legacy=SurfaceMarginalOwner::zeroPrior(current);
        if(input.contains("legacy_j")) {legacy.H=io::rows(input.at("legacy_j"),current.total_size);legacy.b=io::vector(input.at("legacy_b"));}
        PressureCalibration calibration;if(input.contains("pressure_lever"))calibration.imu_to_pressure=io::probe::vector<3>(input.at("pressure_lever"));
        pressure=std::make_unique<PressureFactorStore>(calibration);
        for(const auto& p:input.value("pressure",Json::array()))pressure->add({p.at(0),p.at(1),p.at(2)},owner);
        for(const auto& v:input.value("dvl",Json::array()))dvl.add({v.at("stamp"),io::probe::vector<3>(v.at("velocity")),Vec3::Zero(),
          v.value("variance",.04)*Mat3::Identity(),Mat3::Zero()});
        intervals(input.value("imu_intervals",Json::array()));
        if(input.contains("pressure_prior")) {
          const auto& p=input.at("pressure_prior");PressureDatumPrior prior;
          prior.order=current;prior.states=io::vector(p.at("states")).transpose();prior.datum=p.at("datum");
          prior.b=p.at("b");prior.reference=p.at("reference");prior.active=true;
          pressure->retireWith(pressure->planRetirement({}),prior,[]{});
        }
        surface.adopt(owner,current,legacy,*pressure,"native-surface-owner");
      } else if(operation=="snapshot")saved=surface.snapshot(owner,order(),legacy,*pressure);
      else if(operation=="append") {
        for(const auto& v:io::order(input.at("states")))addState(owner,v);
        for(const auto& p:input.value("pressure",Json::array()))pressure->add({p.at(0),p.at(1),p.at(2)},owner);
        for(const auto& v:input.value("dvl",Json::array()))dvl.add({v.at("stamp"),io::probe::vector<3>(v.at("velocity")),Vec3::Zero(),
          v.value("variance",.04)*Mat3::Identity(),Mat3::Zero()});
        intervals(input.value("imu_intervals",Json::array()));
      } else if(operation=="late" || operation=="trial_reject") {
        const auto current=order();const auto expected=saved ? *saved : surface.snapshot(owner,current,legacy,*pressure);
        const auto plane_input=input.value("new_planes",Json::array());
        const auto planes=plane_input.empty() ? std::vector<sp::Variable>{} : io::order(plane_input);
        std::vector<RetiredPoseConditional> replacements;for(const auto& v:input.value("conditionals",Json::array()))replacements.push_back(readNode(v));
        auto prepared=surface.prepareLate(owner,current,legacy,*pressure,expected,
          io::rows(input.at("jacobian"),expected.coordinates.delta.size()+3*planes.size()),io::vector(input.at("offset")),planes,
          io::order(input.at("source_order"),true),replacements,io::keys(input.at("keys")));
        if(operation=="late") {surface.install(owner,current,legacy,*pressure,std::move(prepared));saved.reset();}
        else {
          JointPingTrial trial(owner);PressureFactorStore::Trial depth(*pressure);depth.apply(.1);
          Eigen::Matrix<double,15,1> inc=Eigen::Matrix<double,15,1>::Zero();inc[2]=.1;
          if(!owner.frame_states.empty())owner.frame_states.begin()->second.applyInc(inc);
          else owner.frame_poses.begin()->second.applyInc(inc.head<6>());
          for(const auto& v:surface.state().prior.order)if(v.kind==sp::Kind::Plane)owner.posim_ping_planes.apply(int(v.id),Vec3(.01,0,.1));
          bool rejected=false;try{surface.install(owner,current,legacy,*pressure,std::move(prepared));}catch(const std::exception&){rejected=true;}
          out["installation_during_trial_rejected"]=rejected;
        }
      } else if(operation=="optimize") {
        const auto report=optimizePingGroups(owner,order(),{},legacy,imu(),{20,8,input.value("damping",.001)},nullptr,&dvl,pressure.get(),&surface);
        out["optimization"]={{"initial_cost",report.initial_cost},{"final_cost",report.final_cost},{"accepted",report.accepted},{"rejected",report.rejected}};
      } else if(operation=="retire") {
        surface.marginalize(owner,order(),legacy,measurements,imu(),input.at("removed").get<std::set<int64_t>>(),
          input.value("pose_only",std::set<int64_t>{}),&dvl,*pressure);saved.reset();
      } else if(operation=="duplicate_prior")legacy.H(0,0)=1;
      else throw std::invalid_argument("unknown_surface_owner_probe_operation");
    } catch(const std::exception& error) {out["error"]=error.what();}
    try {out["state"]=result();}catch(const std::exception& error){out["state_error"]=error.what();}
    std::cout<<out.dump()<<std::endl;
  }
}
