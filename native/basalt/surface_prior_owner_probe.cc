// SPDX-License-Identifier: Apache-2.0
#include "surface_prior_owner.hpp"
#include "surface_prior_probe_io.hpp"
#include <iostream>
namespace io=posim_slam::surface_prior_probe;
namespace sp=posim_slam::surface_prior;
using Json=nlohmann::json;
int main() {
  try {
    Json input;std::cin>>input;const auto expected=io::order(input.at("order"));const auto delta=io::vector(input.at("delta"));
    const auto epoch=input.at("epoch").get<std::string>();
    if(delta.size()!=sp::layout(expected).count)throw std::invalid_argument("owner_delta_shape");
    basalt::BundleAdjustmentBase<double> owner;std::optional<sp::ScalarCoordinate> pressure;Eigen::Index column=0;
    for(const auto& v:expected) {
      if(v.kind==sp::Kind::State) {
        const Eigen::Map<const Eigen::Matrix<double,3,3,Eigen::RowMajor>> r(v.reference.data()+3);
        const Sophus::SE3d pose(r,v.reference.head<3>());
        if(v.size==6) {
          owner.frame_poses.emplace(v.id,basalt::PoseStateWithLin<double>(v.id,pose,true));
          owner.frame_poses.at(v.id).applyInc(delta.segment<6>(column));
        } else {
          owner.frame_states.emplace(v.id,basalt::PoseVelBiasStateWithLin<double>(v.id,pose,v.reference.segment<3>(12),
            v.reference.segment<3>(15),v.reference.segment<3>(18),true));
          owner.frame_states.at(v.id).applyInc(delta.segment<15>(column));
        }
      } else if(v.kind==sp::Kind::Plane) {
        if(v.id>std::numeric_limits<int>::max())throw std::invalid_argument("plane_id");
        owner.posim_ping_planes.set(int(v.id),v.reference.head<3>(),v.reference[3]);
        owner.posim_ping_planes.apply(int(v.id),delta.segment<3>(column));
      } else pressure=sp::ScalarCoordinate{v.id,v.reference[0],v.reference[0]+delta[column]};
      column+=v.size;
    }
    if(input.contains("remove_state")) {
      const int64_t stamp=input.at("remove_state");owner.frame_poses.erase(stamp);owner.frame_states.erase(stamp);
    }
    if(input.contains("change_pose_reference")) {
      const int64_t stamp=input.at("change_pose_reference");const auto current=owner.frame_poses.at(stamp);
      auto reference=current.getPoseLin();reference.translation().x()+=.1;
      owner.frame_poses.at(stamp)=basalt::PoseStateWithLin<double>(stamp,reference,true);
      owner.frame_poses.at(stamp).applyInc(current.getDelta());
    }
    const auto coordinates=sp::ownedCoordinates(owner,expected,pressure);Json result;
    result["owner_delta"]=io::probe::jsonVector(coordinates.delta);
    std::vector<int64_t> states;for(const auto& v:coordinates.order)if(v.kind==sp::Kind::State)states.push_back(v.id);
    result["resident_pose_ids"]=states;
    std::optional<sp::Prior> previous;
    if(input.contains("previous")) {
      previous=io::prior(input.at("previous"));const auto rows=sp::linearize(*previous,coordinates.order,coordinates.delta,epoch);
      result["prior_jacobian"]=io::probe::jsonMatrix(rows.jacobian);result["prior_residual"]=io::probe::jsonVector(rows.residual);
      Eigen::MatrixXd numerical(rows.jacobian.rows(),rows.jacobian.cols());constexpr double epsilon=1e-6;
      for(Eigen::Index k=0;k<numerical.cols();++k) {
        Eigen::VectorXd values[2];
        for(int sign=0;sign<2;++sign) {
          owner.backup();const auto saved_pressure=pressure;const double step=(sign ? 1. : -1.)*epsilon;Eigen::Index start=0;
          for(const auto& v:expected) {
            if(k>=start && k<start+v.size) {
              Eigen::VectorXd increment=Eigen::VectorXd::Zero(v.size);increment[k-start]=step;
              if(v.kind==sp::Kind::State) {
                if(v.size==6)owner.frame_poses.at(v.id).applyInc(increment);
                else owner.frame_states.at(v.id).applyInc(increment);
              } else if(v.kind==sp::Kind::Plane)owner.posim_ping_planes.apply(int(v.id),increment);
              else pressure->value+=step;
            }
            start+=v.size;
          }
          const auto changed=sp::ownedCoordinates(owner,expected,pressure);
          values[sign]=sp::linearize(*previous,changed.order,changed.delta,epoch).residual;
          owner.restore();pressure=saved_pressure;
        }
        numerical.col(k)=(values[1]-values[0])/(2*epsilon);
      }
      result["maximum_owner_derivative_error"]=(numerical-rows.jacobian).cwiseAbs().maxCoeff();
    }
    std::vector<sp::Retain> retain;
    for(const auto& v:input.at("retain"))retain.push_back({io::kind(v.at(0).get<std::string>()),v.at(1).get<int64_t>(),v.at(2).get<Eigen::Index>()});
    std::optional<int64_t> cutoff;if(input.contains("close_through_ns"))cutoff=io::probe::integer(input.at("close_through_ns"));
    const auto current=sp::marginalize(coordinates.order,coordinates.delta,io::rows(input.at("jacobian"),delta.size()),io::vector(input.at("residual")),
      retain,epoch,input.contains("range_keys") ? io::keys(input.at("range_keys")) : std::vector<sp::RangeKey>{},previous ? &*previous : nullptr,cutoff);
    result["prior"]=io::write(current);result["live_worker_updated"]=false;
    std::cout<<result.dump()<<'\n';return 0;
  } catch(const std::exception& error) {std::cerr<<error.what()<<'\n';return 2;}
}
