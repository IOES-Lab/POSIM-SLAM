// SPDX-License-Identifier: Apache-2.0
#pragma once
// Shared diagnostic JSON boundary; not part of the streaming protocol.
#include "range_surface.hpp"
#include <nlohmann/json.hpp>
#include <limits>
namespace posim_slam::range_surface_probe {
using Json=nlohmann::json;
namespace sr=surface_ranges;
template<int N> Eigen::Matrix<double,N,1> vector(const Json& value) {
  if(!value.is_array() || value.size()!=N)throw std::invalid_argument("invalid_vector");
  Eigen::Matrix<double,N,1> result;
  for(int i=0;i<N;++i)result[i]=value.at(i).get<double>();
  if(!result.allFinite())throw std::invalid_argument("invalid_vector");return result;
}
inline Eigen::MatrixXd matrix(const Json& value) {
  if(!value.is_array() || value.empty() || !value.at(0).is_array())throw std::invalid_argument("invalid_matrix");
  const auto rows=value.size(),cols=value.at(0).size();
  if(rows>512 || cols>512)throw std::invalid_argument("bounded_matrix_required");
  Eigen::MatrixXd result(rows,cols);
  for(size_t i=0;i<rows;++i) {
    if(!value.at(i).is_array() || value.at(i).size()!=cols)throw std::invalid_argument("invalid_matrix");
    for(size_t j=0;j<cols;++j)result(i,j)=value.at(i).at(j).get<double>();
  }
  return result;
}
inline sr::M3 rotation(const Json& value) {
  auto m=matrix(value);if(m.rows()!=3 || m.cols()!=3 || !sr::rotation(m))throw std::invalid_argument("invalid_rotation");return m;
}
inline int64_t integer(const Json& value) {
  if(!value.is_number_integer())throw std::invalid_argument("integer_identity_required");return value.get<int64_t>();
}
inline int64_t identity(const std::string& text) {
  size_t end=0;const int64_t value=std::stoll(text,&end);
  if(end!=text.size())throw std::invalid_argument("integer_identity_required");return value;
}
inline int smallIdentity(int64_t value) {
  if(value<0 || value>std::numeric_limits<int>::max())throw std::invalid_argument("invalid_surface_identity");return int(value);
}
inline sr::Key key(const Json& value) {
  if(!value.is_array() || value.size()!=3)throw std::invalid_argument("invalid_surface_source_key");
  return {integer(value.at(0)),smallIdentity(integer(value.at(1))),smallIdentity(integer(value.at(2)))};
}
inline Json jsonMatrix(const Eigen::MatrixXd& value) {
  Json out=Json::array();
  for(Eigen::Index i=0;i<value.rows();++i) {
    Json row=Json::array();for(Eigen::Index j=0;j<value.cols();++j)row.push_back(value(i,j));out.push_back(row);
  }
  return out;
}
inline Json jsonVector(const Eigen::VectorXd& value) {
  Json out=Json::array();for(Eigen::Index i=0;i<value.size();++i)out.push_back(value[i]);return out;
}
struct Input {
  std::map<int64_t,sr::Pose> poses;std::map<int,sr::Plane> planes;
  std::vector<sr::Observation> observations;Eigen::MatrixXd covariance;
  std::string uncertainty;bool require_support;
};
inline Input read(const Json& input) {
    std::map<int64_t,sr::Pose> poses;std::map<int,sr::Plane> planes;std::vector<sr::Observation> observations;
    for(const auto& item:input.at("states").items()) {
      const auto& s=item.value();sr::Pose p;p.position=vector<3>(s.at("position"));
      if(s.contains("rotation_reference")) {
        if(s.contains("attitude"))throw std::invalid_argument("complete_surface_pose_coordinates_required");
        const auto delta=vector<3>(s.at("rotation_delta"));
        p.attitude=sr::exp(delta)*rotation(s.at("rotation_reference"));p.coordinates=sr::leftJacobian(delta);
      } else {
        if(s.contains("rotation_delta"))throw std::invalid_argument("surface_pose_reference_required");
        p.attitude=rotation(s.at("attitude"));
      }
      poses.emplace(identity(item.key()),p);
    }
    for(const auto& item:input.at("planes").items()) {
      const auto& s=item.value();sr::Plane p;p.distance=s.at("distance").get<double>();
      if(s.contains("support_points"))throw std::invalid_argument("frozen_surface_support_points_not_allowed");
      if(s.contains("normal_reference")) {
        if(s.contains("normal"))throw std::invalid_argument("complete_surface_plane_coordinates_required");
        const auto reference=vector<3>(s.at("normal_reference"));const auto basis=sr::normalBasis(reference);
        const sr::V3 delta=basis*vector<2>(s.at("normal_delta"));
        p.normal=sr::exp(delta)*reference;p.coordinates=sr::leftJacobian(delta)*basis;
      } else {
        if(s.contains("normal_delta"))throw std::invalid_argument("surface_plane_reference_required");
        p.normal=vector<3>(s.at("normal"));p.coordinates=sr::normalBasis(p.normal);
      }
      if(s.contains("support_keys"))for(const auto& k:s.at("support_keys"))p.support_keys.push_back(key(k));
      p.max_plane_deviation=s.value("max_plane_deviation",0.);p.max_support_distance=s.value("max_support_distance",0.);
      planes.emplace(smallIdentity(identity(item.key())),p);
    }
    for(const auto& row:input.at("observations")) {
      sr::Observation o;o.key={integer(row.at("stamp_ns")),smallIdentity(integer(row.at("sensor_id"))),smallIdentity(integer(row.at("beam_id")))};
      const auto model=row.at("model").get<std::string>();
      if(model=="cone") {o.model=sr::Model::Cone;o.axis=vector<3>(row.at("beam_axis"));o.aperture=row.at("aperture_rad").get<double>();}
      else if(model=="fan") {o.model=sr::Model::Fan;o.azimuth=row.at("azimuth_rad").get<double>();o.vertical_fov=row.at("vertical_fov_rad").get<double>();}
      else throw std::invalid_argument("unknown_surface_range_model");
      const auto role=row.value("role",std::string("factor"));
      if(role=="support_only")o.role=sr::Role::SupportOnly;
      else if(role=="factor") {
        o.role=sr::Role::Factor;o.plane_id=smallIdentity(integer(row.at("plane_id")));
        o.surface_hypothesis=row.value("association",std::string())=="surface_hypothesis";
      } else throw std::invalid_argument("unknown_surface_row_role");
      o.lever=vector<3>(row.at("imu_to_sensor"));o.mount=rotation(row.at("sensor_to_imu"));
      o.measured=row.at("measured_range").get<double>();o.minimum=row.at("range_min").get<double>();o.maximum=row.at("range_max").get<double>();
      observations.push_back(o);
    }
    const bool support=input.value("require_support",true);
    return {poses,planes,observations,matrix(input.at("measurement_covariance")),
      input.at("uncertainty_source").get<std::string>(),support};
}
inline Json write(const sr::Block& out,bool support) {
    Json seeds=Json::object(),keys=Json::array();
    for(const auto& [id,points]:out.support_seeds)seeds[std::to_string(id)]=jsonMatrix(points);
    for(const auto& k:out.source_keys)keys.push_back({k.stamp,k.sensor,k.beam});
    const Json result={{"jacobian",jsonMatrix(out.jacobian)},{"residual",jsonVector(out.residual)},
      {"unwhitened_jacobian",jsonMatrix(out.raw_jacobian)},{"unwhitened_residual",jsonVector(out.raw_residual)},
      {"measurement_covariance",jsonMatrix(out.measurement_covariance)},{"predicted_ranges",jsonVector(out.predicted_ranges)},
      {"conditional_hits",jsonMatrix(out.conditional_hits)},{"support_seeds",seeds},{"source_keys",keys},
      {"state_stamps_ns",out.state_stamps},{"plane_ids",out.plane_ids},{"factor_indices",out.factor_indices},
      {"support_only_indices",out.support_only_indices},{"all_support_checked",support},
      {"map_qualified",false},{"physical_identity_verified",false}};
    return result;
}
}  // namespace posim_slam::range_surface_probe
