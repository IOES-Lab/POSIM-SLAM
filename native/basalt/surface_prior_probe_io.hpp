// SPDX-License-Identifier: Apache-2.0
#pragma once
#include "surface_prior.hpp"
#include "range_surface_probe_io.hpp"
namespace posim_slam::surface_prior_probe {
namespace probe=range_surface_probe;
namespace sp=surface_prior;
using Json=nlohmann::json;
inline sp::Kind kind(const std::string& name) {
  if(name=="state")return sp::Kind::State;if(name=="plane")return sp::Kind::Plane;
  if(name=="pressure")return sp::Kind::Pressure;
  throw std::invalid_argument("invalid_persistent_surface_variable");
}
inline Eigen::VectorXd vector(const Json& value) {
  if(!value.is_array() || value.size()>4096)throw std::invalid_argument("bounded_vector_required");
  Eigen::VectorXd out(value.size());for(Eigen::Index i=0;i<out.size();++i)out[i]=value.at(i).get<double>();return out;
}
inline std::vector<sp::Variable> order(const Json& value,bool retirement_source=false) {
  if(!value.is_array() || value.size()>(retirement_source ? kMaxLivePoseBlocks+9 : 25))throw std::invalid_argument("bounded_order_required");
  std::vector<sp::Variable> out;
  for(const auto& v:value)out.push_back({kind(v.at("kind").get<std::string>()),probe::integer(v.at("ident")),
    probe::integer(v.at("size")),vector(v.at("reference"))});
  sp::layout(out,retirement_source);return out;
}
inline std::vector<sp::RangeKey> keys(const Json& value) {
  if(!value.is_array() || value.size()>sp::maxRecentKeys)throw std::invalid_argument("bounded_range_keys_required");
  std::vector<sp::RangeKey> out;
  for(const auto& k:value) {
    const auto v=probe::key(k);out.emplace_back(v.stamp,v.sensor,v.beam);
  }
  return out;
}
inline Eigen::MatrixXd rows(const Json& value,Eigen::Index columns) {
  if(!value.is_array() || value.size()>4096)throw std::invalid_argument("bounded_rows_required");
  Eigen::MatrixXd out(value.size(),columns);
  for(Eigen::Index i=0;i<out.rows();++i) {
    if(!value.at(i).is_array() || value.at(i).size()!=size_t(columns))throw std::invalid_argument("surface_row_shape");
    for(Eigen::Index j=0;j<columns;++j)out(i,j)=value.at(i).at(j).get<double>();
  }
  return out;
}
inline sp::Prior prior(const Json& value) {
  sp::Prior out;out.order=order(value.at("order"));out.jacobian=rows(value.at("jacobian"),sp::layout(out.order).count);
  out.offset=vector(value.at("offset"));out.ledger.recent=sp::keys(keys(value.at("ledger").at("recent")));
  out.ledger.closed_through=probe::integer(value.at("ledger").at("closed_through_ns"));
  out.epoch=value.at("epoch").get<std::string>();out.revision=probe::integer(value.at("revision"));out.requireValid();return out;
}
inline Json write(const sp::Prior& p) {
  Json variables=Json::array(),measurements=Json::array();
  for(const auto& v:p.order)variables.push_back({{"kind",v.kind==sp::Kind::State ? "state" : (v.kind==sp::Kind::Plane ? "plane" : "pressure")},
    {"ident",v.id},{"size",v.size},{"reference",probe::jsonVector(v.reference)}});
  for(const auto& k:p.ledger.recent)measurements.push_back({std::get<0>(k),std::get<1>(k),std::get<2>(k)});
  return {{"order",variables},{"jacobian",probe::jsonMatrix(p.jacobian)},{"offset",probe::jsonVector(p.offset)},
    {"ledger",{{"recent",measurements},{"closed_through_ns",p.ledger.closed_through}}},
    {"epoch",p.epoch},{"revision",p.revision}};
}
}  // namespace posim_slam::surface_prior_probe
