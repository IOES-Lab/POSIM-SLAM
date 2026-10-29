// SPDX-License-Identifier: Apache-2.0
#pragma once
#include "surface_owner.hpp"
#include "surface_information.hpp"
#include <nlohmann/json.hpp>
#include <filesystem>
#include <fstream>

namespace posim_slam::surface_protocol {
using Json=nlohmann::json;
namespace sp=surface_prior;namespace fs=std::filesystem;
constexpr uint64_t maxFileBytes=128*1024*1024;
inline int64_t integer(const Json& value,int64_t minimum=0) {
  if(!value.is_number_integer() || (value.is_number_unsigned() && value.get<uint64_t>()>uint64_t(INT64_MAX)))
    throw std::invalid_argument("surface_protocol_integer");
  const auto result=value.get<int64_t>();if(result<minimum)throw std::invalid_argument("surface_protocol_integer");return result;
}
inline double number(const Json& value) {
  if(!value.is_number())throw std::invalid_argument("surface_protocol_number");
  const double out=value.get<double>();if(!std::isfinite(out))throw std::invalid_argument("surface_protocol_number");return out;
}
inline Json vector(const Eigen::VectorXd& value) {
  Json out=Json::array();for(Eigen::Index i=0;i<value.size();++i)out.push_back(value[i]);return out;
}
inline Json matrix(const Eigen::MatrixXd& value) {
  Json out=Json::array();for(Eigen::Index i=0;i<value.rows();++i)out.push_back(vector(value.row(i).transpose()));return out;
}
inline Json variables(const std::vector<sp::Variable>& order) {
  Json out=Json::array();for(const auto& v:order)out.push_back({{"kind",v.kind==sp::Kind::State ? "state" : (v.kind==sp::Kind::Plane ? "plane" : "pressure")},
    {"ident",v.id},{"size",v.size},{"reference",vector(v.reference)}});return out;
}
inline Json information(const NativeSurfaceInformation& rows) {
  return {{"schema","posim-slam-native-surface-information-1"},
    {"scope","local_frozen_weight_gauss_newton"},{"jacobian",matrix(rows.jacobian)},
    {"residual",vector(rows.residual)},{"imu_intervals",rows.imu_intervals},
    {"dvl_observations",rows.dvl_observations},{"pressure_observations",rows.pressure_observations}};
}
struct Decoder {
  uint64_t numeric_bytes=0;
  void reserve(uint64_t count) {
    if(count>(40*1024*1024-numeric_bytes)/8)throw std::invalid_argument("surface_protocol_numeric_capacity");numeric_bytes+=count*8;
  }
  Eigen::VectorXd vector(const Json& value) {
    if(!value.is_array() || value.size()>4096)throw std::invalid_argument("surface_protocol_vector");
    reserve(value.size());Eigen::VectorXd out(value.size());for(Eigen::Index i=0;i<out.size();++i)out[i]=number(value.at(i));return out;
  }
  Eigen::MatrixXd rows(const Json& value,Eigen::Index columns,Eigen::Index maximum_rows=4096) {
    if(!value.is_array() || value.size()>size_t(maximum_rows) || columns<0 || columns>1024)
      throw std::invalid_argument("surface_protocol_matrix");
    reserve(value.size()*size_t(columns));Eigen::MatrixXd out(value.size(),columns);
    for(Eigen::Index i=0;i<out.rows();++i) {
      const auto& row=value.at(i);if(!row.is_array() || row.size()!=size_t(columns))throw std::invalid_argument("surface_protocol_matrix");
      for(Eigen::Index j=0;j<columns;++j)out(i,j)=number(row.at(j));
    }
    return out;
  }
  Eigen::MatrixXd matrix(const Json& value) {
    if(!value.is_array() || (!value.empty() && !value.at(0).is_array()))throw std::invalid_argument("surface_protocol_matrix");
    return rows(value,value.empty() ? 0 : value.at(0).size(),1024);
  }
  std::vector<sp::Variable> variables(const Json& value,bool source=false,bool empty=false) {
    if(!value.is_array() || value.size()>82 || (!empty && value.empty()))throw std::invalid_argument("surface_protocol_variables");
    std::vector<sp::Variable> out;
    for(const auto& v:value) {
      const auto name=v.at("kind").get<std::string>();sp::Kind kind;
      if(name=="state")kind=sp::Kind::State;else if(name=="plane")kind=sp::Kind::Plane;
      else if(name=="pressure")kind=sp::Kind::Pressure;else throw std::invalid_argument("surface_protocol_variable_kind");
      out.push_back({kind,integer(v.at("ident")),integer(v.at("size"),1),vector(v.at("reference"))});
    }
    if(!out.empty())sp::layout(out,source);return out;
  }
  std::vector<int> indices(const Json& value) {
    if(!value.is_array() || value.size()>1024)throw std::invalid_argument("surface_protocol_indices");
    std::vector<int> out;for(const auto& v:value) {const auto n=integer(v);if(n>1024)throw std::invalid_argument("surface_protocol_indices");out.push_back(int(n));}return out;
  }
  std::vector<RetiredPoseConditional> conditionals(const Json& value) {
    if(!value.is_array() || value.size()>256)throw std::invalid_argument("surface_protocol_conditionals");
    std::vector<RetiredPoseConditional> out;
    for(const auto& v:value) {
      RetiredPoseConditional n;n.id=integer(v.at("id"),1);n.variables=variables(v.at("variables"),true);
      n.kept=indices(v.at("kept_indices"));n.removed=indices(v.at("removed_indices"));
      n.regression=rows(v.at("regression"),n.kept.size(),1024);n.offset=vector(v.at("offset"));
      n.noise=matrix(v.at("noise_factor"));n.unbounded=matrix(v.at("unbounded_basis"));out.push_back(std::move(n));
    }
    return out;
  }
  std::vector<sp::RangeKey> keys(const Json& value) {
    if(!value.is_array() || value.size()>sp::maxRecentKeys)throw std::invalid_argument("surface_protocol_keys");
    std::vector<sp::RangeKey> out;
    for(const auto& key:value) {
      if(!key.is_array() || key.size()!=3)throw std::invalid_argument("surface_protocol_keys");
      const auto sensor=integer(key.at(1)),beam=integer(key.at(2));
      if(sensor>INT_MAX || beam>INT_MAX)throw std::invalid_argument("surface_protocol_keys");
      out.emplace_back(integer(key.at(0),1),int(sensor),int(beam));
    }
    return out;
  }
};
inline Json conditional(const RetiredPoseConditional& n) {
  return {{"id",n.id},{"variables",variables(n.variables)},{"kept_indices",n.kept},{"removed_indices",n.removed},
    {"regression",matrix(n.regression)},{"offset",vector(n.offset)},{"noise_factor",matrix(n.noise)},{"unbounded_basis",matrix(n.unbounded)},
    {"marginal_information_relative_error",n.marginal_information_relative_error},{"marginal_gradient_relative_error",n.marginal_gradient_relative_error}};
}

// One pending snapshot and one per-process directory. Bulk matrices never pass
// through the ordinary line-message or raw-sensor input queues.
class Controller {
 public:
  Controller(const fs::path& directory,const fs::path& calibration_directory):directory_(fs::canonical(directory)) {
    if(fs::is_symlink(directory) || directory_.parent_path()!=fs::canonical(calibration_directory) ||
        directory_.filename().string().rfind("worker-surface-",0)!=0 || !fs::is_directory(directory_))
      throw std::invalid_argument("owned_surface_directory_required");
  }
  template<class Optimize,class Information> Json handle(SurfaceMarginalOwner& surface,basalt::BundleAdjustmentBase<double>& owner,
      const basalt::AbsOrderMap& order,basalt::MargLinData<double>& legacy,PressureFactorStore& pressure,
      const Json& command,Optimize optimize,Information current_information) {
    const auto request=integer(command.at("request_id"),1);const auto op=command.at("op").get<std::string>();
    Json reply={{"type","surface_control"},{"request_id",request},{"op",op},{"accepted",false},{"map_qualified",false}};
    try {
      if(!surface.active())throw std::invalid_argument("surface_owner_not_ready");
      if(op=="snapshot") {
        auto captured=surface.snapshot(owner,order,legacy,pressure);
        if(next_id_==uint64_t(INT64_MAX))throw std::invalid_argument("surface_snapshot_id_overflow");
        clear();pending_id_=++next_id_;pending_=std::move(captured);
        const auto& s=*pending_;const auto& state=*s.state;Json nodes=Json::array(),keys=Json::array();
        for(const auto& [id,n]:state.nodes)nodes.push_back(conditional(*n));
        for(const auto& key:state.prior.ledger.recent)keys.push_back({std::get<0>(key),std::get<1>(key),std::get<2>(key)});
        Json packet={{"schema","posim-slam-native-surface-snapshot-1"},{"snapshot_id",pending_id_},
          {"owner_revision",s.owner_revision},{"prior_revision",state.prior.revision},{"epoch",state.prior.epoch},
          {"stamp_ns",order.abs_order_map.rbegin()->first},{"variables",variables(s.coordinates.order)},{"delta",vector(s.coordinates.delta)},
          {"conditionals",nodes},{"archive_closed_through_ns",state.archive_closed_through},{"protected_states",state.protected_states},
          {"range_ledger",{{"recent",keys},{"closed_through_ns",state.prior.ledger.closed_through}}},
          {"information",information(current_information())}};
        snapshot_file_=directory_/("worker-surface-snapshot-"+std::to_string(pending_id_)+".json");
        const auto temporary=fs::path(snapshot_file_.string()+".tmp");
        if(fs::exists(snapshot_file_) || fs::exists(temporary))throw std::invalid_argument("surface_snapshot_path_exists");
        std::ofstream file(temporary,std::ios::binary|std::ios::trunc);file<<packet.dump();file.flush();
        if(!file || file.tellp()<0 || uint64_t(file.tellp())>maxFileBytes)throw std::invalid_argument("surface_snapshot_file_capacity");
        file.close();fs::rename(temporary,snapshot_file_);
        reply["file"]={{"schema","posim-slam-native-surface-file-1"},{"path",snapshot_file_.string()},
          {"snapshot_id",pending_id_},{"bytes",fs::file_size(snapshot_file_)},{"epoch",state.prior.epoch},{"stamp_ns",packet.at("stamp_ns")}};
      } else if(op=="update") {
        if(!pending_)throw std::invalid_argument("native_surface_snapshot_required");
        const auto value=readUpdate(command,request);
        if(value.at("schema")!="posim-slam-native-surface-increment-1" || value.at("epoch")!=pending_->state->prior.epoch ||
            uint64_t(integer(value.at("snapshot_id"),1))!=pending_id_ || uint64_t(integer(value.at("owner_revision")))!=pending_->owner_revision ||
            integer(value.at("prior_revision"))!=pending_->state->prior.revision || integer(value.at("stamp_ns"),1)!=order.abs_order_map.rbegin()->first)
          throw std::invalid_argument("native_surface_snapshot_identity_mismatch");
        Decoder decoder;auto planes=decoder.variables(value.at("new_planes"),false,true);
        const auto columns=pending_->coordinates.delta.size()+3*planes.size();
        auto rows=decoder.rows(value.at("jacobian"),columns,columns+1);auto offset=decoder.vector(value.at("offset"));
        auto source=decoder.variables(value.at("source_order"),true);auto nodes=decoder.conditionals(value.at("conditionals"));
        const auto keys=decoder.keys(value.at("range_keys"));
        auto prepared=surface.prepareLate(owner,order,legacy,pressure,*pending_,rows,offset,planes,source,nodes,keys);
        surface.install(owner,order,legacy,pressure,std::move(prepared));clear();
        reply["accepted"]=true;
        try {optimize();reply["optimized"]=true;}
        catch(const std::exception& error) {reply["optimized"]=false;reply["optimization_error"]=error.what();}
      } else if(op=="expire") {
        std::set<int64_t> protect;const auto& values=command.at("protected_states");
        if(!values.is_array() || values.size()>73)throw std::invalid_argument("bounded_surface_protected_states_required");
        for(const auto& v:values)if(!protect.insert(integer(v,1)).second)throw std::invalid_argument("unique_surface_protected_states_required");
        surface.expireThrough(owner,order,legacy,pressure,integer(command.at("closed_through_ns")),protect);clear();
      } else if(op=="release")clear();
      else throw std::invalid_argument("unknown_surface_control");
      reply["accepted"]=true;reply["prior_revision"]=surface.state().prior.revision;
      reply["conditional_nodes"]=surface.state().nodes.size();reply["ledger_entries"]=surface.state().prior.ledger.recent.size();
      reply["owner_revision"]=owner.posim_ping_state_revision;
    } catch(const std::exception& error) {reply["reason"]=error.what();}
    return reply;
  }
 private:
  fs::path directory_,snapshot_file_;std::optional<SurfaceMarginalOwner::Snapshot> pending_;
  uint64_t next_id_=0,pending_id_=0;
  void clear() {
    pending_.reset();pending_id_=0;
    if(!snapshot_file_.empty()) {
      std::error_code error;fs::remove(snapshot_file_,error);fs::remove(snapshot_file_.string()+".tmp",error);snapshot_file_.clear();
    }
  }
  Json readUpdate(const Json& command,int64_t request) const {
    const fs::path path=command.at("path").get<std::string>();const auto bytes=integer(command.at("bytes"),1);
    if(uint64_t(bytes)>maxFileBytes || path.filename()!="worker-surface-update-"+std::to_string(request)+".json" ||
        fs::canonical(path.parent_path())!=directory_ || fs::is_symlink(path) || !fs::is_regular_file(path) || fs::file_size(path)!=uint64_t(bytes))
      throw std::invalid_argument("owned_surface_update_file_required");
    const auto time=fs::last_write_time(path);std::ifstream file(path,std::ios::binary);
    std::string data(size_t(bytes),'\0');file.read(data.data(),bytes);
    if(!file || file.peek()!=std::char_traits<char>::eof() || fs::file_size(path)!=uint64_t(bytes) || fs::last_write_time(path)!=time)
      throw std::invalid_argument("surface_update_file_changed");
    return Json::parse(data);
  }
};
}  // namespace posim_slam::surface_protocol
