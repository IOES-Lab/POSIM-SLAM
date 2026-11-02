// SPDX-License-Identifier: Apache-2.0
#include "persistent_mapper.hpp"
#include <Eigen/Cholesky>
#include <chrono>
#include <numeric>

namespace posim_slam {
namespace {
MappingInformationResult information(basalt::NfrMapper& mapper,
    const basalt::Calibration<double>& calibration,const basalt::VioConfig& config,
    int64_t reference_stamp,double three_sigma_m) {
  const auto& poses=mapper.getFramePoses();
  std::map<int64_t,int> offsets;
  for(const auto& [stamp,pose]:poses)offsets.emplace(stamp,6*offsets.size());
  MappingInformation graph(6*offsets.size());
  auto columns=[&](int64_t stamp) {
    std::vector<int> cols(6);std::iota(cols.begin(),cols.end(),offsets.at(stamp));return cols;
  };
  for(const auto& factor:mapper.rel_pose_factors) {
    Sophus::Matrix6d first,second;
    basalt::relPoseError(factor.T_i_j,poses.at(factor.t_i_ns).getPose(),
                        poses.at(factor.t_j_ns).getPose(),&first,&second);
    Eigen::MatrixXd rows(6,12);rows<<first,second;
    Eigen::LLT<Eigen::MatrixXd> weight(factor.cov_inv);
    if(weight.info()!=Eigen::Success)throw std::runtime_error("mapping_relative_information_not_positive");
    auto cols=columns(factor.t_i_ns);const auto target=columns(factor.t_j_ns);
    cols.insert(cols.end(),target.begin(),target.end());
    graph.addPoseFactor("relative:"+std::to_string(factor.t_i_ns)+":"+std::to_string(factor.t_j_ns),
                        cols,weight.matrixU()*rows);
  }
  for(const auto& factor:mapper.roll_pitch_factors) {
    Eigen::Matrix<double,2,6> rows;
    basalt::rollPitchError(poses.at(factor.t_ns).getPose(),factor.R_w_i_meas,&rows);
    Eigen::LLT<Eigen::MatrixXd> weight(factor.cov_inv);
    if(weight.info()!=Eigen::Success)throw std::runtime_error("mapping_roll_pitch_information_not_positive");
    graph.addPoseFactor("roll-pitch:"+std::to_string(factor.t_ns),
                        columns(factor.t_ns),weight.matrixU()*rows);
  }
  for(const auto& [id,landmark]:mapper.lmdb.getLandmarks()) {
    const auto host=poses.at(landmark.host_kf_id.frame_id).getPose()*calibration.T_i_c.at(landmark.host_kf_id.cam_id);
    auto local=basalt::StereographicParam<double>::unproject(landmark.direction);local[3]=landmark.inv_dist;
    const auto homogeneous=(host*local).eval();
    const Eigen::Vector3d world=homogeneous.head<3>()/homogeneous[3];
    if(!world.allFinite() || landmark.inv_dist<=0)continue;
    struct Row {
      MappingObservation key;
      Eigen::Matrix<double,2,6> pose;
      Eigen::Matrix<double,2,3> point;
      bool full;
    };
    std::vector<Row> rows;
    for(const auto& [image,pixel]:landmark.obs) {
      const auto pose=poses.at(image.frame_id).getPose();
      const auto inverse=(pose*calibration.T_i_c.at(image.cam_id)).inverse();
      Eigen::Vector4d camera;camera.head<3>()=inverse*world;camera[3]=1;
      Eigen::Vector2d projected;Eigen::Matrix<double,2,4> derivative;
      if(!calibration.intrinsics.at(image.cam_id).project(camera,projected,&derivative) ||
          !projected.allFinite() || !derivative.allFinite() || camera.z()<=0)continue;
      const double error=(projected-pixel).norm();
      const double weight=std::sqrt(error<config.mapper_obs_huber_thresh?1:config.mapper_obs_huber_thresh/error)
                           /config.mapper_obs_std_dev;
      Row row;row.key={image.frame_id,image.cam_id,mapper.feature_tracks.at(id).at(image)};
      row.point=derivative.leftCols<3>()*inverse.rotationMatrix();
      row.pose.leftCols<3>()=-row.point;
      row.pose.rightCols<3>()=row.point*Sophus::SO3d::hat(world-pose.translation());
      row.point*=weight;row.pose*=weight;row.full=error<=config.mapper_obs_huber_thresh;
      rows.push_back(row);
    }
    if(rows.size()<2)continue;
    MappingPointRows point;point.id=id;point.world=world;point.host_origin=host.translation();
    point.pose=Eigen::MatrixXd::Zero(2*rows.size(),6*rows.size());point.point.resize(2*rows.size(),3);
    for(size_t i=0;i<rows.size();++i) {
      const auto cols=columns(std::get<0>(rows[i].key));
      point.pose_columns.insert(point.pose_columns.end(),cols.begin(),cols.end());
      point.observations.push_back(rows[i].key);point.full_weight.push_back(rows[i].full);
      point.pose.block<2,6>(2*i,6*i)=rows[i].pose;point.point.middleRows(2*i,2)=rows[i].point;
    }
    graph.addPoint(point);
  }
  // Reuse the estimator's original position/yaw reference once. No roll/pitch
  // prior, fixed pressure datum, LM damping or truth-derived reference is added.
  Eigen::MatrixXd reference=Eigen::MatrixXd::Zero(4,6);
  reference.topLeftCorner<3,3>().setIdentity();reference(3,5)=1;
  reference*=std::sqrt(config.vio_init_pose_weight);
  graph.addPoseFactor("initial-vio-world-reference",columns(reference_stamp),reference);
  return graph.evaluate(three_sigma_m);
}

} // namespace

PersistentMapper::PersistentMapper(const basalt::Calibration<double>& calibration,
    const basalt::VioConfig& config,double three_sigma_m,PersistentMapLimits limits)
    :calibration_(calibration),config_(MappingInputStore::configuration(config)),three_sigma_m_(three_sigma_m),limits_(limits),
     store_(calibration_,config_,limits),
     latest_(std::make_shared<PersistentMapSnapshot>()) {
  if(!std::isfinite(three_sigma_m) || three_sigma_m<=0 || three_sigma_m>1)
    throw std::invalid_argument("invalid_persistent_mapping_configuration");
  retained_.input=store_.snapshot();
  input.set_capacity(2);
  ingest_thread_=std::thread(&PersistentMapper::ingest,this);
  try {mapper_thread_=std::thread(&PersistentMapper::optimize,this);}
  catch(...) {input.push(nullptr);ingest_thread_.join();throw;}
}
PersistentMapper::~PersistentMapper(){finish();}
void PersistentMapper::finish() {
  {std::lock_guard<std::mutex> lock(mutex_);if(finished_)return;finished_=true;}
  input.push(nullptr);
  ingest_thread_.join();mapper_thread_.join();
}
void PersistentMapper::setReference(int64_t stamp,const Sophus::SE3d& pose) {
  if(stamp<=0 || !pose.matrix().allFinite())throw std::invalid_argument("invalid_mapping_reference");
  std::lock_guard<std::mutex> lock(mutex_);
  if(retained_.reference_stamp)throw std::invalid_argument("mapping_reference_already_set");
  retained_.reference_stamp=stamp;retained_.reference_pose=pose;changed_.notify_one();
}
uint64_t PersistentMapper::requestUpdate() {
  std::lock_guard<std::mutex> lock(mutex_);
  if(finished_ || !error_.empty())return 0;
  requested_revision_=std::max(requested_revision_,retained_.revision);
  changed_.notify_all();return requested_revision_;
}
void PersistentMapper::invalidate(const std::string& error) {
  std::lock_guard<std::mutex> lock(mutex_);
  if(!error_.empty())return;
  error_=error.empty()?"mapping_invalidated":error;
  auto failed=std::make_shared<PersistentMapSnapshot>(*latest_);
  failed->error=error_;failed->points.clear();failed->ids.clear();failed->observed=0;
  for(auto& point:failed->candidates){point.eligible=false;point.observed=false;}
  latest_=std::move(failed);
  changed_.notify_one();
}
PersistentMapper::Snapshot PersistentMapper::snapshot() const {
  std::lock_guard<std::mutex> lock(mutex_);return latest_;
}
void PersistentMapper::ingest() {
  for(;;) {
    basalt::MargData::Ptr source;input.pop(source);
    if(!source) {
      std::lock_guard<std::mutex> lock(mutex_);input_finished_=true;changed_.notify_one();return;
    }
    try {
      {std::lock_guard<std::mutex> lock(mutex_);if(!error_.empty())continue;}
      const auto committed=store_.append(*source);
      std::lock_guard<std::mutex> lock(mutex_);
      if(!error_.empty())continue;
      retained_.input=committed;retained_.revision=committed->revision;changed_.notify_one();
    } catch(const std::exception& error){invalidate(error.what());}
  }
}
void PersistentMapper::optimize() {
  uint64_t completed=0;
  for(;;) {
    Batch batch;
    {
      std::unique_lock<std::mutex> lock(mutex_);
      changed_.wait_for(lock,std::chrono::seconds(5),[&]{return input_finished_ || !error_.empty() ||
        (retained_.reference_stamp && (requested_revision_>completed ||
         retained_.revision>=completed+limits_.update_retirements));});
      if(!error_.empty())return;
      if(retained_.revision==completed){if(input_finished_)return;continue;}
      if(!retained_.reference_stamp) {
        if(!input_finished_)continue;
        lock.unlock();invalidate("mapping_initial_reference_missing");return;
      }
      batch=retained_;
    }
    try {
      auto result=build(batch);
      std::lock_guard<std::mutex> lock(mutex_);
      if(error_.empty())latest_=std::move(result);
      completed=batch.revision;
    } catch(const std::exception& error){invalidate(error.what());return;}
  }
}
std::shared_ptr<PersistentMapSnapshot> PersistentMapper::build(const Batch& batch) const {
  const auto started=std::chrono::steady_clock::now();
  auto restored=store_.makeMapper(batch.input);auto& mapper=*restored;
  if(!mapper.getFramePoses().count(batch.reference_stamp))
    throw std::runtime_error("mapping_pose_image_or_reference_mismatch");
  mapper.match_all();mapper.build_tracks();mapper.setup_opt();
  mapper.optimize(10);
  const auto quality=information(mapper,calibration_,config_,batch.reference_stamp,three_sigma_m_);
  const auto reference=mapper.getFramePoses().at(batch.reference_stamp).getPose();
  const Eigen::Matrix3d difference=batch.reference_pose.rotationMatrix()*reference.rotationMatrix().transpose();
  const double yaw=std::atan2(difference(1,0),difference(0,0));
  const Sophus::SO3d rotation=Sophus::SO3d::exp(Eigen::Vector3d(0,0,yaw));
  const Sophus::SE3d gauge(rotation,batch.reference_pose.translation()-rotation*reference.translation());
  auto result=std::make_shared<PersistentMapSnapshot>();
  result->revision=batch.revision;result->optimized_through_ns=batch.input->frames.rbegin()->first;
  result->keyframes=batch.input->frames.size();result->retirements=batch.input->revision;
  result->source_image_bytes=batch.input->source_image_bytes;
  result->source_factor_matrix_bytes=batch.input->source_factor_matrix_bytes;
  result->compact_input_bytes=batch.input->compactPayloadBytes();
  result->observations=quality.observations;result->factors=quality.pose_factors;
  for(const auto& [id,landmark]:mapper.lmdb.getLandmarks()) {
    const auto host=mapper.getFramePoses().at(landmark.host_kf_id.frame_id).getPose()
                        *calibration_.T_i_c.at(landmark.host_kf_id.cam_id);
    auto point=basalt::StereographicParam<double>::unproject(landmark.direction);point[3]=landmark.inv_dist;
    const auto world=(host*point).eval();PersistentMapCandidate candidate;
    candidate.id=id;candidate.position=gauge*(world.head<3>()/world[3]);
    candidate.finite=candidate.position.allFinite() && landmark.inv_dist>0;
    result->nonfinite+=!candidate.finite;
    const auto found=quality.points.find(id);
    if(found!=quality.points.end()) {
      candidate.eligible=found->second.eligible;candidate.observed=found->second.world.observed;
      candidate.mature=found->second.mature;candidate.maximum_sigma_m=found->second.world.maximum_sigma_m;
    }
    result->observed+=candidate.observed;
    if(candidate.eligible) {
      if(!candidate.finite || id>std::numeric_limits<int>::max() ||
          candidate.position.cwiseAbs().maxCoeff()>std::numeric_limits<float>::max())
        throw std::runtime_error("mapping_certified_point_invalid");
      result->points.push_back({float(candidate.position.x()),float(candidate.position.y()),float(candidate.position.z())});
      result->ids.push_back(int(id));
    }
    result->candidates.push_back(std::move(candidate));
  }
  result->optimize_seconds=std::chrono::duration<double>(std::chrono::steady_clock::now()-started).count();
  return result;
}
} // namespace posim_slam
