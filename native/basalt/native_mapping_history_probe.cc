// SPDX-License-Identifier: Apache-2.0
#include "native_mapping_checkpoint.hpp"
#include <Eigen/SparseQR>
#include <tbb/global_control.h>
#include <cassert>
#include <iostream>

namespace {
using namespace posim_slam;
template<class F> void rejects(F action,const std::string& reason) {
  bool rejected=false;
  try {action();} catch(const std::exception& error) {
    if(error.what()!=reason)std::cerr<<"Expected "<<reason<<", received "<<error.what()<<'\n';
    assert(error.what()==reason);rejected=true;
  }
  assert(rejected);
}
template<class A,class B> double close(const A& a,const B& b) {
  assert(a.rows()==b.rows() && a.cols()==b.cols());
  const double error=(a-b).norm()/std::max(1.,b.norm());assert(error<1e-8);return error;
}
void appendPose(basalt::NfrMapper& mapper,int64_t stamp,int64_t previous) {
  const Sophus::SE3d pose(Sophus::SO3d(),Eigen::Vector3d(.04*(stamp-1),0,0));
  mapper.getFramePoses().emplace(stamp,basalt::PoseStateWithLin<double>(stamp,pose));
  if(previous) {
    basalt::RelPoseFactor factor;factor.t_i_ns=previous;factor.t_j_ns=stamp;
    factor.T_i_j=mapper.getFramePoses().at(previous).getPose().inverse()*pose;
    factor.cov_inv=100*Sophus::Matrix6d::Identity();mapper.rel_pose_factors.push_back(factor);
  }
}
std::unique_ptr<basalt::NfrMapper> chain(int count,double reference_weight=1e4,bool star=false) {
  basalt::Calibration<double> calibration;
  auto camera=basalt::GenericCamera<double>::fromString("pinhole");
  camera.setFromInit(Eigen::Vector4d(400,400,320,240));
  calibration.intrinsics.push_back(camera);calibration.T_i_c.emplace_back();
  calibration.resolution.emplace_back(640,480);
  basalt::VioConfig config;config.mapper_use_factors=true;config.vio_init_pose_weight=reference_weight;
  auto mapper=std::make_unique<basalt::NfrMapper>(calibration,config);
  for(int64_t stamp=1;stamp<=count;++stamp)appendPose(*mapper,stamp,stamp==1?0:(star?1:stamp-1));
  basalt::RollPitchFactor gravity;gravity.t_ns=1;gravity.R_w_i_meas=Sophus::SO3d();
  gravity.cov_inv=100*Eigen::Matrix2d::Identity();mapper->roll_pitch_factors.push_back(gravity);
  for(auto [id,host]:{std::pair<int,int>{100,1},{200,count-2}}) {
    basalt::Keypoint<double> point;point.host_kf_id={host,0};point.direction.setZero();point.inv_dist=.25;
    mapper->lmdb.addLandmark(id,point);
    const auto host_pose=mapper->getFramePoses().at(host).getPose();
    Eigen::Vector4d local=basalt::StereographicParam<double>::unproject(point.direction);local[3]=point.inv_dist;
    for(int stamp=host;stamp<host+3;++stamp) {
      const basalt::TimeCamId image(stamp,0);Eigen::Vector2d pixel;
      const auto relative=mapper->getFramePoses().at(stamp).getPose().inverse()*host_pose;
      assert(calibration.intrinsics[0].project(relative*local,pixel));
      basalt::KeypointObservation<double> observation;observation.kpt_id=id;observation.pos=pixel;
      mapper->lmdb.addObservation(image,observation);mapper->feature_tracks[id].emplace(image,id);
    }
  }
  return mapper;
}
// Independent whole-system sparse QR of the original pose/landmark Jacobian.
// No conditional elimination, chart, reduced landmark or posterior from the
// implementation is used to form this covariance. The original world prior
// is included exactly once, with the same weight as the mapper configuration.
Eigen::MatrixXd fullCovariance(basalt::NfrMapper& mapper,const std::vector<int64_t>& ids) {
  const auto rows=nativeMappingRows(mapper,nullptr,kMaxNativeMappingPoseCoordinates);
  const auto offsets=nativeMappingPoseAddresses(rows.poses);
  std::map<int64_t,int> point_columns;
  int columns=6*rows.poses.size(),count=4;
  for(const auto& factor:rows.factors)count+=factor.jacobian.rows();
  for(const auto& point:rows.points){point_columns.emplace(point.id,columns);columns+=3;count+=point.pose.rows();}
  std::vector<Eigen::Triplet<double>> triplets;
  auto block=[&](int first_row,int first_col,const auto& matrix) {
    for(int col=0;col<matrix.cols();++col)for(int row=0;row<matrix.rows();++row)
      if(matrix(row,col)!=0.)triplets.emplace_back(first_row+row,first_col+col,matrix(row,col));
  };
  int first=0;
  for(const auto& factor:rows.factors) {
    for(size_t i=0;i<factor.poses.size();++i)block(first,offsets.at(factor.poses[i]),factor.jacobian.middleCols<6>(6*i));
    first+=factor.jacobian.rows();
  }
  for(const auto& point:rows.points) {
    for(size_t i=0;i<point.poses.size();++i)block(first,offsets.at(point.poses[i]),point.pose.middleCols<6>(6*i));
    block(first,point_columns.at(point.id),point.landmark);first+=point.pose.rows();
  }
  for(int axis=0;axis<3;++axis)triplets.emplace_back(first+axis,axis,std::sqrt(mapper.config.vio_init_pose_weight));
  triplets.emplace_back(first+3,5,std::sqrt(mapper.config.vio_init_pose_weight));
  Eigen::SparseMatrix<double> jacobian(count,columns);jacobian.setFromTriplets(triplets.begin(),triplets.end());
  Eigen::SparseQR<Eigen::SparseMatrix<double>,Eigen::COLAMDOrdering<int>> qr;
  qr.compute(jacobian);assert(qr.info()==Eigen::Success && qr.rank()==columns);
  Eigen::MatrixXd derivative=Eigen::MatrixXd::Zero(3*ids.size(),columns);
  for(size_t i=0;i<ids.size();++i) {
    const auto& point=mapper.lmdb.getLandmark(ids[i]);
    const auto host=mapper.getFramePoses().at(point.host_kf_id.frame_id).getPose();
    const auto camera=host*mapper.calib.T_i_c.at(point.host_kf_id.cam_id);
    Eigen::Matrix<double,4,2> direction;
    const Eigen::Vector4d ray=basalt::StereographicParam<double>::unproject(point.direction,&direction);
    const Eigen::Vector3d local=ray.head<3>()/point.inv_dist,world=camera*local;
    const int pose_column=offsets.at(point.host_kf_id.frame_id);
    derivative.block<3,3>(3*i,pose_column).setIdentity();
    derivative.block<3,3>(3*i,pose_column+3)=-Sophus::SO3d::hat(world-host.translation());
    Eigen::Matrix3d point_derivative;point_derivative.leftCols<2>()=direction.topRows<3>()/point.inv_dist;
    point_derivative.col(2)=-local/point.inv_dist;
    derivative.block<3,3>(3*i,point_columns.at(ids[i]))=camera.rotationMatrix()*point_derivative;
  }
  // A P = Q R, so D (A' A)^-1 D' = U' U with R' U = P' D'.
  Eigen::SparseMatrix<double> lower=qr.matrixR().topRows(columns).transpose();
  const Eigen::MatrixXd rhs=qr.colsPermutation().transpose()*derivative.transpose();
  const Eigen::MatrixXd solved=lower.triangularView<Eigen::Lower>().solve(rhs);
  assert(solved.allFinite());return solved.transpose()*solved;
}
void historyAndRestore(const std::filesystem::path& directory) {
  auto mapper=chain(300);std::set<int64_t> active;
  for(int64_t stamp=293;stamp<=300;++stamp)active.insert(stamp);
  rejects([&]{nativeMappingRows(*mapper);},"native_mapping_linearization_configuration");
  std::vector<int64_t> pose_order;for(const auto& [stamp,pose]:mapper->getFramePoses())pose_order.push_back(stamp);
  rejects([&]{nativeMappingPoseOffsets(pose_order);},"native_mapping_pose_order");
  SubmapInformation::Limits limits;limits.coordinates=1806;limits.active_columns=48;
  const auto identity=nativeMappingStateIdentity(*mapper);
  auto store=std::make_shared<SubmapRowStore>(directory/"history-cache",65536);
  NativeMappingWindow window(*mapper,1,active,limits,store);
  assert(window.snapshot()->information->known.size()==1800 && window.snapshot()->layout.frontier.size()==48);
  assert(window.snapshot()->information->measurements.size()==306);
  const auto context=window.worldLinearization();const auto both=window.worldInformation(context,{100,200});
  assert(both.observed);const double error=close(both.covariance,fullCovariance(*mapper,{100,200}));
  assert((both.covariance.topRightCorner<3,3>().norm()>0));
  const auto saved=NativeMappingCheckpoint::document(window);
  auto second_store=std::make_shared<SubmapRowStore>(directory/"restored-cache",65536);
  auto restored=NativeMappingCheckpoint::restore(*mapper,saved,limits,{},second_store);
  close(restored->worldInformation(restored->worldLinearization(),{100,200}).covariance,both.covariance);
  auto too_small=limits;too_small.coordinates=1794;
  rejects([&]{NativeMappingCheckpoint::restore(*mapper,saved,too_small);},"native_mapping_pose_order");
  rejects([&]{window.step([]{return true;});},"native_window_cancelled");
  assert(nativeMappingStateIdentity(*mapper)==identity);
  const auto step=restored->step();assert(step.after<=step.before);
  // Use an independent nominal source for append-only pose addresses: the
  // zero-residual step above may change roundoff-level means.
  auto growing=chain(300);
  auto grown_base=std::make_unique<NativeMappingWindow>(*growing,1,active,limits);
  const auto old=grown_base->snapshot();appendPose(*growing,301,300);
  active.erase(293);active.insert(301);
  auto grown=grown_base->prepareGrowth(active);
  assert(grown->snapshot()->information->known.size()==1806);
  assert(old->information->known.size()==1800);
  const auto grown_query=grown->worldInformation(grown->worldLinearization(),{100,200});
  close(grown_query.covariance,fullCovariance(*growing,{100,200}));
  const auto saved_grown=NativeMappingCheckpoint::document(*grown);
  auto again=NativeMappingCheckpoint::restore(*growing,saved_grown,limits);
  close(again->worldInformation(again->worldLinearization(),{100,200}).covariance,grown_query.covariance);
  const auto before=grown->snapshot();appendPose(*growing,302,301);
  rejects([&]{grown->prepareGrowth({295,296,297,298,299,300,301,302});},"native_window_coordinate_capacity");
  assert(grown->snapshot()==before);
  std::cout<<"HISTORY poses=300 coordinates=1800 active_columns=48 original_keys=306 sparse_qr_relative_error="<<error<<'\n';
}
void unchangedSafety() {
  auto mapper=chain(300,0);NativeMappingWindow window(*mapper,1,{299,300});
  const auto output=window.worldInformation(window.worldLinearization(),{100,200});
  assert(!output.observed && output.unbounded_gram.norm()>0);
  auto wide=chain(300,1e4,true);const auto before=nativeMappingStateIdentity(*wide);
  // Eliminating the hub exposes 299 neighbours: namespace space is available,
  // but the unchanged local separator bound must still reject the operation.
  rejects([&]{NativeMappingWindow invalid(*wide,1,{298,299,300});},"submap_factor_separator_capacity");
  assert(nativeMappingStateIdentity(*wide)==before);
}
} // namespace
int main(int argc,char** argv) {
  assert(argc==2);tbb::global_control threads(tbb::global_control::max_allowed_parallelism,2);
  assert(!std::filesystem::exists(argv[1]));std::filesystem::create_directories(argv[1]);
  historyAndRestore(argv[1]);unchangedSafety();
  assert(std::filesystem::is_empty(argv[1]));std::filesystem::remove(argv[1]);
  std::cout<<"Native history: sparse full-system covariance, restored sources, growth, gauge and local capacity pass.\n";
}
