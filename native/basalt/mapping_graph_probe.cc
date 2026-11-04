// SPDX-License-Identifier: Apache-2.0
#include "mapping_graph.hpp"
#include "native_mapping_linearization.hpp"
#include "native_mapping_window.hpp"
#include "native_mapping_checkpoint.hpp"
#include "mapping_output.hpp"
#include "mapping_frame_io.hpp"
#include <basalt/utils/keypoints.h>
#include <cassert>
#include <iostream>

namespace {
using Registry=posim_slam::MappingGraphRegistry;using Json=nlohmann::json;
template<class F> void rejects(F call,const std::string& reason) {
  bool rejected=false;try{call();}catch(const std::exception& e){assert(e.what()==reason);rejected=true;}assert(rejected);
}
struct Fixture {
  basalt::Calibration<double> calibration;basalt::VioConfig config;
  std::shared_ptr<posim_slam::MappingInputStore::Data> input=std::make_shared<posim_slam::MappingInputStore::Data>();
  Fixture() {
    auto camera=basalt::GenericCamera<double>::fromString("pinhole");camera.setFromInit(Eigen::Vector4d(400,400,320,240));
    calibration.intrinsics.push_back(camera);calibration.T_i_c.emplace_back();calibration.resolution.emplace_back(640,480);
    config=posim_slam::MappingInputStore::configuration(config);input->revision=1;
    for(int64_t stamp:{10,20,30}) {
      auto frame=std::make_shared<posim_slam::MappingInputStore::Frame>();frame->stamp=stamp;frame->width=640;frame->height=480;frame->digest.fill(uint8_t(stamp));
      frame->features.corners={Eigen::Vector2d(320-4*(stamp/10-1),240),Eigen::Vector2d(350-3*(stamp/10-1),260)};
      input->frames.emplace(stamp,frame);
    }
  }
  std::unique_ptr<basalt::NfrMapper> mapper(int first=42,int second=99) const {
    auto map=std::make_unique<basalt::NfrMapper>(calibration,config);
    for(const auto& [stamp,frame]:input->frames) {
      map->getFramePoses().emplace(stamp,basalt::PoseStateWithLin<double>(stamp,Sophus::SE3d(Sophus::SO3d(),Eigen::Vector3d(.01*stamp,0,0))));
      map->feature_corners.emplace(basalt::TimeCamId(stamp,0),frame.read()->features);
      map->feature_tracks[first].emplace(basalt::TimeCamId(stamp,0),0);map->feature_tracks[second].emplace(basalt::TimeCamId(stamp,0),1);
    }
    for(auto [id,index]:{std::pair<int,int>{first,0},{second,1}}) {
      basalt::Keypoint<double> point;point.host_kf_id=basalt::TimeCamId(10,0);point.direction=Eigen::Vector2d(.1*index,-.1*index);point.inv_dist=index?0:.25;
      map->lmdb.addLandmark(id,point);
      for(int64_t stamp:{10,20,30}) {
        basalt::KeypointObservation<double> obs;obs.kpt_id=id;obs.pos=input->frames.at(stamp).read()->features.corners[index];map->lmdb.addObservation(basalt::TimeCamId(stamp,0),obs);
      }
    }
    basalt::RelPoseFactor relative;relative.t_i_ns=10;relative.t_j_ns=20;relative.T_i_j=map->getFramePoses().at(10).getPose().inverse()*map->getFramePoses().at(20).getPose();relative.cov_inv=Eigen::Matrix<double,6,6>::Identity();map->rel_pose_factors.push_back(relative);
    basalt::RollPitchFactor roll;roll.t_ns=10;roll.R_w_i_meas=Sophus::SO3d();roll.cov_inv=Eigen::Matrix2d::Identity();map->roll_pitch_factors.push_back(roll);
    return map;
  }
};
void compare(basalt::NfrMapper& first,basalt::NfrMapper& second) {
  assert(first.feature_tracks==second.feature_tracks && first.lmdb.getLandmarks().size()==second.lmdb.getLandmarks().size());
  for(const auto& [stamp,value]:first.getFramePoses()) {
    const auto a=value.getPose(),b=second.getFramePoses().at(stamp).getPose();assert(std::memcmp(a.data(),b.data(),7*sizeof(double))==0);
  }
  for(const auto& [id,value]:first.lmdb.getLandmarks()) {
    const auto& other=second.lmdb.getLandmarks().at(id);assert(value.host_kf_id==other.host_kf_id && value.obs==other.obs);
    assert(std::memcmp(value.direction.data(),other.direction.data(),2*sizeof(double))==0 && std::memcmp(&value.inv_dist,&other.inv_dist,sizeof(double))==0);
  }
  for(size_t i=0;i<first.rel_pose_factors.size();++i) {
    const auto& a=first.rel_pose_factors[i];const auto& b=second.rel_pose_factors[i];assert(a.t_i_ns==b.t_i_ns && a.t_j_ns==b.t_j_ns);
    assert(std::memcmp(a.T_i_j.data(),b.T_i_j.data(),7*sizeof(double))==0 && std::memcmp(a.cov_inv.data(),b.cov_inv.data(),36*sizeof(double))==0);
  }
}
void nativeRowsControls(const Fixture& fixture) {
  auto mapper=fixture.mapper();auto untouched=fixture.mapper();
  const auto rows=posim_slam::nativeMappingRows(*mapper);
  assert(rows.points.size()==2 && rows.originals==6 && rows.valid_projections==6 && rows.factors.size()==2);
  const auto offsets=posim_slam::nativeMappingPoseOffsets(rows.poses);
  double maximum=0;const double epsilon=1e-6;
  auto project=[&](const basalt::Keypoint<double>& point,const basalt::TimeCamId& image,
                   const Eigen::aligned_map<int64_t,basalt::PoseStateWithLin<double>>& poses) {
    const auto host=poses.at(point.host_kf_id.frame_id).getPose()*mapper->calib.T_i_c.at(point.host_kf_id.cam_id);
    const auto target=poses.at(image.frame_id).getPose()*mapper->calib.T_i_c.at(image.cam_id);
    Eigen::Vector4d local=basalt::StereographicParam<double>::unproject(point.direction);local[3]=point.inv_dist;
    Eigen::Vector2d pixel;const bool valid=mapper->calib.intrinsics.at(image.cam_id).project((target.inverse()*host)*local,pixel);
    assert(valid);return Eigen::Vector2d(pixel-point.obs.at(image));
  };
  for(const auto& group:rows.points) {
    const auto& point=mapper->lmdb.getLandmark(group.id);
    assert(point.inv_dist>0 || (group.landmark.col(2).norm()>0 && group.pose.norm()>0));
    const auto reduced=posim_slam::reduceNativeMappingPoint(group,offsets);
    assert(reduced.likelihood.originals.size()==3 && reduced.backsub.rows()==3);
    for(size_t i=0;i<group.originals.size();++i) {
      const auto [stamp,camera,feature]=group.originals[i];const basalt::TimeCamId image(stamp,camera);
      const auto base=project(point,image,mapper->getFramePoses());
      const double weight=std::sqrt(base.norm()<mapper->huber_thresh?1:mapper->huber_thresh/base.norm())/mapper->obs_std_dev;
      assert((weight*base-group.residual.segment<2>(2*i)).norm()<1e-10);
      for(size_t p=0;p<group.poses.size();++p)for(int axis=0;axis<6;++axis) {
        auto plus=mapper->getFramePoses(),minus=plus;Sophus::Vector6d increment=Sophus::Vector6d::Zero();increment[axis]=epsilon;
        plus.at(group.poses[p]).applyInc(increment);minus.at(group.poses[p]).applyInc(-increment);
        const Eigen::Vector2d numeric=weight*(project(point,image,plus)-project(point,image,minus))/(2*epsilon);
        maximum=std::max(maximum,(numeric-group.pose.block<2,1>(2*i,6*p+axis)).norm());
      }
      for(int axis=0;axis<3;++axis) {
        auto plus=point,minus=point;
        if(axis<2){plus.direction[axis]+=epsilon;minus.direction[axis]-=epsilon;}
        else{plus.inv_dist+=epsilon;minus.inv_dist-=epsilon;}
        const Eigen::Vector2d numeric=weight*(project(plus,image,mapper->getFramePoses())-project(minus,image,mapper->getFramePoses()))/(2*epsilon);
        maximum=std::max(maximum,(numeric-group.landmark.block<2,1>(2*i,axis)).norm());
      }
    }
  }
  assert(maximum<2e-7);compare(*mapper,*untouched);
  Eigen::VectorXd trial(6*rows.poses.size());
  for(Eigen::Index i=0;i<trial.size();++i)trial[i]=.03*std::sin(double(i+1));
  const auto normalized=posim_slam::nativeMappingReferenceIncrement(*mapper,rows.poses,trial,10);
  assert(normalized.head<3>().norm()==0 && std::abs(normalized[5])<1e-15);
  assert((posim_slam::nativeMappingReferenceIncrement(*mapper,rows.poses,normalized,10)-normalized).norm()<1e-15);
  const Eigen::VectorXd removed=trial-normalized;
  for(const auto& group:rows.points) {
    Eigen::VectorXd local(group.pose.cols());
    for(size_t i=0;i<group.poses.size();++i)local.segment<6>(6*i)=removed.segment<6>(offsets.at(group.poses[i]));
    assert((group.pose*local).norm()<1e-10);
  }
  for(const auto& group:rows.factors) {
    Eigen::VectorXd local(group.jacobian.cols());
    for(size_t i=0;i<group.poses.size();++i)local.segment<6>(6*i)=removed.segment<6>(offsets.at(group.poses[i]));
    assert((group.jacobian*local).norm()<1e-10);
  }
  auto tilted=fixture.mapper();auto& reference=tilted->getFramePoses().at(10);
  Sophus::Vector6d tilt=Sophus::Vector6d::Zero();tilt.tail<3>()=Eigen::Vector3d(.2,-.1,.3);reference.applyInc(tilt);
  const auto aligned=posim_slam::nativeMappingReferenceIncrement(*tilted,rows.poses,trial,10);
  auto plus=reference,minus=reference;plus.applyInc(epsilon*aligned.head<6>());minus.applyInc(-epsilon*aligned.head<6>());
  auto yaw=[](const auto& state){const auto rotation=state.getPose().rotationMatrix();return std::atan2(rotation(1,0),rotation(0,0));};
  assert(std::abs(yaw(plus)-yaw(minus))/(2*epsilon)<1e-8);
  mapper->config.mapper_use_factors=false;assert(posim_slam::nativeMappingRows(*mapper).factors.empty());
  auto invalid=fixture.mapper();invalid->lmdb.getLandmark(42).direction=Eigen::Vector2d(2,0);
  const auto invalid_rows=posim_slam::nativeMappingRows(*invalid);
  bool retained=false;
  for(const auto& group:invalid_rows.points)if(group.id==42) {
    retained=true;assert(group.originals.size()==3 && !group.pose.norm() && !group.landmark.norm() && !group.residual.norm());
    for(bool valid:group.valid)assert(!valid);
    const auto reduced=posim_slam::reduceNativeMappingPoint(group,offsets);assert(reduced.unbounded.cols()==3);
  }
  assert(retained && invalid_rows.originals==6 && invalid_rows.valid_projections==3);
  auto frozen=fixture.mapper();frozen->getFramePoses().at(10).setLinTrue();
  rejects([&]{posim_slam::nativeMappingRows(*frozen);},"native_mapping_current_tangent_required");
  auto duplicate=fixture.mapper();duplicate->feature_tracks[99].at(basalt::TimeCamId(20,0))=0;
  rejects([&]{posim_slam::nativeMappingRows(*duplicate);},"native_mapping_original_reused");
  auto invalid_factor=fixture.mapper();invalid_factor->rel_pose_factors[0].cov_inv(0,0)=-1;
  rejects([&]{posim_slam::nativeMappingRows(*invalid_factor);},"native_mapping_factor_information");
  std::cout<<"Native mapping rows: finite/infinite original observations, analytic pose/landmark derivatives, invalid-row ownership and current-tangent checks pass; derivative error "<<maximum<<".\n";
}
void nativeRowStreamControls(const Fixture& fixture) {
  auto mapper=fixture.mapper();mapper->lmdb.getLandmark(42).direction=Eigen::Vector2d(2,0);
  size_t factors=0,points=0,originals=0,invalid_count=0;
  const auto summary=posim_slam::streamNativeMappingRows(*mapper,
    [&](posim_slam::NativeMappingPoseRows row) {++factors;assert(!row.original.empty());},
    [&](posim_slam::NativeMappingPointRows row) {
      ++points;originals+=row.originals.size();
      for(size_t i=0;i<row.valid.size();++i)if(!row.valid[i]) {
        ++invalid_count;assert(row.pose.middleRows<2>(2*i).isZero() && row.landmark.middleRows<2>(2*i).isZero());
        assert(row.residual.segment<2>(2*i).isZero());
      }
    });
  assert(summary.poses.size()==3 && factors==2 && points==2 && originals==6 && invalid_count==3);
  assert(summary.pose_factors==factors && summary.points==points && summary.originals==originals);
  assert(summary.valid_projections==3);
  auto unchanged=fixture.mapper();mapper=fixture.mapper();
  rejects([&]{posim_slam::streamNativeMappingRows(*mapper,
    [](posim_slam::NativeMappingPoseRows){},
    [](posim_slam::NativeMappingPointRows){throw std::runtime_error("sink_cancelled");});},"sink_cancelled");
  compare(*mapper,*unchanged);
  mapper->feature_tracks[99].at(basalt::TimeCamId(20,0))=0;
  rejects([&]{posim_slam::streamNativeMappingRows(*mapper,
    [](posim_slam::NativeMappingPoseRows){},[](posim_slam::NativeMappingPointRows){});},"native_mapping_original_reused");
  auto invalid=fixture.mapper();invalid->lmdb.getLandmark(42).inv_dist=-1;
  auto original=fixture.mapper();original->lmdb.getLandmark(42).inv_dist=-1;
  rejects([&]{posim_slam::NativeMappingWindow window(*invalid,10,{10,20});},"native_mapping_landmark");
  compare(*invalid,*original);
  std::cout<<"Original row stream: invalid-slot ownership, summary counts, sink exception, duplicate rejection and failed-construction source preservation pass.\n";
}
void nativeScheduleControls(const Fixture& fixture) {
  auto mapper=fixture.mapper();
  for(int64_t stamp:{30,40,50,60}) {
    mapper->getFramePoses().emplace(stamp,basalt::PoseStateWithLin<double>(stamp,
      Sophus::SE3d(Sophus::SO3d(),Eigen::Vector3d(.01*stamp,0,0))));
    auto relative=mapper->rel_pose_factors.front();relative.t_i_ns=stamp-10;relative.t_j_ns=stamp;
    relative.T_i_j=mapper->getFramePoses().at(stamp-10).getPose().inverse()*mapper->getFramePoses().at(stamp).getPose();
    mapper->rel_pose_factors.push_back(relative);
  }
  auto roll=mapper->roll_pitch_factors.front();roll.t_ns=60;mapper->roll_pitch_factors.push_back(roll);
  basalt::Keypoint<double> point;point.host_kf_id={40,0};point.direction.setZero();point.inv_dist=.25;
  mapper->lmdb.addLandmark(777,point);
  for(int64_t stamp:{40,50,60}) {
    basalt::KeypointObservation<double> observation;observation.kpt_id=777;observation.pos={320.-4*(stamp/10-4),240.};
    mapper->lmdb.addObservation({stamp,0},observation);mapper->feature_tracks[777][{stamp,0}]=0;
  }
  posim_slam::NativeMappingSchedule schedule(*mapper,{50,60});
  assert(schedule.frontier==std::set<int64_t>({50,60}) && schedule.unobserved.empty());
  assert(schedule.last_use.at(10)==30 && schedule.last_use.at(20)==30 && schedule.last_use.at(30)==40);
  const auto collected=posim_slam::nativeMappingRows(*mapper);
  const auto summary=posim_slam::streamNativeMappingRows(*mapper,
    [](posim_slam::NativeMappingPoseRows){},[](posim_slam::NativeMappingPointRows){},nullptr,&schedule.order);
  assert(summary.originals==collected.originals && summary.pose_factors==collected.factors.size());
  auto incomplete=schedule.order;incomplete.pop_back();
  rejects([&]{posim_slam::streamNativeMappingRows(*mapper,
    [](posim_slam::NativeMappingPoseRows){},[](posim_slam::NativeMappingPointRows){},nullptr,&incomplete);},"native_mapping_row_order_incomplete");
  posim_slam::NativeMappingWindow full(*mapper,10,{10,20,30,40,50,60});
  posim_slam::NativeMappingWindow window(*mapper,10,{50,60});
  assert(window.snapshot()->base->nodes.size()==3 && window.snapshot()->base->order.size()==12);
  assert(window.snapshot()->information->measurements==full.snapshot()->information->measurements);
  assert((window.proposal()-full.proposal()).norm()<1e-10);
  const auto expected=full.worldInformation({42,777}),actual=window.worldInformation({42,777});
  assert(actual.observed==expected.observed && (actual.positions-expected.positions).norm()==0);
  const double covariance_error=(actual.covariance-expected.covariance).norm()/expected.covariance.norm();
  assert(covariance_error<1e-10);
  const double reference_weight=mapper->config.vio_init_pose_weight;mapper->config.vio_init_pose_weight=0;
  posim_slam::NativeMappingWindow free_full(*mapper,10,{10,20,30,40,50,60});
  posim_slam::NativeMappingWindow free_window(*mapper,10,{50,60});
  const auto free_a=free_full.worldInformation({42,777}),free_b=free_window.worldInformation({42,777});
  assert(!free_a.observed && !free_b.observed);
  assert((free_a.unbounded_gram-free_b.unbounded_gram).norm()<1e-10);
  mapper->config.vio_init_pose_weight=reference_weight;
  // An unused original pose retains its free coordinates, without a new key.
  mapper->getFramePoses().emplace(70,basalt::PoseStateWithLin<double>(70,Sophus::SE3d()));
  posim_slam::NativeMappingWindow unused(*mapper,10,{50,60});
  assert(unused.snapshot()->information->known.size()==42);
  assert(unused.snapshot()->information->measurements==window.snapshot()->information->measurements);
  assert(unused.proposal().tail<6>().norm()==0);
  bool free_pose=false;
  for(const auto& node:unused.snapshot()->base->nodes)if(node->removed==std::vector<int64_t>({36,37,38,39,40,41})) {
    free_pose=true;assert(node->noise.cols()==0 && node->unbounded.cols()==6);
    assert((node->unbounded*node->unbounded.transpose()).isApprox(Eigen::Matrix<double,6,6>::Identity(),1e-12));
  }
  assert(free_pose);
  auto oversized=fixture.mapper();
  for(int64_t i=4;i<=posim_slam::kMaxJointColumns/6+1;++i)
    oversized->getFramePoses().emplace(10*i,basalt::PoseStateWithLin<double>(10*i,Sophus::SE3d()));
  rejects([&]{posim_slam::NativeMappingSchedule rejected(*oversized,{10,20,30});},"native_mapping_linearization_configuration");
  rejects([&]{posim_slam::nativeMappingRows(*oversized);},"native_mapping_linearization_configuration");
  std::cout<<"Original lifetime schedule: multiple conditionals, shared-world cross-point covariance, complete ownership and unobserved pose retention pass; covariance error "<<covariance_error<<".\n";
}
void nativeWindowControls(const Fixture& fixture) {
  auto mapper=fixture.mapper();
  for(auto& [stamp,state]:mapper->getFramePoses()) {
    Sophus::Vector6d tilt=Sophus::Vector6d::Zero();tilt.tail<3>()=Eigen::Vector3d(.2,-.1,.3);state.applyInc(tilt);
  }
  posim_slam::NativeMappingChart chart(*mapper,10);
  Eigen::VectorXd value=Eigen::VectorXd::Zero(18);
  for(int i=0;i<18;++i)value[i]=.1*std::sin(double(i));value.head<3>().setZero();value[5]=0;
  chart.apply(*mapper,value);assert((chart.coordinates(*mapper)-value).norm()<1e-14);
  const auto current=mapper->getFramePoses();const double epsilon=1e-6;double maximum=0;
  for(const auto& [stamp,index]:chart.offsets())for(int axis=0;axis<6;++axis) {
    auto plus=value,minus=value;plus[index+axis]+=epsilon;minus[index+axis]-=epsilon;
    chart.apply(*mapper,plus);const auto p=mapper->getFramePoses().at(stamp).getPose();
    chart.apply(*mapper,minus);const auto m=mapper->getFramePoses().at(stamp).getPose();
    Sophus::Vector6d numeric;numeric.head<3>()=(p.translation()-m.translation())/(2*epsilon);
    numeric.tail<3>()=(p.so3()*m.so3().inverse()).log()/(2*epsilon);
    maximum=std::max(maximum,(numeric-chart.derivative(stamp,value).col(axis)).norm());
  }
  assert(maximum<1e-8);mapper->getFramePoses()=current;
  auto outside=value;outside[15]=posim_slam::kNativeMappingPi;
  rejects([&]{chart.apply(*mapper,outside);},"native_window_rotation_chart_boundary");
  for(const auto& [stamp,state]:current)assert(std::memcmp(state.getPose().data(),mapper->getFramePoses().at(stamp).getPose().data(),7*sizeof(double))==0);
  mapper->lmdb.getLandmark(42).direction+=Eigen::Vector2d(.002,-.001);
  const auto source=posim_slam::nativeMappingRows(*mapper),anchored=chart.rows(*mapper,source);
  for(size_t i=0;i<source.points.size();++i) {
    const auto& group=anchored.points[i];Eigen::VectorXd local(6*group.poses.size());
    for(size_t p=0;p<group.poses.size();++p)local.segment<6>(6*p)=value.segment<6>(chart.offsets().at(group.poses[p]));
    const auto& point=mapper->lmdb.getLandmark(group.id);
    const Eigen::Vector3d delta=Eigen::Vector3d(point.direction.x(),point.direction.y(),point.inv_dist)-chart.point(group.id);
    assert((group.residual+group.pose*local+group.landmark*delta-source.points[i].residual).norm()<1e-10);
  }
  posim_slam::NativeMappingSelection selected;selected.points.insert(42);
  const auto subset=posim_slam::nativeMappingRows(*mapper,&selected);
  assert(subset.points.size()==1 && subset.originals==3 && subset.factors.empty());
  selected.points.insert(999);rejects([&]{posim_slam::nativeMappingRows(*mapper,&selected);},"native_mapping_selection_missing");
  auto first=fixture.mapper(),untouched=fixture.mapper();
  posim_slam::SubmapInformation::Limits limits;limits.active_columns=12;
  rejects([&]{posim_slam::NativeMappingWindow rejected(*first,10,{10,20,30},limits);},"native_window_separator_capacity");
  compare(*first,*untouched);
  limits.active_columns=18;limits.numeric_bytes=1;
  rejects([&]{posim_slam::NativeMappingWindow rejected(*first,10,{10,20,30},limits);},"native_window_numeric_capacity");
  compare(*first,*untouched);
  posim_slam::NativeMappingWindow window(*first,10,{10,20,30});
  assert(window.proposal().allFinite());const auto before=window.snapshot();
  int calls=0;rejects([&]{window.step([&]{return ++calls==3;});},"native_window_cancelled");
  assert(window.snapshot()==before);compare(*first,*untouched);
  const auto step=window.step();assert(step.accepted && step.after<step.before);
  assert(window.snapshot()->information->measurements==before->information->measurements);
  assert(window.snapshot()->information->measurements.size()==8);
  assert(window.numericBytes()>window.snapshot()->numericBytes() && window.numericBytes()<96*1024*1024);
  assert((first->getFramePoses().at(10).getPose().translation()-untouched->getFramePoses().at(10).getPose().translation()).norm()==0);
  std::cout<<"Native active window: fixed-chart derivatives, gauge convention, selected originals, capacity, nonlinear update and cancellation rollback pass; derivative error "<<maximum<<".\n";
}
void nativeWorldControls(const Fixture& fixture) {
  auto mapper=fixture.mapper();mapper->config.vio_init_pose_weight=2.;
  mapper->calib.T_i_c[0]=Sophus::SE3d(Sophus::SO3d::exp(Eigen::Vector3d(.03,-.06,.02)),Eigen::Vector3d(.12,-.04,.07));
  mapper->lmdb.getLandmark(99).inv_dist=.2;
  for(auto& [stamp,state]:mapper->getFramePoses()) {
    Sophus::Vector6d tilt=Sophus::Vector6d::Zero();tilt.tail<3>()=Eigen::Vector3d(.25,-.3,.4);state.applyInc(tilt);
  }
  mapper->rel_pose_factors.clear();
  for(const auto pair:{std::pair<int64_t,int64_t>{10,20},{20,30}}) {
    basalt::RelPoseFactor factor;factor.t_i_ns=pair.first;factor.t_j_ns=pair.second;
    factor.T_i_j=mapper->getFramePoses().at(pair.first).getPose().inverse()*mapper->getFramePoses().at(pair.second).getPose();
    factor.cov_inv=10*Sophus::Matrix6d::Identity();mapper->rel_pose_factors.push_back(factor);
  }
  mapper->roll_pitch_factors[0].R_w_i_meas=mapper->getFramePoses().at(10).getPose().so3();
  auto world=[&](int64_t id) {
    const auto& p=mapper->lmdb.getLandmark(id);
    const auto host=mapper->getFramePoses().at(p.host_kf_id.frame_id).getPose()*mapper->calib.T_i_c.at(p.host_kf_id.cam_id);
    const Eigen::Vector4d ray=basalt::StereographicParam<double>::unproject(p.direction);
    return Eigen::Vector3d(host*(ray.head<3>()/p.inv_dist));
  };
  for(int64_t id:{42,99}) {
    const auto point=world(id);auto& p=mapper->lmdb.getLandmark(id);
    for(auto& [image,pixel]:p.obs) {
      const auto camera=mapper->getFramePoses().at(image.frame_id).getPose()*mapper->calib.T_i_c.at(image.cam_id);
      Eigen::Vector4d local;local.head<3>()=camera.inverse()*point;local[3]=1;
      assert(mapper->calib.intrinsics.at(image.cam_id).project(local,pixel));
    }
  }
  auto dense=[&] {
    auto rows=posim_slam::nativeMappingRows(*mapper);int count=4;
    std::sort(rows.points.begin(),rows.points.end(),[](const auto& a,const auto& b){return a.id<b.id;});
    for(const auto& group:rows.factors)count+=group.jacobian.rows();
    for(const auto& group:rows.points)count+=group.pose.rows();
    Eigen::MatrixXd jacobian=Eigen::MatrixXd::Zero(count,24);int offset=0;
    const auto poses=posim_slam::nativeMappingPoseOffsets(rows.poses);
    for(const auto& group:rows.factors) {
      for(size_t i=0;i<group.poses.size();++i)jacobian.block(offset,poses.at(group.poses[i]),group.jacobian.rows(),6)=group.jacobian.middleCols<6>(6*i);
      offset+=group.jacobian.rows();
    }
    for(size_t p=0;p<rows.points.size();++p) {
      const auto& group=rows.points[p];
      for(size_t i=0;i<group.poses.size();++i)jacobian.block(offset,poses.at(group.poses[i]),group.pose.rows(),6)=group.pose.middleCols<6>(6*i);
      jacobian.block(offset,18+3*p,group.pose.rows(),3)=group.landmark;offset+=group.pose.rows();
    }
    jacobian.block<3,3>(offset,0)=std::sqrt(2.)*Eigen::Matrix3d::Identity();jacobian(offset+3,5)=std::sqrt(2.);
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(jacobian,Eigen::ComputeThinV);assert(svd.singularValues().minCoeff()>1e-5);
    const Eigen::MatrixXd root=svd.matrixV()*svd.singularValues().cwiseInverse().asDiagonal();
    // Numerical world derivatives independently exercise host, calibration and
    // inverse-distance transport, without using the window's analytic chain.
    Eigen::MatrixXd output=Eigen::MatrixXd::Zero(6,24);const double epsilon=1e-6;
    for(size_t p=0;p<rows.points.size();++p) {
      const auto id=rows.points[p].id;
      for(const auto& [stamp,index]:poses)for(int axis=0;axis<6;++axis) {
        const auto state=mapper->getFramePoses().at(stamp);Sophus::Vector6d d=Sophus::Vector6d::Zero();d[axis]=epsilon;
        mapper->getFramePoses().at(stamp).applyInc(d);const auto plus=world(id);
        mapper->getFramePoses().at(stamp)=state;mapper->getFramePoses().at(stamp).applyInc(-d);const auto minus=world(id);
        mapper->getFramePoses().at(stamp)=state;output.block<3,1>(3*p,index+axis)=(plus-minus)/(2*epsilon);
      }
      auto& point=mapper->lmdb.getLandmark(id);const auto direction=point.direction;const double inv=point.inv_dist;
      for(int axis=0;axis<3;++axis) {
        if(axis<2)point.direction[axis]+=epsilon;else point.inv_dist+=epsilon;const auto plus=world(id);
        point.direction=direction;point.inv_dist=inv;
        if(axis<2)point.direction[axis]-=epsilon;else point.inv_dist-=epsilon;const auto minus=world(id);
        point.direction=direction;point.inv_dist=inv;output.block<3,1>(3*p,18+3*p+axis)=(plus-minus)/(2*epsilon);
      }
    }
    const Eigen::MatrixXd finite=output*root;return Eigen::MatrixXd(finite*finite.transpose());
  };
  posim_slam::NativeMappingWindow window(*mapper,10,{10,20,30});const auto before=window.snapshot();
  const auto query=window.worldInformation({42,99});const auto expected=dense();
  assert(query.observed && query.unbounded_gram.norm()<1e-16);
  assert((query.covariance-expected).norm()/expected.norm()<1e-8);
  assert((query.positions.head<3>()-world(42)).norm()==0 && window.snapshot()==before);
  const auto single=window.worldInformation({42});assert((single.covariance-query.covariance.topLeftCorner<3,3>()).norm()<1e-10);
  rejects([&]{window.worldInformation({42,42});},"native_window_world_points");
  rejects([&]{window.worldInformation({123});},"native_window_unknown_point");
  auto& infinite=mapper->lmdb.getLandmark(99);const double inv=infinite.inv_dist;infinite.inv_dist=0;
  rejects([&]{window.worldInformation({99});},"native_window_nonfinite_world_point");infinite.inv_dist=inv;
  // Pure translation reference error is shared. Subtracting two world points
  // cancels it, while independent per-point treatment leaves an extra I.
  Eigen::Matrix<double,3,6> relative;relative<<Eigen::Matrix3d::Identity(),-Eigen::Matrix3d::Identity();
  const double relative_error=(relative*(query.covariance-expected)*relative.transpose()).norm()/
    std::max(1.,(relative*expected*relative.transpose()).norm());
  assert(relative_error<1e-8);
  const Eigen::Matrix3d wrong=query.covariance.topLeftCorner<3,3>()+query.covariance.bottomRightCorner<3,3>();
  assert((wrong-relative*query.covariance*relative.transpose()).norm()>1.);
  // A changed active tangent must still match the original physical reference.
  Sophus::Vector6d delta=Sophus::Vector6d::Zero();delta[0]=.004;delta[4]=.002;
  mapper->getFramePoses().at(30).applyInc(delta);
  posim_slam::NativeMappingWindow changing(*mapper,10,{10,20,30});
  const auto stale=changing.worldLinearization();
  rejects([&]{window.worldInformation(stale,{42});},"native_window_stale_world_linearization");
  const auto step=changing.step();assert(step.accepted);
  rejects([&]{changing.worldInformation(stale,{42});},"native_window_stale_world_linearization");
  const auto changed=changing.worldInformation({42,99});const auto changed_expected=dense();
  assert(changed.observed && (changed.covariance-changed_expected).norm()/changed_expected.norm()<1e-8);
  mapper->config.vio_init_pose_weight=0;
  posim_slam::NativeMappingWindow unreferenced(*mapper,10,{10,20,30});const auto unbounded=unreferenced.worldInformation({42,99});
  assert(!unbounded.observed && unbounded.unbounded_gram.norm()>1.);
  // Existing owners retain the original weight instead of rereading settings.
  assert((changing.worldInformation({42,99}).covariance-changed.covariance).norm()==0);
  std::cout<<"Native world information: tilted reference, camera extrinsics, shared point covariance, changed chart, missing reference and invalid candidates pass; world covariance relative error "
    <<(query.covariance-expected).norm()/expected.norm()<<", relative-point covariance error "<<relative_error<<".\n";
}
} // namespace

void nativeCheckpointControls(const Fixture& fixture) {
  using Checkpoint=posim_slam::NativeMappingCheckpoint;
  auto original=fixture.mapper(),restored=fixture.mapper();
  posim_slam::NativeMappingWindow window(*original,10,{10,20,30});assert(window.step().accepted);
  const auto document=Checkpoint::document(window);
  const auto saved=Json::from_cbor(Json::to_cbor(document));assert(saved==document);
  restored->getFramePoses()=original->getFramePoses();
  for(const auto& [id,p]:original->lmdb.getLandmarks()) {
    auto& q=restored->lmdb.getLandmark(id);q.direction=p.direction;q.inv_dist=p.inv_dist;
  }
  auto reopened=Checkpoint::restore(*restored,saved);compare(*original,*restored);
  assert((window.proposal()-reopened->proposal()).norm()<1e-8);
  assert(window.snapshot()->information->measurements==reopened->snapshot()->information->measurements);
  const auto a=window.worldInformation({42}),b=reopened->worldInformation({42});
  assert((a.positions-b.positions).norm()==0 && a.observed==b.observed);
  assert((a.covariance-b.covariance).norm()<1e-8*std::max(1e-12,a.covariance.norm()));
  auto corrupted=saved;corrupted["model"]["reference"]=20;
  rejects([&]{Checkpoint::restore(*restored,corrupted);},"native_window_checkpoint_digest");compare(*original,*restored);
  auto& pixel=restored->lmdb.getLandmark(42).obs.at(basalt::TimeCamId(20,0));const auto old_pixel=pixel;pixel.x()+=.25;
  rejects([&]{Checkpoint::restore(*restored,saved);},"native_window_checkpoint_source");assert(pixel.x()==old_pixel.x()+.25);pixel=old_pixel;
  const double old_noise=restored->obs_std_dev;restored->obs_std_dev*=2;
  rejects([&]{Checkpoint::restore(*restored,saved);},"native_window_checkpoint_source");restored->obs_std_dev=old_noise;
  const auto camera=restored->calib.T_i_c[0];restored->calib.T_i_c[0].translation().x()+=.01;
  rejects([&]{Checkpoint::restore(*restored,saved);},"native_window_checkpoint_source");restored->calib.T_i_c[0]=camera;
  restored->rel_pose_factors[0].cov_inv(0,0)*=2;
  rejects([&]{Checkpoint::restore(*restored,saved);},"native_window_checkpoint_source");restored->rel_pose_factors[0].cov_inv(0,0)/=2;
  const auto host=restored->lmdb.getLandmark(42).host_kf_id;restored->lmdb.getLandmark(42).host_kf_id=basalt::TimeCamId(20,0);
  rejects([&]{Checkpoint::restore(*restored,saved);},"native_window_checkpoint_source");restored->lmdb.getLandmark(42).host_kf_id=host;
  Sophus::Vector6d delta=Sophus::Vector6d::Zero();delta[0]=.1;restored->getFramePoses().at(20).applyInc(delta);
  const auto changed=posim_slam::nativeMappingStateIdentity(*restored);
  rejects([&]{Checkpoint::restore(*restored,saved);},"native_window_checkpoint_state");
  assert(posim_slam::nativeMappingStateIdentity(*restored)==changed);restored->getFramePoses()=original->getFramePoses();
  // Even a self-consistent checksum cannot turn invalid anchor values into a
  // valid model. Decode failure after earlier pose writes must roll back too.
  auto invalid=saved;const double bad[]={0,0,-1};
  invalid["model"]["points"][0][3]=posim_slam::mapping_graph_detail::doubles(bad,3);
  const auto encoded=Json::to_cbor(invalid["model"]);posim_slam::InputDigest digest;digest.update(encoded.data(),encoded.size());
  const auto hash=digest.finish();invalid["sha256"]=Json::binary(std::vector<uint8_t>(hash.begin(),hash.end()));
  rejects([&]{Checkpoint::restore(*restored,invalid);},"native_window_checkpoint_point");compare(*original,*restored);
  posim_slam::SubmapInformation::Limits small;small.numeric_bytes=1;
  rejects([&]{Checkpoint::restore(*restored,saved,small);},"native_window_numeric_capacity");compare(*original,*restored);
  int checks=0;rejects([&]{Checkpoint::restore(*restored,saved,{},[&]{return ++checks==3;});},"native_window_cancelled");compare(*original,*restored);
  original->huber_thresh*=2;
  rejects([&]{Checkpoint::document(window);},"native_window_checkpoint_source_changed");original->huber_thresh/=2;
  const auto first=window.step(),second=reopened->step();assert(first.accepted==second.accepted);
  assert(std::abs(first.after-second.after)<1e-8);
  for(const auto& [stamp,state]:original->getFramePoses())
    assert((state.getPose().matrix()-restored->getFramePoses().at(stamp).getPose().matrix()).norm()<1e-8);
  std::cout<<"Native window checkpoint: CBOR anchors, shared uncertainty, original ownership, continued optimization, source/state binding, corruption, capacity and cancellation rollback pass.\n";
}

void nativeArchiveControls(const Fixture& fixture,const std::filesystem::path& directory) {
  using Checkpoint=posim_slam::NativeMappingCheckpoint;
  auto mapper=fixture.mapper(),other=fixture.mapper();
  posim_slam::NativeMappingWindow window(*mapper,10,{10,20,30});assert(window.step().accepted);
  Registry registry(fixture.calibration,fixture.config,"window-fixture");
  rejects([&]{registry.capture(fixture.input,*other,10,Sophus::SE3d(),Sophus::SE3d(),&window);},"native_window_checkpoint_owner");
  rejects([&]{registry.capture(fixture.input,*mapper,20,Sophus::SE3d(),Sophus::SE3d(),&window);},"native_window_checkpoint_owner");
  const auto noise=mapper->obs_std_dev;mapper->obs_std_dev*=2;
  rejects([&]{registry.capture(fixture.input,*mapper,10,Sophus::SE3d(),Sophus::SE3d(),&window);},"mapping_graph_source");mapper->obs_std_dev=noise;
  const auto graph=registry.capture(fixture.input,*mapper,10,Sophus::SE3d(),Sophus::SE3d(),&window);
  assert(graph->hasNativeWindow() && Registry::document(graph).at("graph_revision")==1);
  const auto checkpoint=Checkpoint::document(window);const auto quality=window.mapInformation(.1);
  const auto output=posim_slam::mappingOutput(*mapper,Sophus::SE3d(),quality,graph);
  const auto path=directory/"window.graph.pmap";
  assert(window.step().accepted); // The export must retain the earlier state.
  Registry::write(path,graph,321,30);
  const auto loaded=Registry::read(path,"window-fixture");assert(loaded.graph->hasNativeWindow());
  auto restored=Registry::restore(Registry::document(loaded.graph));
  auto reopened=Checkpoint::restore(*restored,loaded.graph);
  assert(Checkpoint::document(*reopened)==checkpoint);
  const auto replay=posim_slam::mappingOutput(*restored,Sophus::SE3d(),reopened->mapInformation(.1),loaded.graph);
  assert(output.points==replay.points && output.ids==replay.ids && output.nonfinite==replay.nonfinite);
  const auto bytes=[&]{std::ifstream in(path,std::ios::binary);return std::vector<uint8_t>((std::istreambuf_iterator<char>(in)),{});}();
  assert(std::string(bytes.begin(),bytes.begin()+8)=="PSIMMAP2");
  Registry::write(path,loaded.graph,321,30);
  {std::ifstream in(path,std::ios::binary);assert(std::vector<uint8_t>((std::istreambuf_iterator<char>(in)),{})==bytes);}
  rejects([&]{Registry::write(path,loaded.graph,322,20);},"mapping_graph_export_timing");assert(Registry::read(path).request==321);
  auto envelope=Json::from_cbor(bytes.begin()+48,bytes.end());
  auto malformed=[&](Json value,const char* signature="PSIMMAP2") {
    const auto body=Json::to_cbor(value);posim_slam::InputDigest digest;digest.update(body.data(),body.size());const auto hash=digest.finish();
    const auto bad=directory/"bad-window.graph.pmap";std::ofstream out(bad,std::ios::binary);
    out.write(signature,8);out.write(reinterpret_cast<const char*>(hash.data()),hash.size());
    uint8_t size[8];for(int i=0;i<8;++i)size[i]=uint8_t(uint64_t(body.size())>>(8*i));out.write(reinterpret_cast<const char*>(size),8);
    out.write(reinterpret_cast<const char*>(body.data()),body.size());out.close();return bad;
  };
  auto changed=envelope;changed["native_window"]=Checkpoint::document(window);
  rejects([&]{Registry::read(malformed(changed));},"native_window_checkpoint_state");
  changed=envelope;changed.erase("native_window");
  rejects([&]{Registry::read(malformed(changed));},"mapping_graph_archive_window");
  rejects([&]{Registry::read(malformed(envelope,"PSIMMAP1"));},"mapping_graph_archive_window");
  changed=envelope;changed["native_window"]["model"]["reference"]=20;
  rejects([&]{Registry::read(malformed(changed));},"native_window_checkpoint_digest");
  changed=envelope;changed["graph"]["reference"][0]=20;
  rejects([&]{Registry::read(malformed(changed));},"native_window_checkpoint_reference");
  auto regular=registry.capture(fixture.input,*mapper,10,Sophus::SE3d(),Sophus::SE3d());
  assert(!regular->hasNativeWindow());
  rejects([&]{Checkpoint::restore(*mapper,regular);},"native_window_checkpoint_missing");
  int cancelled=0;const auto before=posim_slam::nativeMappingStateIdentity(*restored);
  rejects([&]{Checkpoint::restore(*restored,loaded.graph,{},[&]{return ++cancelled==3;});},"native_window_cancelled");
  assert(posim_slam::nativeMappingStateIdentity(*restored)==before);
  std::cout<<"Native archive: immutable graph/window capture, owner binding, versioned export, byte-identical re-export, mismatched pairs and cancellation pass.\n";
}

void inputPageControls(const std::filesystem::path& directory) {
  Fixture fixture;basalt::HashBow<256> vocabulary(fixture.config.mapper_bow_num_bits);
  for(auto& [stamp,source]:fixture.input->frames) {
    auto frame=std::make_shared<posim_slam::MappingInputStore::Frame>(*source.read());auto& f=frame->features;
    f.corners.emplace_back(100,100); // This detector feature has no old track.
    f.corner_angles={.1,.2,.3};f.corner_descriptors.resize(3);
    std::mt19937 random(37);
    for(auto& descriptor:f.corner_descriptors)for(size_t bit=0;bit<256;++bit)descriptor[bit]=random()&1;
    std::vector<bool> success;fixture.calibration.intrinsics[0].unproject(f.corners,f.corners_3d,success);
    vocabulary.compute_bow(f.corner_descriptors,f.hashes,f.bow_vector);source=frame;
  }
  auto mapper=fixture.mapper();
  for(const auto& [stamp,frame]:fixture.input->frames)
    mapper->hash_bow_database->add_to_database(basalt::TimeCamId(stamp,0),frame.read()->features.bow_vector);
  Registry registry(fixture.calibration,fixture.config,"input-pages");
  posim_slam::NativeMappingWindow window(*mapper,10,{10,20,30});
  const auto graph=registry.capture(fixture.input,*mapper,10,Sophus::SE3d(),Sophus::SE3d(),&window);
  const auto path=directory/"inputs.pmap";Registry::writeWithInputs(path,graph,7,30);
  const auto loaded=Registry::read(path,"input-pages");assert(loaded.graph->hasInputFrames());
  const auto restored=Registry::restoreForMatching(loaded.graph);compare(*mapper,*restored);
  auto reopened=posim_slam::NativeMappingCheckpoint::restore(*restored,loaded.graph);
  const auto original_output=posim_slam::mappingOutput(*mapper,Sophus::SE3d(),window.mapInformation(.1),graph);
  const auto restored_output=posim_slam::mappingOutput(*restored,Sophus::SE3d(),reopened->mapInformation(.1),loaded.graph);
  assert(original_output.points==restored_output.points && original_output.ids==restored_output.ids);
  for(const auto& [stamp,source]:fixture.input->frames) {
    const auto frame=Registry::inputFrame(loaded.graph,stamp);
    assert(posim_slam::mapping_frame_io::encode(*frame)==posim_slam::mapping_frame_io::encode(*source.read()));
    assert(frame->features.corners.size()==3 && restored->feature_corners.at(basalt::TimeCamId(stamp,0)).corners.size()==3);
  }
  const auto query_frame=fixture.input->frames.at(30).read();const auto& query=query_frame->features;
  using Scores=std::vector<std::pair<basalt::TimeCamId,double>>;Scores before,after;
  mapper->hash_bow_database->querry_database(query.bow_vector,10,before);
  restored->hash_bow_database->querry_database(query.bow_vector,10,after);
  std::sort(before.begin(),before.end());std::sort(after.begin(),after.end());assert(!before.empty() && before==after);
  std::vector<std::pair<int,int>> matches;
  basalt::matchDescriptors(query.corner_descriptors,restored->feature_corners.at(basalt::TimeCamId(10,0)).corner_descriptors,
      matches,fixture.config.mapper_max_hamming_distance,fixture.config.mapper_second_best_test_ratio);
  assert(matches.size()==3 && std::find(matches.begin(),matches.end(),std::pair<int,int>{2,2})!=matches.end());
  rejects([&]{Registry::inputFrame(loaded.graph,10,1);},"mapping_input_page_budget");
  rejects([&]{Registry::inputFrame(loaded.graph,99);},"mapping_input_frame_missing");
  rejects([&]{Registry::read(path,"another-epoch");},"mapping_graph_epoch_mismatch");
  int cancelled=0;rejects([&]{Registry::restoreForMatching(loaded.graph,[&]{return ++cancelled==4;});},"mapping_input_restore_cancelled");
  assert(Registry::document(loaded.graph)==Registry::document(graph));
  const auto frozen=Registry::inputFrame(loaded.graph,30)->features.corner_descriptors;
  auto changed=std::make_shared<posim_slam::MappingInputStore::Frame>(*fixture.input->frames.at(30).read());
  changed->features.corner_descriptors[2].flip();vocabulary.compute_bow(changed->features.corner_descriptors,changed->features.hashes,changed->features.bow_vector);
  fixture.input->frames[30]=changed;
  const auto next=registry.capture(fixture.input,*mapper,10,Sophus::SE3d(),Sophus::SE3d(),&window);
  Registry::writeWithInputs(path,next,8,30);
  assert(Registry::inputFrame(loaded.graph,30)->features.corner_descriptors==frozen);
  assert(Registry::inputFrame(Registry::read(path).graph,30)->features.corner_descriptors!=frozen);
  const auto repeated=directory/"reexport.pmap";Registry::writeWithInputs(repeated,loaded.graph,7,30);
  assert(Registry::inputFrame(Registry::read(repeated).graph,30)->features.corner_descriptors==frozen);
  // A corrupt unrequested page is not read speculatively. Loading it rejects it.
  {std::fstream file(repeated,std::ios::in|std::ios::out|std::ios::binary);file.seekg(-1,std::ios::end);
    char value;file.read(&value,1);value^=1;file.seekp(-1,std::ios::end);file.write(&value,1);}
  const auto damaged=Registry::read(repeated);
  assert(Registry::inputFrame(damaged.graph,10)->stamp==10);
  rejects([&]{Registry::inputFrame(damaged.graph,30);},"mapping_input_page_digest");
  rejects([&]{Registry::restoreForMatching(damaged.graph);},"mapping_input_page_digest");
  rejects([&]{posim_slam::mapping_frame_io::decode(posim_slam::mapping_frame_io::encode(*changed),Registry::document(graph).at("frames")[0]);},"mapping_input_frame_identity");
  const auto legacy=directory/"without-inputs.pmap";Registry::write(legacy,graph,7,30);
  const auto old=Registry::read(legacy);assert(!old.graph->hasInputFrames());
  rejects([&]{Registry::restoreForMatching(old.graph);},"mapping_input_frames_missing");
  rejects([&]{Registry::writeWithInputs(path,old.graph,9,30);},"mapping_input_frames_missing");
  assert(Registry::read(path).request==8);
  std::cout<<"Input pages: exact unused features/descriptors, HashBoW and descriptor matching, unchanged map/covariance, lazy integrity, budgets, cancellation, replacement ownership and legacy rejection pass.\n";
}

int main(int argc,char** argv) {
  if(argc!=2)return 2;const std::filesystem::path directory=argv[1];Fixture fixture;
  inputPageControls(directory);
  nativeArchiveControls(fixture,directory);
  nativeRowsControls(fixture);nativeRowStreamControls(fixture);nativeScheduleControls(fixture);
  nativeWindowControls(fixture);
  nativeWorldControls(fixture);
  nativeCheckpointControls(fixture);
  Registry registry(fixture.calibration,fixture.config,"fixture-segment");auto original=fixture.mapper();
  const auto first=registry.capture(fixture.input,*original,10,original->getFramePoses().at(10).getPose(),Sophus::SE3d());
  assert(first->stable_ids.at(42)==0 && first->stable_ids.at(99)==1);
  {
    posim_slam::NativeMappingWindow window(*original,10,{10,20,30});
    const auto information=window.mapInformation(.1);
    const auto output=posim_slam::mappingOutput(*original,Sophus::SE3d(),information,first);
    assert(output.candidates.size()==2 && output.nonfinite==1 && information.observations==6);
    assert(output.ids.size()==output.points.size());
    for(const auto& point:output.candidates) {
      assert(point.id==first->stable_ids.at(point.native_id));
      if(!point.finite)assert(!point.eligible && !point.observed && !point.mature);
    }
    auto no_reference=fixture.mapper();no_reference->config.vio_init_pose_weight=0;
    posim_slam::NativeMappingWindow unbounded(*no_reference,10,{10,20,30});
    const auto uncertain=unbounded.mapInformation(.1);
    assert(!uncertain.points.at(42).world.observed && !uncertain.points.at(42).eligible);
    rejects([&]{window.mapInformation(0);},"mapping_world_uncertainty_limit");
    // NfrMapper copies configuration into its live likelihood parameters.
    // Geometry must use the same live noise/Huber fields as the optimizer.
    const auto configured_noise=original->config.mapper_obs_std_dev;
    original->config.mapper_obs_std_dev*=2;
    assert(window.mapInformation(.1).points.at(42).geometry.relative_range_sigma==
        information.points.at(42).geometry.relative_range_sigma);
    original->config.mapper_obs_std_dev=configured_noise;
    const auto noise=original->obs_std_dev;original->obs_std_dev*=2;
    rejects([&]{window.mapInformation(.1);},"native_window_output_source_changed");
    original->obs_std_dev=noise;
    // A weak two-view point stays in output; its support cannot be promoted
    // by the global covariance being finite or by requesting a larger limit.
    auto support=information.points.at(42);support.geometry.exposures=2;
    support.geometry.eligible=true;support.world.observed=true;support.world.maximum_sigma_m=0;
    posim_slam::certifyMappingPoint(support,.1);assert(!support.mature && !support.eligible);
  }
  auto restored=Registry::restore(Registry::document(first));compare(*original,*restored);
  auto renumbered=fixture.mapper(7,3);const auto second=registry.capture(fixture.input,*renumbered,10,renumbered->getFramePoses().at(10).getPose(),Sophus::SE3d());
  assert(second->stable_ids.at(7)==0 && second->stable_ids.at(3)==1 && Registry::document(first).at("graph_revision")==1);
  auto duplicate=fixture.mapper();duplicate->feature_tracks[99].at(basalt::TimeCamId(20,0))=0;
  rejects([&]{registry.capture(fixture.input,*duplicate,10,Sophus::SE3d(),Sophus::SE3d());},"mapping_graph_observation_reused");
  auto changed=fixture.mapper();changed->feature_corners.at(basalt::TimeCamId(10,0)).corners[0].x()+=.01;
  rejects([&]{registry.capture(fixture.input,*changed,10,Sophus::SE3d(),Sophus::SE3d());},"mapping_source_pixel_changed");
  auto invalid_information=fixture.mapper();invalid_information->rel_pose_factors[0].cov_inv(0,0)=-1;
  rejects([&]{registry.capture(fixture.input,*invalid_information,10,Sophus::SE3d(),Sophus::SE3d());},"mapping_graph_information");
  auto invalid_pose=fixture.mapper();invalid_pose->getFramePoses().at(20)=basalt::PoseStateWithLin<double>(20,
    Sophus::SE3d(Sophus::SO3d(),Eigen::Vector3d(std::numeric_limits<double>::quiet_NaN(),0,0)));
  rejects([&]{registry.capture(fixture.input,*invalid_pose,10,Sophus::SE3d(),Sophus::SE3d());},"mapping_graph_nonfinite_input");
  auto invalid_optimizer=fixture.mapper();invalid_optimizer->lambda=-1;
  rejects([&]{registry.capture(fixture.input,*invalid_optimizer,10,Sophus::SE3d(),Sophus::SE3d());},"mapping_graph_optimizer");
  const auto frozen=Registry::document(first);
  original->getFramePoses().clear();original->feature_tracks.clear();original.reset();
  assert(Registry::document(first)==frozen); // Snapshot owns its values, independent of the source mapper.
  const auto third=registry.capture(fixture.input,*renumbered,10,renumbered->getFramePoses().at(10).getPose(),Sophus::SE3d());assert(Registry::document(third).at("graph_revision")==3);
  const auto path=directory/"fixture.graph.pmap";Registry::write(path,third,123,30);const auto loaded=Registry::read(path,"fixture-segment");
  assert(loaded.request==123 && loaded.tracking_stamp==30 && Registry::document(loaded.graph)==Registry::document(third));auto recovered=Registry::restore(Registry::document(loaded.graph));compare(*renumbered,*recovered);
  Registry reloaded(fixture.calibration,fixture.config,"fixture-segment");reloaded.adopt(loaded.graph);
  const auto fourth=reloaded.capture(fixture.input,*renumbered,10,renumbered->getFramePoses().at(10).getPose(),Sophus::SE3d());assert(Registry::document(fourth).at("graph_revision")==4 && fourth->stable_ids==third->stable_ids);
  rejects([&]{reloaded.adopt(loaded.graph);},"mapping_graph_adoption_requires_empty_registry");
  rejects([&]{Registry::read(path,"wrong-segment");},"mapping_graph_epoch_mismatch");
  auto malformed=Registry::document(third);malformed["pixels"][0][3]=Json::binary(std::vector<uint8_t>(1));
  rejects([&]{Registry::restore(malformed);},"mapping_graph_binary_shape");
  std::ifstream input(path,std::ios::binary);std::vector<char> bytes((std::istreambuf_iterator<char>(input)),{});
  const auto legacy=Json::to_cbor(Json{{"graph",Registry::document(third)},{"request_id",uint64_t(123)},{"tracking_stamp_ns",int64_t(30)}});
  assert(bytes.size()==legacy.size()+48 && std::memcmp(bytes.data()+48,legacy.data(),legacy.size())==0);
  assert(Registry::document(third)==Registry::document(loaded.graph)); // Export leaves the immutable snapshot intact.
  bytes.back()^=1;
  const auto corrupt=directory/"corrupt.graph.pmap";std::ofstream output(corrupt,std::ios::binary);output.write(bytes.data(),bytes.size());output.close();
  rejects([&]{Registry::read(corrupt);},"mapping_graph_archive_digest");
  Registry::write(path,fourth,124,30);assert(Registry::read(path).request==124);
  rejects([&]{Registry::write(path,fourth,125,20);},"mapping_graph_export_timing");assert(Registry::read(path).request==124);
  rejects([&]{Registry::write(directory/"missing"/"graph.pmap",fourth,126,30);},"mapping_graph_archive_unwritable");
  assert(Registry::read(path).request==124);
  std::cout<<"Mapping graph: exact nonlinear state/observations, stable renumbering, replacement rollback, identity adoption, epoch binding and archive corruption detection pass.\n";
}
