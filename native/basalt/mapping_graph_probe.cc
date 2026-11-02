// SPDX-License-Identifier: Apache-2.0
#include "mapping_graph.hpp"
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
      map->feature_corners.emplace(basalt::TimeCamId(stamp,0),frame->features);
      map->feature_tracks[first].emplace(basalt::TimeCamId(stamp,0),0);map->feature_tracks[second].emplace(basalt::TimeCamId(stamp,0),1);
    }
    for(auto [id,index]:{std::pair<int,int>{first,0},{second,1}}) {
      basalt::Keypoint<double> point;point.host_kf_id=basalt::TimeCamId(10,0);point.direction=Eigen::Vector2d(.1*index,-.1*index);point.inv_dist=index?0:.25;
      map->lmdb.addLandmark(id,point);
      for(int64_t stamp:{10,20,30}) {
        basalt::KeypointObservation<double> obs;obs.kpt_id=id;obs.pos=input->frames.at(stamp)->features.corners[index];map->lmdb.addObservation(basalt::TimeCamId(stamp,0),obs);
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
} // namespace

int main(int argc,char** argv) {
  if(argc!=2)return 2;const std::filesystem::path directory=argv[1];Fixture fixture;
  Registry registry(fixture.calibration,fixture.config,"fixture-segment");auto original=fixture.mapper();
  const auto first=registry.capture(fixture.input,*original,10,original->getFramePoses().at(10).getPose(),Sophus::SE3d());
  assert(first->stable_ids.at(42)==0 && first->stable_ids.at(99)==1);
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
