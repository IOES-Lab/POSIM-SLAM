// SPDX-License-Identifier: Apache-2.0
#include "mapping_input_store.hpp"
#include "mapping_frame_io.hpp"
#include "mapping_feature_matcher.hpp"
#include <sqlite3.h>
#include <cassert>
#include <iostream>
#include <random>

namespace {
using Store=posim_slam::MappingInputStore;
basalt::MargData::Ptr sample(int64_t first=10,int64_t second=20,bool inertial=false) {
  auto m=std::make_shared<basalt::MargData>();m->use_imu=true;
  m->kfs_all={first,second};m->kfs_to_marg={first};
  m->aom.abs_order_map={{first,{0,6}},{second,{6,inertial?15:6}}};
  m->aom.total_size=inertial?21:12;m->aom.items=2;
  m->abs_H=2*Eigen::MatrixXd::Identity(m->aom.total_size,m->aom.total_size);
  m->abs_b=Eigen::VectorXd::LinSpaced(m->aom.total_size,-.1,.1);
  if(inertial) {
    // Velocity and bias cross information must survive the same elimination
    // as the original retained-input mapper, rather than being discarded.
    m->abs_H(0,12)=m->abs_H(12,0)=.25;
    m->abs_H(4,17)=m->abs_H(17,4)=.3;
  }
  for(int64_t stamp:m->kfs_all) {
    if(inertial && stamp==second)
      m->frame_states.emplace(stamp,basalt::PoseVelBiasStateWithLin<double>(stamp,
        Sophus::SE3d(),Eigen::Vector3d(.1,.2,.3),Eigen::Vector3d(.01,.02,.03),Eigen::Vector3d(.04,.05,.06),false));
    else m->frame_poses.emplace(stamp,basalt::PoseStateWithLin<double>(stamp,Sophus::SE3d()));
    auto f=std::make_shared<basalt::OpticalFlowResult>();f->t_ns=stamp;
    f->input_images=std::make_shared<basalt::OpticalFlowInput>();f->input_images->t_ns=stamp;
    f->input_images->img_data.resize(1);
    f->input_images->img_data[0].img=std::make_shared<basalt::ManagedImage<uint16_t>>(4,4);
    f->input_images->img_data[0].img->Fill(100);m->opt_flow_res.push_back(f);
  }
  return m;
}
template<class F> void rejects(F action,const std::string& expected) {
  bool rejected=false;
  try {action();}catch(const std::exception& error){assert(error.what()==expected);rejected=true;}
  assert(rejected);
}
template<class A,class B> void equal(const A& a,const B& b) {
  assert(a.rows()==b.rows() && a.cols()==b.cols() && (a.array()==b.array()).all());
}
void descriptorWireFormat() {
  namespace io=posim_slam::mapping_frame_io;
  posim_slam::MappingInputFrame frame;frame.stamp=17;frame.width=160;frame.height=120;frame.digest.fill(3);
  auto& features=frame.features;features.corners.emplace_back(12.5,44.25);features.corner_angles.push_back(.25);
  features.corners_3d.emplace_back(0,0,1,1);features.corner_descriptors.resize(1);features.hashes.emplace_back(17);
  const nlohmann::json identity={17,160,120,1,io::binary(frame.digest)};
  for(size_t bit=0;bit<258;++bit) {
    auto& descriptor=features.corner_descriptors.front();descriptor.reset();
    std::vector<uint8_t> expected(32);
    if(bit<256){descriptor.set(bit);expected[bit/8]=uint8_t(1u<<(bit%8));}
    else if(bit==256){descriptor.set();std::fill(expected.begin(),expected.end(),255);}
    const auto bytes=io::encode(frame);const auto document=nlohmann::json::from_cbor(bytes);
    assert(document.at("descriptors")==nlohmann::json::binary(expected));
    assert(io::decode(bytes,identity)->features.corner_descriptors.front()==descriptor);
  }
  // Independent wire bytes exercise every byte and all four word boundaries.
  auto document=nlohmann::json::from_cbor(io::encode(frame));std::vector<uint8_t> wire(32);
  for(size_t byte=0;byte<wire.size();++byte)wire[byte]=uint8_t(13+37*byte);
  document["descriptors"]=nlohmann::json::binary(wire);const auto bytes=nlohmann::json::to_cbor(document);
  const auto decoded=io::decode(bytes,identity);
  for(size_t bit=0;bit<256;++bit)assert(decoded->features.corner_descriptors.front()[bit]==bool(wire[bit/8]&(1u<<(bit%8))));
  assert(io::encode(*decoded)==bytes);
  document["descriptors"]=nlohmann::json::binary(std::vector<uint8_t>(31));
  rejects([&]{io::decode(nlohmann::json::to_cbor(document),identity);},"mapping_input_frame_shape");
}
void compare(const Store::Snapshot& compact,const basalt::NfrMapper& raw) {
  assert(compact->relative.size()==raw.rel_pose_factors.size());
  assert(compact->roll_pitch.size()==raw.roll_pitch_factors.size());
  for(size_t i=0;i<compact->relative.size();++i) {
    const auto& a=compact->relative[i];const auto& b=raw.rel_pose_factors[i];
    assert(a.t_i_ns==b.t_i_ns && a.t_j_ns==b.t_j_ns);
    equal(a.T_i_j.matrix(),b.T_i_j.matrix());equal(a.cov_inv,b.cov_inv);
  }
  for(size_t i=0;i<compact->roll_pitch.size();++i) {
    const auto& a=compact->roll_pitch[i];const auto& b=raw.roll_pitch_factors[i];
    assert(a.t_ns==b.t_ns);equal(a.R_w_i_meas.matrix(),b.R_w_i_meas.matrix());equal(a.cov_inv,b.cov_inv);
  }
}
void exact(const Store::Snapshot& a,const Store::Snapshot& b) {
  assert(a->revision==b->revision && a->retired==b->retired && a->frames.size()==b->frames.size());
  assert(a->source_image_bytes==b->source_image_bytes && a->source_factor_matrix_bytes==b->source_factor_matrix_bytes &&
    a->feature_payload_bytes==b->feature_payload_bytes);
  for(const auto& [stamp,reference]:a->frames) {
    const auto first=reference.read(),second=b->frames.at(stamp).read();
    assert(posim_slam::mapping_frame_io::encode(*first)==posim_slam::mapping_frame_io::encode(*second));
    equal(a->poses.at(stamp).getPose().matrix(),b->poses.at(stamp).getPose().matrix());
  }
  basalt::Calibration<double> calibration;basalt::VioConfig config;basalt::NfrMapper mapper(calibration,config);
  mapper.rel_pose_factors=b->relative;mapper.roll_pitch_factors=b->roll_pitch;compare(a,mapper);
}
basalt::MargData::Ptr textured(int64_t first,int64_t second) {
  auto source=sample(first,second,true);
  for(auto& flow:source->opt_flow_res) {
    auto image=std::make_shared<basalt::ManagedImage<uint16_t>>(160,120);std::mt19937 random(uint32_t(flow->t_ns));
    for(size_t y=0;y<image->h;++y)for(size_t x=0;x<image->w;++x)(*image)(x,y)=uint16_t(random()%256)<<8;
    flow->input_images->img_data[0].img=image;
  }
  return source;
}
void sql(const std::filesystem::path& path,const char* command) {
  sqlite3* db=nullptr;assert(sqlite3_open_v2(path.string().c_str(),&db,SQLITE_OPEN_READWRITE,nullptr)==SQLITE_OK);
  assert(sqlite3_exec(db,command,nullptr,nullptr,nullptr)==SQLITE_OK);assert(sqlite3_close(db)==SQLITE_OK);
}
void durable(const std::filesystem::path& path,const std::string& mode) {
  basalt::Calibration<double> calibration;auto camera=basalt::GenericCamera<double>::fromString("pinhole");
  camera.setFromInit(Eigen::Vector4d(100,100,80,60));calibration.intrinsics.push_back(camera);
  calibration.T_i_c.emplace_back();calibration.resolution.emplace_back(160,120);basalt::VioConfig config;
  Store memory(calibration,config);memory.append(*textured(10,20));memory.append(*textured(20,30));
  if(mode=="create") {
    Store::Snapshot held;
    {
      Store disk(calibration,config,{},path,false,"durable");const auto first=disk.append(*textured(10,20));
      held=disk.append(*textured(20,30));exact(memory.snapshot(),held);
      assert(first->revision==1 && !first->frames.count(30));
      for(const auto& [stamp,reference]:held->frames) {
        assert(!reference.resident());auto lease=reference.read();assert(!lease->features.corners.empty());
        std::weak_ptr<const Store::Frame> released=lease;lease.reset();assert(released.expired());
      }
      rejects([&]{held->frames.at(10).read(1);},"mapping_input_page_budget");
      compare(held,*memory.makeMapper(memory.snapshot()));
      rejects([&]{Store duplicate(calibration,config,{},path,false,"durable");},"mapping_input_storage_path");
    }
    // Immutable snapshots keep the backing connection alive after the producer.
    exact(memory.snapshot(),held);
  } else {
    Store disk(calibration,config,{},path,true,"durable");const auto before=disk.snapshot();exact(memory.snapshot(),before);
    rejects([&]{disk.append(*textured(20,30));},"mapping_retirement_reused");
    rejects([&]{Store other(calibration,config,{},path,true,"another-epoch");},"mapping_input_storage_identity");
    auto wrong=config;wrong.mapper_detection_num_points+=1;
    rejects([&]{Store other(calibration,wrong,{},path,true,"durable");},"mapping_input_storage_identity");
    // The failure occurs after new frame rows have been inserted, before the
    // retirement commit. Both on-disk rows and the published snapshot roll back.
    sql(path,"CREATE TRIGGER fail_batch BEFORE INSERT ON batches WHEN NEW.revision=3 BEGIN SELECT RAISE(ABORT,'injected'); END;");
    rejects([&]{disk.append(*textured(30,40));},"mapping_input_storage:19");assert(disk.snapshot()==before);
    {Store reopened(calibration,config,{},path,true,"durable");exact(before,reopened.snapshot());}
    sql(path,"DROP TRIGGER fail_batch;");
    const auto committed=disk.append(*textured(30,40));memory.append(*textured(30,40));exact(memory.snapshot(),committed);
    // Retrying succeeds only if the aborted frame insertion was also undone.
    assert(before->revision==2 && !before->frames.count(40));
    const auto path_copy=path.string()+".damaged";std::filesystem::copy_file(path,path_copy);
    {Store relocated(calibration,config,{},path_copy,true,"durable");exact(committed,relocated.snapshot());}
    sql(path_copy,"UPDATE frames SET value=zeroblob(length(value)) WHERE stamp=40;");
    {
      Store damaged(calibration,config,{},path_copy,true,"durable");
      assert(damaged.snapshot()->frames.at(10).read()->stamp==10);
      rejects([&]{damaged.makeMapper(damaged.snapshot());},"mapping_input_page_digest");
    }
    sql(path_copy,"UPDATE batches SET value=zeroblob(length(value)) WHERE revision=3;");
    rejects([&]{Store damaged(calibration,config,{},path_copy,true,"durable");},"mapping_input_batch_digest");
  }
  std::cout<<"Durable inputs "<<mode<<": exact features/factors/poses, immutable page ownership and transaction controls pass.\n";
}
} // namespace

int main(int argc,char** argv) {
  descriptorWireFormat();
  if(argc==3){durable(std::filesystem::absolute(argv[2]),argv[1]);return 0;}
  basalt::Calibration<double> calibration;calibration.intrinsics.resize(1);calibration.T_i_c.resize(1);
  calibration.resolution.push_back(Eigen::Vector2i(4,4));
  basalt::VioConfig config;
  for(bool inertial:{false,true}) {
    Store store(calibration,config);auto source=sample(10,20,inertial);
    const Eigen::MatrixXd h=source->abs_H;const Eigen::VectorXd b=source->abs_b;
    const auto pixel_hash=Store::imageDigest(*source->opt_flow_res[0]->input_images->img_data[0].img);
    std::weak_ptr<basalt::ManagedImage<uint16_t>> borrowed=source->opt_flow_res[0]->input_images->img_data[0].img;
    const auto first=store.append(*source);equal(h,source->abs_H);equal(b,source->abs_b);
    assert(pixel_hash==Store::imageDigest(*source->opt_flow_res[0]->input_images->img_data[0].img));
    basalt::NfrMapper raw(calibration,Store::configuration(config));
    auto owned=std::make_shared<basalt::MargData>(*source);raw.addMargData(owned);compare(first,raw);
    source.reset();owned.reset();raw.img_data.clear();assert(borrowed.expired());
    auto second=sample(20,30);store.append(*second);
    assert(first->revision==1 && first->frames.size()==2 && !first->frames.count(30));
    assert(store.snapshot()->revision==2 && store.snapshot()->frames.size()==3);
    assert(store.makeMapper(first)->img_data.empty());
    const auto before=store.snapshot();rejects([&]{store.append(*second);},"mapping_retirement_reused");
    assert(store.snapshot()==before);
    Store other(calibration,config);
    rejects([&]{other.makeMapper(first);},"mapping_input_snapshot_owner");
    rejects([&]{store.makeMapper(nullptr);},"mapping_input_snapshot_owner");
    auto matching=store.makeMatchHistory();auto matched=store.makeMatchedMapper(first,*matching);
    assert(matching->snapshot()->new_frames==2 && matched->getFramePoses().size()==2);
    store.makeMatchedMapper(before,*matching);assert(matching->snapshot()->new_frames==1);
    const auto retained=matching->snapshot();store.makeMatchedMapper(before,*matching);assert(matching->snapshot()==retained);
    auto wrong=other.makeMatchHistory();
    rejects([&]{store.makeMatchedMapper(before,*wrong);},"mapping_input_snapshot_owner");
    rejects([&]{other.makeMatchedMapper(before,*matching);},"mapping_input_snapshot_owner");
  }
  {
    Store store(calibration,config);const auto first=store.append(*sample());
    auto changed=sample();changed->kfs_to_marg={20};
    (*changed->opt_flow_res[0]->input_images->img_data[0].img)(0,0)=101;
    rejects([&]{store.append(*changed);},"mapping_image_identity_changed");assert(store.snapshot()==first);
  }
  {
    posim_slam::MappingInputLimits limits;limits.compact_bytes=2;
    Store store(calibration,config,limits);const auto empty=store.snapshot();
    rejects([&]{store.append(*sample());},"mapping_compact_input_capacity");assert(store.snapshot()==empty);
  }
  {
    posim_slam::MappingInputLimits limits;limits.keyframes=2;Store store(calibration,config,limits);
    auto source=sample();const auto extra=sample(20,30);source->kfs_all.insert(30);
    source->frame_poses.emplace(30,extra->frame_poses.at(30));source->opt_flow_res.push_back(extra->opt_flow_res.back());
    source->aom.abs_order_map.emplace(30,std::make_pair(12,6));source->aom.total_size=18;source->aom.items=3;
    source->abs_H=2*Eigen::MatrixXd::Identity(18,18);source->abs_b=Eigen::VectorXd::LinSpaced(18,-.1,.1);
    const auto empty=store.snapshot();rejects([&]{store.append(*source);},"mapping_image_capacity");assert(store.snapshot()==empty);
  }
  {
    posim_slam::MappingInputLimits limits;limits.retirements=limits.update_retirements=1;
    Store store(calibration,config,limits);const auto first=store.append(*sample());
    rejects([&]{store.append(*sample(20,30));},"mapping_retirement_capacity");assert(store.snapshot()==first);
  }
  std::cout<<"Compact mapping inputs: exact 6/15-state recovery, immutable snapshots, image ownership/identity and atomic capacity failures pass.\n";
}
