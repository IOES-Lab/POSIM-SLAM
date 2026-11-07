// SPDX-License-Identifier: Apache-2.0
#include "reference_patches.hpp"
#include <basalt/optical_flow/optical_flow.h>
#include <basalt/image/image.h>
#include <tbb/global_control.h>
#include <cassert>
#include <cstdint>
#include <iostream>
#include <map>
#include <set>

int main() {
  std::map<int,int> references{{1,0},{2,0},{3,0}};
  std::vector<std::map<int,int>> views{{{1,0}},{{2,0}}};
  assert(posim_slam::retain_observed_references(references,views)==1);
  views[0].clear();
  assert(posim_slam::retain_observed_references(references,views)==1 && references.count(2));
  views[1].clear();
  assert(posim_slam::retain_observed_references(references,views)==1 && references.empty());

  tbb::global_control threads(tbb::global_control::max_allowed_parallelism,2);
  constexpr int width=128,height=128,grid=16,frames=400;
  const std::size_t bound=2*((width/grid)+1)*((height/grid)+1);
  basalt::Calibration<double> calibration;
  auto camera=basalt::GenericCamera<double>::fromString("pinhole");
  camera.setFromInit(Eigen::Vector4d(100,100,width/2,height/2));
  calibration.intrinsics.push_back(camera);calibration.resolution.emplace_back(width,height);
  calibration.T_i_c.emplace_back();
  basalt::VioConfig config;config.optical_flow_type="patch";
  config.optical_flow_levels=2;config.optical_flow_detection_grid_size=grid;
  config.optical_flow_skip_frames=1;
  tbb::concurrent_bounded_queue<basalt::OpticalFlowResult::Ptr> output;
  output.set_capacity(1);
  auto flow=basalt::OpticalFlowFactory::getOpticalFlow(config,calibration);
  flow->output_queue=&output;flow->input_queue.set_capacity(1);
  std::size_t peak=0,blank_frames=0,retained_identity_frames=0,positive_frames=0;
  std::set<basalt::KeypointId> previous;
  for(int frame=0;frame<frames;++frame) {
    auto image=std::make_shared<basalt::ManagedImage<uint16_t>>(width,height);
    const bool blank=frame%4==3;
    uint32_t random=uint32_t(frame/4*2+(frame%4==2?2:1));
    for(int y=0;y<height;++y)for(int x=0;x<width;++x) {
      random=1664525*random+1013904223;
      (*image)(x,y)=blank?0:uint16_t((random>>24)<<8);
    }
    auto input=std::make_shared<basalt::OpticalFlowInput>();
    input->t_ns=100000000LL*(frame+1);input->img_data.resize(1);input->img_data[0].img=image;
    flow->input_queue.push(input);
    basalt::OpticalFlowResult::Ptr result;output.pop(result);
    assert(result && result->t_ns==input->t_ns && result->observations.size()==1);
    const auto& observed=result->observations[0];
    assert(result->reference_patch_count==observed.size());
    assert(result->reference_patch_peak<=bound);
    peak=std::max(peak,result->reference_patch_peak);
    positive_frames+=!observed.empty();
    if(blank){assert(observed.empty() && result->reference_patch_count==0);++blank_frames;}
    if(frame%4==1)for(const auto& item:observed)if(previous.count(item.first)){++retained_identity_frames;break;}
    previous.clear();for(const auto& item:observed)previous.insert(item.first);
  }
  flow->input_queue.push(nullptr);
  basalt::OpticalFlowResult::Ptr end;output.pop(end);assert(!end);flow.reset();
  assert(positive_frames>0 && retained_identity_frames>0 && blank_frames==100);
  std::cout<<"{\"passed\":true,\"frames\":"<<frames<<",\"blank_frames\":"<<blank_frames
           <<",\"reference_patch_peak\":"<<peak<<",\"peak_bound\":"<<bound
           <<",\"retained_identity_frames\":"<<retained_identity_frames<<"}\n";
}
