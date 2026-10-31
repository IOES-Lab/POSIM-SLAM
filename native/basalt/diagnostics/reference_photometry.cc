// SPDX-License-Identifier: Apache-2.0
#include "calibration.hpp"
#include <basalt/optical_flow/patch_optical_flow.h>
#include <tbb/global_control.h>
#include <iostream>
#include <map>
#include <set>
using Patch=basalt::OpticalFlowPatch<float,basalt::Pattern51<float>>;
int main(int argc,char** argv) {
 if(argc!=3)return 2;
 try {
  const auto c=input_calibration(argv[1]);auto config=processing_config(c);
  if(config.optical_flow_type!="patch")throw std::runtime_error("patch_control_required");
  const auto archive=read_json(argv[2]);std::set<int> selected;
  for(const auto& id:archive.at("ids")) {
   int value=id.get<int>();if(value<0 || !selected.insert(value).second)throw std::runtime_error("diagnostic_archive_ids");
  }
  if(selected.size()>10000)throw std::runtime_error("diagnostic_archive_budget");
  tbb::global_control threads(tbb::global_control::max_allowed_parallelism,2);
  tbb::concurrent_bounded_queue<basalt::OpticalFlowResult::Ptr> output;output.set_capacity(1);
  auto flow=std::make_unique<basalt::PatchOpticalFlow<float,basalt::Pattern51>>(config,c.cal);
  flow->output_queue=&output;flow->input_queue.set_capacity(1);
  std::map<int,Patch> patches;std::string line;int64_t last=-1;
  auto stop=[&](){flow->input_queue.push(nullptr);basalt::OpticalFlowResult::Ptr end;output.pop(end);flow.reset();};
  try {
  while(std::getline(std::cin,line)) {
   const auto request=Json::parse(line);const int64_t stamp=request.at("stamp_ns");
   if(stamp<0 || stamp<=last)throw std::runtime_error("diagnostic_clock");last=stamp;
   cv::Mat raw(c.height,c.width,CV_8UC1);
   for(int y=0;y<c.height;++y)std::cin.read(reinterpret_cast<char*>(raw.ptr<uint8_t>(y)),c.width);
   if(!std::cin)throw std::runtime_error("pixels_truncated");
   auto image=estimator_image(c,raw);const basalt::Image<const uint16_t> view(image->ptr,image->w,image->h,image->pitch);
   auto input=std::make_shared<basalt::OpticalFlowInput>();
   input->t_ns=stamp;input->img_data.resize(1);input->img_data[0].img=image;flow->input_queue.push(input);
   basalt::OpticalFlowResult::Ptr result;output.pop(result);
   if(!result || result->t_ns!=stamp)throw std::runtime_error("output_clock");
   Json rows=Json::array();
   for(const auto& [id,transform]:result->observations[0]) {
    if(!selected.count(id))continue;
    if(!patches.count(id))patches.emplace(id,Patch(view,transform.translation()));
    const auto& p=patches.at(id);Patch::Matrix2P pattern=transform.linear()*Patch::pattern2;
    pattern.colwise()+=transform.translation();Patch::VectorP residual;
    const bool valid=p.residual(view,pattern,residual);int count=0;
    for(int i=0;i<Patch::PATTERN_SIZE;++i)count+=p.data[i]>=0 && image->InBounds(pattern.col(i),2);
    Json row={{"id",id},{"pixel",{transform.translation()[0],transform.translation()[1]}},
     {"linear",{{transform.linear()(0,0),transform.linear()(0,1)},{transform.linear()(1,0),transform.linear()(1,1)}}},
     {"photometric_valid",valid},{"samples",count}};
    if(valid && count)row["normalized_rmse"]=std::sqrt(residual.cast<double>().squaredNorm()/count);
    rows.push_back(row);
   }
   std::cout<<Json({{"stamp_ns",stamp},{"tracks",rows},{"reference_patches",result->reference_patch_count}}).dump()<<std::endl;
  }
  }catch(...){stop();throw;}
  stop();
 }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
}
