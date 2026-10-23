// SPDX-License-Identifier: Apache-2.0
#include "calibration.hpp"
#include "visual_support.hpp"
#include <iostream>
int main(int argc,char** argv) {
  if(argc!=2)return 2;
  try {
    auto c=input_calibration(argv[1]);
    cv::Mat raw(c.height,c.width,CV_8UC1);
    for(int y=0;y<c.height;++y)for(int x=0;x<c.width;++x)raw.at<uint8_t>(y,x)=(x+3*y)%256;
    auto resized=prepare_image(c,raw);auto pixels=estimator_image(c,raw);
    auto k=c.cal.intrinsics[0].getParam();
    basalt::VioVisualizationData data;
    data.opt_flow_res=std::make_shared<basalt::OpticalFlowResult>();
    data.opt_flow_res->observations.resize(1);data.projections.resize(1);
    for(int id=0;id<3;++id) {
      Eigen::AffineCompact2f point=Eigen::AffineCompact2f::Identity();point.translation()<<10,12;
      data.opt_flow_res->observations[0].emplace(id,point);
    }
    data.projections[0].emplace_back(10,12,1,0);
    data.projections[0].emplace_back(13,16,1,1);
    data.projections[0].emplace_back(10,12,-1,2);
    data.projections[0].emplace_back(10,12,1,99);
    data.projections[0].emplace_back(-1,12,1,2);
    auto support=visual_support(data,c.processed_width,c.processed_height,1.);
    std::cout<<Json({{"width",resized.cols},{"height",resized.rows},
      {"intrinsics",{k[0],k[1],k[2],k[3]}},{"first_pixel",resized.at<uint8_t>(0,0)},
      {"estimator_first_pixel",(*pixels)(0,0)},{"visual_support",support}}).dump()<<'\n';
  } catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
}
