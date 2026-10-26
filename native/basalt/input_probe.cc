// SPDX-License-Identifier: Apache-2.0
#include "calibration.hpp"
#include "visual_support.hpp"
#include <basalt/vi_estimator/ba_base.h>
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
    basalt::BundleAdjustmentBase<double> map;
    map.calib.T_i_c.emplace_back();
    map.calib.intrinsics.emplace_back(basalt::GenericCamera<double>::fromString("pinhole"));
    map.calib.intrinsics[0].setFromInit(Eigen::Vector4d(100,100,0,0));
    map.huber_thresh=1.;map.obs_std_dev=.5;
    map.frame_poses.emplace(100,basalt::PoseStateWithLin<double>(100,Sophus::SE3d()));
    for(int id:{7,42}) {
      basalt::Keypoint<double> landmark;
      landmark.direction.setZero();landmark.inv_dist=.5;
      landmark.host_kf_id=basalt::TimeCamId(100,0);
      map.lmdb.addLandmark(id,landmark);
      for(int frame:{100,200,300}) {
        basalt::KeypointObservation<double> observation;
        observation.kpt_id=id;observation.pos.setZero();
        map.lmdb.addObservation(basalt::TimeCamId(frame,0),observation);
      }
    }
    Eigen::aligned_vector<Eigen::Vector3d> landmarks;std::vector<int> ids;
    map.get_current_points(landmarks,ids);
    Json coordinates=Json::array();for(const auto& point:landmarks)coordinates.push_back({point.x(),point.y(),point.z()});
    const int grid=processing_config(c).optical_flow_detection_grid_size;
    std::cout<<Json({{"width",resized.cols},{"height",resized.rows},
      {"max_recent_states",processing_config(c).vio_max_states},{"feature_grid",grid},{"feature_cells",(resized.cols/grid)*(resized.rows/grid)},
      {"intrinsics",{k[0],k[1],k[2],k[3]}},{"first_pixel",resized.at<uint8_t>(0,0)},
      {"estimator_first_pixel",(*pixels)(0,0)},{"visual_support",support},{"mapping_inliers",mapping_inliers(data,c.processed_width,c.processed_height,1.)},
      {"map_export",{{"ids",ids},{"points",coordinates}}}}).dump()<<'\n';
  } catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
}
