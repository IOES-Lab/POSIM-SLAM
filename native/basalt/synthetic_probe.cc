// SPDX-License-Identifier: Apache-2.0
// Synthetic observations and truth are generated locally. Only pixels or 2D
// tracks and raw IMU enter Basalt; truth is written separately for evaluation.
#include <basalt/vi_estimator/vio_estimator.h>
#include <basalt/vi_estimator/sqrt_keypoint_vio.h>
#include "sonar_factors.hpp"
#include <tbb/global_control.h>
#include <fstream>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <random>
#include <thread>
#include <iostream>
using V=Eigen::Vector3d;
constexpr double G=9.80665;
V pos(double t){t=std::max(0.,t-1.);return V(.8*(1-std::cos(t)),.4*(1-std::cos(.7*t)),.2*(1-std::cos(.5*t)));}
V acc(double t){if(t<=1)return V::Zero();t-=1;return V(.8*std::cos(t),.196*std::cos(.7*t),.05*std::cos(.5*t));}
Eigen::Matrix3d rot(double t){t=std::max(0.,t-1.);return (Eigen::AngleAxisd(.2*(1-std::cos(.8*t)),V::UnitZ())*Eigen::AngleAxisd(.08*(1-std::cos(.9*t)),V::UnitY())*Eigen::AngleAxisd(.06*(1-std::cos(.6*t)),V::UnitX())).toRotationMatrix();}
V omega(double t){if(t<=1)return V::Zero();t-=1.;double roll=.06*(1-std::cos(.6*t)),pitch=.08*(1-std::cos(.9*t));double dr=.036*std::sin(.6*t),dp=.072*std::sin(.9*t),dy=.16*std::sin(.8*t);return V(dr-dy*std::sin(pitch),dp*std::cos(roll)+dy*std::sin(roll)*std::cos(pitch),-dp*std::sin(roll)+dy*std::cos(roll)*std::cos(pitch));}
int main(int argc,char**argv){
 if(argc<3||argc>5)return 2; const bool images=argc>=4 && std::string(argv[3])=="images";
 const bool sparse_sonar=argc>=4 && std::string(argv[3])=="sonar-2hz";
 const bool asynchronous=sparse_sonar || (argc>=4 && std::string(argv[3])=="sonar-async"); const bool acoustic=asynchronous || (argc>=4 && std::string(argv[3])=="sonar"); const double camera_hz=std::stod(argv[2]);
 if(!std::isfinite(camera_hz)||camera_hz!=(sparse_sonar?10:5)||(argc>=4 && !images && !acoustic))return 2;
 tbb::global_control threads(tbb::global_control::max_allowed_parallelism,2);
 basalt::VioConfig cfg; cfg.vio_max_iterations=7;
 // Keep the same three-camera time horizon when each camera has an auxiliary state.
 if(asynchronous)cfg.vio_max_states=sparse_sonar?16:6;
 if(argc==5) {
   if(!sparse_sonar)return 2;
   cfg.vio_max_states=std::stoi(argv[4]);
   if(cfg.vio_max_states<3 || cfg.vio_max_states>64)return 2;
 }
 const int camera_frames=int(12*camera_hz);
 const int sonar_stride=sparse_sonar?5:1;
 basalt::Calibration<double> cal;
 auto cam=basalt::GenericCamera<double>::fromString("pinhole");
 cam.setFromInit(Eigen::Vector4d(400,400,320,240)); cal.intrinsics.push_back(cam);cal.resolution.emplace_back(640,480);
 Eigen::Matrix3d Ric;Ric<<0,0,1,-1,0,0,0,-1,0;
 cal.T_i_c.emplace_back(Ric,V(.1,0,0));
 cal.imu_update_rate=200;cal.gyro_noise_std.setConstant(.001);cal.accel_noise_std.setConstant(.02);
 cal.gyro_bias_std.setConstant(.0001);cal.accel_bias_std.setConstant(.001);
 auto vio=basalt::VioEstimatorFactory::getVioEstimator(cfg,cal,V(0,0,-G),true,true);
 // Synthetic acoustic measurements are generated before estimation. The
 // owner receives only range/azimuth, with known fixture correspondence.
 std::map<int64_t,posim_slam::SonarObservation> acoustic_measurements;
 std::mt19937 acoustic_rng(83);std::normal_distribution<double> acoustic_noise;
 for(int frame=0;frame<=camera_frames;frame+=sonar_stride) {
   const double t=frame/camera_hz+(asynchronous?.01:0.);const int64_t stamp=1000000000LL+int64_t(std::llround(t*1e9));
   const V local=rot(t).transpose()*(V(7.,.4,.2)-pos(t));
   acoustic_measurements.emplace(stamp,posim_slam::SonarObservation{stamp,
     local.norm()+.02*acoustic_noise(acoustic_rng),std::atan2(local.y(),local.x())+.005*acoustic_noise(acoustic_rng)});
 }
 int registered_tracks=0,retired_tracks=0,accepted_acoustic_steps=0,expired_seeds=0;
 std::vector<posim_slam::SonarObservation> acoustic_track;
 int64_t active_track=-1;
 auto owner=std::dynamic_pointer_cast<basalt::SqrtKeypointVioEstimator<double>>(vio);
 if(acoustic) {
   if(!owner)throw std::runtime_error("synthetic_joint_owner_unavailable");
   owner->posim_sonar_factors=std::make_shared<posim_slam::SonarFactorStore>();
   owner->posim_before_marginalization=[&](auto& estimator,int64_t stamp) {
     auto& base=static_cast<basalt::BundleAdjustmentBase<double>&>(estimator);
     if(base.lmdb.numLandmarks()==0 || !acoustic_measurements.count(stamp))return;
     auto& store=*estimator.posim_sonar_factors;
     if(active_track>=0 && !store.contains(active_track)) {
       ++retired_tracks;active_track=-1;acoustic_track.clear();
     }
     acoustic_track.push_back(acoustic_measurements.at(stamp));
     if(acoustic_track.size()<3)return;
     posim_slam::SonarPoses poses;
     for(const auto& [time,state]:base.frame_poses) {
       const auto pose=state.getPose();poses.emplace(time,posim_slam::SonarPose{pose.translation(),pose.rotationMatrix()});
     }
     for(const auto& [time,state]:base.frame_states) {
       const auto pose=state.getState().T_w_i;poses.emplace(time,posim_slam::SonarPose{pose.translation(),pose.rotationMatrix()});
     }
     // A frontend must not submit observations whose estimator poses expired.
     // Count and prune stale seed observations so a short horizon fails clearly.
     if(active_track<0 && std::any_of(acoustic_track.begin(),acoustic_track.end(),
         [&](const auto& observation){return !poses.count(observation.stamp);})) {
       ++expired_seeds;
       acoustic_track.erase(std::remove_if(acoustic_track.begin(),acoustic_track.end(),
         [&](const auto& observation){return !poses.count(observation.stamp);}),acoustic_track.end());
       return;
     }
     Eigen::MatrixXd covariance=Eigen::MatrixXd::Zero(2*acoustic_track.size(),2*acoustic_track.size());
     for(size_t i=0;i<acoustic_track.size();++i) {covariance(2*i,2*i)=.02*.02;covariance(2*i+1,2*i+1)=.005*.005;}
     if(active_track<0)active_track=registered_tracks++;
     store.applySnapshot(stamp,{{active_track,acoustic_track,covariance}},
       {V::Zero(),Eigen::Matrix3d::Identity(),.2,10.,2.1,1.0},poses);
   };
   owner->posim_after_joint=[&](auto& estimator,int64_t) {
     if(estimator.posim_joint_started)accepted_acoustic_steps+=estimator.posim_joint_last_accepted;
   };
 }
 tbb::concurrent_bounded_queue<basalt::PoseVelBiasState<double>::Ptr> output;
 vio->out_state_queue=&output;
 std::ofstream os(argv[1]);os<<"time,x,y,z,qx,qy,qz,qw,vx,vy,vz,bgx,bgy,bgz,bax,bay,baz,gtx,gty,gtz,gtqx,gtqy,gtqz,gtqw\n";
 std::vector<int64_t> output_stamps;
 std::thread reader([&]{for(;;){basalt::PoseVelBiasState<double>::Ptr s;output.pop(s);if(!s)break;output_stamps.push_back(s->t_ns);auto p=s->T_w_i.translation();auto q=s->T_w_i.unit_quaternion();auto gt=pos(s->t_ns*1e-9-1.);Eigen::Quaterniond gtq(rot(s->t_ns*1e-9-1.));os<<s->t_ns*1e-9;for(double v:{p.x(),p.y(),p.z(),q.x(),q.y(),q.z(),q.w(),s->vel_w_i.x(),s->vel_w_i.y(),s->vel_w_i.z(),s->bias_gyro.x(),s->bias_gyro.y(),s->bias_gyro.z(),s->bias_accel.x(),s->bias_accel.y(),s->bias_accel.z(),gt.x(),gt.y(),gt.z(),gtq.x(),gtq.y(),gtq.z(),gtq.w()})os<<','<<v;os<<'\n';}});
 basalt::OpticalFlowBase::Ptr flow; if(images){flow=basalt::OpticalFlowFactory::getOpticalFlow(cfg,cal);flow->output_queue=&vio->vision_data_queue;}
 vio->initialize(V::Zero(),V::Zero());
 std::thread inertial([&]{std::mt19937 rng(7);std::normal_distribution<double> n;for(int i=0;i<=2420;i++){double t=i*.005;auto d=std::make_shared<basalt::ImuData<double>>();d->t_ns=1000000000LL+i*5000000LL;d->gyro=omega(t)+V(.003,-.002,.004);d->accel=rot(t).transpose()*(acc(t)+V(0,0,G))+V(.025,-.015,.01);for(int k=0;k<3;k++){d->gyro[k]+=.001*n(rng);d->accel[k]+=.02*n(rng);}vio->imu_data_queue.push(d);}vio->imu_data_queue.push(nullptr);});
 std::mt19937 rng(4);std::uniform_real_distribution<double> x(3,10),y(-4,4),z(-3,3);std::normal_distribution<double> noise(0,.3);std::vector<V> pts;for(int k=0;k<400;k++)pts.emplace_back(x(rng),y(rng),z(rng));
 struct Tile{V center;cv::Mat texture;};std::vector<Tile> tiles;cv::RNG pixels(5432);for(int iy=-5;iy<=5;iy++)for(int iz=-4;iz<=4;iz++){cv::Mat texture(48,48,CV_8UC1);pixels.fill(texture,cv::RNG::UNIFORM,20,235);cv::GaussianBlur(texture,texture,cv::Size(3,3),.6);tiles.push_back({V(x(rng),iy*.65,iz*.65),texture});}std::sort(tiles.begin(),tiles.end(),[](auto&a,auto&b){return a.center.x()>b.center.x();});
 for(int frame=0;frame<=int(12*camera_hz);frame++){double t=frame/camera_hz;auto f=std::make_shared<basalt::OpticalFlowResult>();f->t_ns=1000000000LL+int64_t(std::llround(t*1e9));f->observations.resize(1);f->pyramid_levels.resize(1);auto T=Sophus::SE3d(rot(t),pos(t))*cal.T_i_c[0];for(size_t k=0;k<pts.size();k++){V pc=T.inverse()*pts[k];Eigen::Vector2d uv;if(!cam.project(pc,uv)||uv.x()<8||uv.x()>632||uv.y()<8||uv.y()>472)continue;Eigen::AffineCompact2f affine=Eigen::AffineCompact2f::Identity();affine.translation()=Eigen::Vector2f(uv.x()+noise(rng),uv.y()+noise(rng));f->observations[0].emplace(k,affine);f->pyramid_levels[0][k]=0;}if(!images){vio->vision_data_queue.push(f);
 if(asynchronous && frame<camera_frames && frame%sonar_stride==0) {
   auto tick=std::make_shared<basalt::OpticalFlowResult>();tick->t_ns=f->t_ns+10000000;
   vio->vision_data_queue.push(tick);
 }
 continue;}
 cv::Mat gray(480,640,CV_8UC1,cv::Scalar(10));for(auto& tile:tiles){std::vector<cv::Point2f> dst,src{{0,0},{47,0},{47,47},{0,47}};for(auto yz:std::vector<Eigen::Vector2d>{{-.27,-.27},{.27,-.27},{.27,.27},{-.27,.27}}){V pc=T.inverse()*(tile.center+V(0,yz.x(),yz.y()));Eigen::Vector2d uv;if(!cam.project(pc,uv))break;dst.emplace_back(uv.x(),uv.y());}if(dst.size()!=4)continue;auto H=cv::getPerspectiveTransform(src,dst);cv::Mat warped,mask;cv::warpPerspective(tile.texture,warped,H,gray.size());cv::warpPerspective(cv::Mat(48,48,CV_8UC1,cv::Scalar(255)),mask,H,gray.size(),cv::INTER_NEAREST);warped.copyTo(gray,mask);}
 auto input=std::make_shared<basalt::OpticalFlowInput>();input->t_ns=f->t_ns;input->img_data.resize(1);auto managed=std::make_shared<basalt::ManagedImage<uint16_t>>(640,480);for(int yy=0;yy<480;yy++)for(int xx=0;xx<640;xx++)(*managed)(xx,yy)=uint16_t(gray.at<uint8_t>(yy,xx))<<8;input->img_data[0].img=managed;flow->input_queue.push(input);if(frame==0)cv::imwrite(std::string(argv[1])+".png",gray);
 }
 if(images)flow->input_queue.push(nullptr);else vio->vision_data_queue.push(nullptr);vio->maybe_join();vio->drain_input_queues();inertial.join();reader.join();
 if(acoustic) {
   std::vector<int64_t> expected;
   for(int frame=0;frame<=camera_frames;++frame) {
     const int64_t stamp=1000000000LL+int64_t(std::llround(frame/camera_hz*1e9));expected.push_back(stamp);
     if(asynchronous && frame<camera_frames && frame%sonar_stride==0)expected.push_back(stamp+10000000);
   }
   const bool passed=output_stamps==expected && registered_tracks>=2 && retired_tracks>=1 && accepted_acoustic_steps>0 && owner->posim_joint_started;
   std::cout<<"sonar_lifecycle camera_hz="<<camera_hz<<" sonar_hz="<<camera_hz/sonar_stride<<" max_recent_states="<<cfg.vio_max_states<<" states="<<output_stamps.size()<<" passed="<<passed<<" expired_seeds="<<expired_seeds<<" registered="<<registered_tracks<<" retired="<<retired_tracks<<" accepted_steps="<<accepted_acoustic_steps<<"\n";
   return passed?0:1;
 }
 return 0;
}
