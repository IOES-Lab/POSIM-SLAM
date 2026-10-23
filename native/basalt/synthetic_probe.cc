// SPDX-License-Identifier: Apache-2.0
// Synthetic observations and truth are generated locally. Only pixels or 2D
// tracks and raw IMU enter Basalt; truth is written separately for evaluation.
#include <basalt/vi_estimator/vio_estimator.h>
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
 if(argc<3||argc>4)return 2; const bool images=argc==4; const double camera_hz=std::stod(argv[2]);
 if(!std::isfinite(camera_hz)||camera_hz!=5||(images&&std::string(argv[3])!="images"))return 2;
 tbb::global_control threads(tbb::global_control::max_allowed_parallelism,2);
 basalt::VioConfig cfg; cfg.vio_max_iterations=7;
 basalt::Calibration<double> cal;
 auto cam=basalt::GenericCamera<double>::fromString("pinhole");
 cam.setFromInit(Eigen::Vector4d(400,400,320,240)); cal.intrinsics.push_back(cam);cal.resolution.emplace_back(640,480);
 Eigen::Matrix3d Ric;Ric<<0,0,1,-1,0,0,0,-1,0;
 cal.T_i_c.emplace_back(Ric,V(.1,0,0));
 cal.imu_update_rate=200;cal.gyro_noise_std.setConstant(.001);cal.accel_noise_std.setConstant(.02);
 cal.gyro_bias_std.setConstant(.0001);cal.accel_bias_std.setConstant(.001);
 auto vio=basalt::VioEstimatorFactory::getVioEstimator(cfg,cal,V(0,0,-G),true,true);
 tbb::concurrent_bounded_queue<basalt::PoseVelBiasState<double>::Ptr> output;
 vio->out_state_queue=&output;
 std::ofstream os(argv[1]);os<<"time,x,y,z,qx,qy,qz,qw,vx,vy,vz,bgx,bgy,bgz,bax,bay,baz,gtx,gty,gtz,gtqx,gtqy,gtqz,gtqw\n";
 std::thread reader([&]{for(;;){basalt::PoseVelBiasState<double>::Ptr s;output.pop(s);if(!s)break;auto p=s->T_w_i.translation();auto q=s->T_w_i.unit_quaternion();auto gt=pos(s->t_ns*1e-9-1.);Eigen::Quaterniond gtq(rot(s->t_ns*1e-9-1.));os<<s->t_ns*1e-9;for(double v:{p.x(),p.y(),p.z(),q.x(),q.y(),q.z(),q.w(),s->vel_w_i.x(),s->vel_w_i.y(),s->vel_w_i.z(),s->bias_gyro.x(),s->bias_gyro.y(),s->bias_gyro.z(),s->bias_accel.x(),s->bias_accel.y(),s->bias_accel.z(),gt.x(),gt.y(),gt.z(),gtq.x(),gtq.y(),gtq.z(),gtq.w()})os<<','<<v;os<<'\n';}});
 basalt::OpticalFlowBase::Ptr flow; if(images){flow=basalt::OpticalFlowFactory::getOpticalFlow(cfg,cal);flow->output_queue=&vio->vision_data_queue;}
 vio->initialize(V::Zero(),V::Zero());
 std::thread inertial([&]{std::mt19937 rng(7);std::normal_distribution<double> n;for(int i=0;i<=2420;i++){double t=i*.005;auto d=std::make_shared<basalt::ImuData<double>>();d->t_ns=1000000000LL+i*5000000LL;d->gyro=omega(t)+V(.003,-.002,.004);d->accel=rot(t).transpose()*(acc(t)+V(0,0,G))+V(.025,-.015,.01);for(int k=0;k<3;k++){d->gyro[k]+=.001*n(rng);d->accel[k]+=.02*n(rng);}vio->imu_data_queue.push(d);}vio->imu_data_queue.push(nullptr);});
 std::mt19937 rng(4);std::uniform_real_distribution<double> x(3,10),y(-4,4),z(-3,3);std::normal_distribution<double> noise(0,.3);std::vector<V> pts;for(int k=0;k<400;k++)pts.emplace_back(x(rng),y(rng),z(rng));
 struct Tile{V center;cv::Mat texture;};std::vector<Tile> tiles;cv::RNG pixels(5432);for(int iy=-5;iy<=5;iy++)for(int iz=-4;iz<=4;iz++){cv::Mat texture(48,48,CV_8UC1);pixels.fill(texture,cv::RNG::UNIFORM,20,235);cv::GaussianBlur(texture,texture,cv::Size(3,3),.6);tiles.push_back({V(x(rng),iy*.65,iz*.65),texture});}std::sort(tiles.begin(),tiles.end(),[](auto&a,auto&b){return a.center.x()>b.center.x();});
 for(int frame=0;frame<=int(12*camera_hz);frame++){double t=frame/camera_hz;auto f=std::make_shared<basalt::OpticalFlowResult>();f->t_ns=1000000000LL+int64_t(std::llround(t*1e9));f->observations.resize(1);f->pyramid_levels.resize(1);auto T=Sophus::SE3d(rot(t),pos(t))*cal.T_i_c[0];for(size_t k=0;k<pts.size();k++){V pc=T.inverse()*pts[k];Eigen::Vector2d uv;if(!cam.project(pc,uv)||uv.x()<8||uv.x()>632||uv.y()<8||uv.y()>472)continue;Eigen::AffineCompact2f affine=Eigen::AffineCompact2f::Identity();affine.translation()=Eigen::Vector2f(uv.x()+noise(rng),uv.y()+noise(rng));f->observations[0].emplace(k,affine);f->pyramid_levels[0][k]=0;}if(!images){vio->vision_data_queue.push(f);continue;}
 cv::Mat gray(480,640,CV_8UC1,cv::Scalar(10));for(auto& tile:tiles){std::vector<cv::Point2f> dst,src{{0,0},{47,0},{47,47},{0,47}};for(auto yz:std::vector<Eigen::Vector2d>{{-.27,-.27},{.27,-.27},{.27,.27},{-.27,.27}}){V pc=T.inverse()*(tile.center+V(0,yz.x(),yz.y()));Eigen::Vector2d uv;if(!cam.project(pc,uv))break;dst.emplace_back(uv.x(),uv.y());}if(dst.size()!=4)continue;auto H=cv::getPerspectiveTransform(src,dst);cv::Mat warped,mask;cv::warpPerspective(tile.texture,warped,H,gray.size());cv::warpPerspective(cv::Mat(48,48,CV_8UC1,cv::Scalar(255)),mask,H,gray.size(),cv::INTER_NEAREST);warped.copyTo(gray,mask);}
 auto input=std::make_shared<basalt::OpticalFlowInput>();input->t_ns=f->t_ns;input->img_data.resize(1);auto managed=std::make_shared<basalt::ManagedImage<uint16_t>>(640,480);for(int yy=0;yy<480;yy++)for(int xx=0;xx<640;xx++)(*managed)(xx,yy)=uint16_t(gray.at<uint8_t>(yy,xx))<<8;input->img_data[0].img=managed;flow->input_queue.push(input);if(frame==0)cv::imwrite(std::string(argv[1])+".png",gray);
 }
 if(images)flow->input_queue.push(nullptr);else vio->vision_data_queue.push(nullptr);vio->maybe_join();vio->drain_input_queues();inertial.join();reader.join();return 0;
}
