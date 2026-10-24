// SPDX-License-Identifier: Apache-2.0
#include "ping_geometry.hpp"
#include <basalt/imu/imu_types.h>
#include <nlohmann/json.hpp>
#include <iostream>

int main() {
  try {
    using namespace posim_slam;
    double range_error=0,point_error=0,surface_error=0,expected_error=0;
    Eigen::MatrixXd rays(25,3);int row=0;
    for(int y=-2;y<=2;++y)for(int x=-2;x<=2;++x) {
      const double h=x*.074175,v=y*.074175;
      rays.row(row++)=Vec3(-std::cos(v)*std::sin(h),std::sin(v),std::cos(v)*std::cos(h)).transpose();
    }
    const Sophus::SE3d mount(Sophus::SO3d::exp(Vec3(.03,-.02,.01)),Vec3(.05,-.03,.1));
    const Sophus::SE3d camera(Sophus::SO3d::exp(Vec3(-.02,.04,.01)),Vec3(-.1,.02,.03));
    const Eigen::Vector2d direction(.12,-.07);const double rho=.4,epsilon=1e-6;
    for(const Vec3 angle:{Vec3(.1,-.2,.3),Vec3(3.141592653589793,0,0),Vec3(-.6,.3,-.8)}) {
      Sophus::SE3d pose(Sophus::SO3d::exp(angle),Vec3(.3,-.2,.4));
      const Vec3 n=pose.rotationMatrix()*Vec3(.2,-.13,1.).normalized();
      const double d=n.dot(pose*Vec3(0,0,3.));const auto basis=planeBasis(n);
      const auto range=pingRange(pose,mount,rays,n,d,.1);
      const auto expected=expectedPingRange(pose,mount,rays,n,d,.1,.005,.2,50.);
      Eigen::Matrix<double,1,9> expected_j;expected_j<<expected.range.pose_jacobian,expected.range.plane_jacobian;
      const auto landmark=worldLandmark(pose,camera,direction,rho);
      const auto surface=surfaceRow(landmark,n,d);
      Eigen::Matrix<double,1,9> analytic_range;analytic_range<<range.pose_jacobian,range.plane_jacobian;
      Eigen::Matrix<double,3,9> analytic_point;analytic_point<<landmark.host_jacobian,landmark.landmark_jacobian;
      Eigen::Matrix<double,1,12> analytic_surface;
      analytic_surface<<surface.host_jacobian,surface.landmark_jacobian,surface.plane_jacobian;
      for(int k=0;k<12;++k) {
        double sr[2];Vec3 points[2];double rr[2],er[2];
        for(int side=0;side<2;++side) {
          const double delta=(side?1:-1)*epsilon;
          auto p=pose;auto normal=n;auto dir=direction;double dist=d,inverse=rho;
          if(k<6) {
            Eigen::Matrix<double,6,1> inc=Eigen::Matrix<double,6,1>::Zero();inc[k]=delta;
            basalt::PoseState<double>::incPose(inc,p);
          } else if(k<8)dir[k-6]+=delta;
          else if(k==8)inverse+=delta;
          else if(k<11)normal=Sophus::SO3d::exp(basis.col(k-9)*delta)*n;
          else dist+=delta;
          const auto point=worldLandmark(p,camera,dir,inverse);points[side]=point.point;
          sr[side]=surfaceRow(point,normal,dist).residual;
          if(k<9) {
            auto range_p=pose;auto rn=n;double rd=d;
            if(k<6)range_p=p;
            else if(k<8)rn=Sophus::SO3d::exp(basis.col(k-6)*delta)*n;
            else rd+=delta;
            rr[side]=pingRange(range_p,mount,rays,rn,rd,.1).prediction;
            er[side]=expectedPingRange(range_p,mount,rays,rn,rd,.1,.005,.2,50.).range.prediction;
          }
        }
        surface_error=std::max(surface_error,std::abs((sr[1]-sr[0])/(2*epsilon)-analytic_surface[k]));
        if(k<9) {
          point_error=std::max(point_error,((points[1]-points[0])/(2*epsilon)-analytic_point.col(k)).norm());
          expected_error=std::max(expected_error,std::abs((er[1]-er[0])/(2*epsilon)-expected_j[k]));
          range_error=std::max(range_error,std::abs((rr[1]-rr[0])/(2*epsilon)-analytic_range[k]));
        }
      }
    }
    bool infinity_rejected=false;
    try {worldLandmark(Sophus::SE3d(),camera,direction,0.);}catch(const std::invalid_argument&){infinity_rejected=true;}
    Eigen::MatrixXd tied(2,3);tied<<std::sin(.1),0,std::cos(.1),-std::sin(.1),0,std::cos(.1);
    const auto tied_result=expectedPingRange(Sophus::SE3d(),Sophus::SE3d(),tied,Vec3(0,0,1),3.,0.,.005,.2,50.);
    const bool tied_ok=std::abs(tied_result.winner_probability[0]-.5)<1e-12 && tied_result.range.pose_jacobian.allFinite();
    const bool passed=tied_ok && expected_error<1e-7 && range_error<1e-7 && point_error<1e-7 && surface_error<1e-7 && infinity_rejected;
    std::cout<<nlohmann::json({{"expected_range_jacobian_error",expected_error},{"equal_range_rays_supported",tied_ok},{"passed",passed},{"range_jacobian_error",range_error},
      {"world_landmark_jacobian_error",point_error},{"surface_jacobian_error",surface_error},
      {"infinite_landmark_rejected",infinity_rejected}}).dump()<<'\n';
    return passed?0:1;
  }catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}
}
