// SPDX-License-Identifier: Apache-2.0
#include "sonar_geometry.hpp"
#include <cassert>
#include <iostream>

int main() {
  using V=Eigen::Vector3d;using M=Eigen::Matrix3d;
  V position(.2,-.1,.3),lever(.24,0,-.1);
  M r=Eigen::AngleAxisd(.2,V(1,2,3).normalized()).toRotationMatrix();
  M mount=Eigen::AngleAxisd(.35,V::UnitY()).toRotationMatrix();
  V local(4.,.7,.3),point=position+r*(lever+mount*local);
  auto evaluate=[&](V p,M rotation,V landmark) {return posim_slam::sonarPointRows(p,rotation,lever,mount,landmark,4.,.1,.2,10.,2.1,.35);};
  const auto rows=evaluate(position,r,point);
  Eigen::Matrix<double,2,9> numeric,analytic;
  analytic<<rows.pose_jacobian,rows.point_jacobian;
  for(int k=0;k<9;++k) {
    Eigen::Vector2d values[2];
    for(int i=0;i<2;++i) {
      const double d=(i?1.:-1.)*1e-6;V p=position,x=point;M rotation=r;
      if(k<3)p[k]+=d;
      else if(k<6)rotation=Eigen::AngleAxisd(d,V::Unit(k-3)).toRotationMatrix()*r;
      else x[k-6]+=d;
      values[i]=evaluate(p,rotation,x).residual;
    }
    numeric.col(k)=(values[1]-values[0])/2e-6;
  }
  assert((numeric-analytic).cwiseAbs().maxCoeff()<1e-8);
  auto exact=posim_slam::sonarPointRows(position,r,lever,mount,point,local.norm(),std::atan2(local.y(),local.x()),.2,10.,2.1,.35);
  assert(exact.residual.norm()<1e-12);
  const double fov=2.0943951023931953;
  const float endpoint=float(fov/2);
  const float dx=std::cos(endpoint),dy=std::sin(endpoint);
  const double measured=std::atan2(double(dy),double(dx));
  assert(measured>fov/2 && posim_slam::sonarWithinFov(measured,fov));
  assert(!posim_slam::sonarWithinFov(fov/2+1e-5,fov));
  const auto boundary=posim_slam::sonarPointRows(V::Zero(),M::Identity(),V::Zero(),M::Identity(),
    V(4*std::cos(measured),4*std::sin(measured),0),4.,measured,.2,10.,fov,.35);
  assert(boundary.residual.norm()<1e-12);
  bool rejected=false;try{evaluate(position,r,position+r*(lever+mount*V(4,0,3)));}
  catch(const std::invalid_argument&){rejected=true;}assert(rejected);
  std::cout<<"PASS native sonar pose/point derivatives, exact residual and FOV rejection\n";
}
