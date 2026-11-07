// SPDX-License-Identifier: Apache-2.0
#include "sonar_geometry.hpp"
#include <cassert>
#include <iostream>
#include <limits>
#include <vector>

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
  for(const auto& test:std::vector<Eigen::Vector3d>{{.2,9.999,10.008360862731934},{.51,.510001,.5016722679138184}}) {
    const auto sampled=posim_slam::sonarPointRows(V::Zero(),M::Identity(),V::Zero(),M::Identity(),
      V(test[1],0,0),test[2],0.,test[0],10.,2.1,.35);
    assert(std::abs(sampled.residual[0]-(test[1]-test[2]))<1e-14);
    assert(sampled.pose_jacobian.allFinite() && sampled.point_jacobian.allFinite());
    for(double outside:{test[0]-.001,10.001}) {
      bool unsupported=false;
      try {posim_slam::sonarPointRows(V::Zero(),M::Identity(),V::Zero(),M::Identity(),
        V(outside,0,0),test[2],0.,test[0],10.,2.1,.35);}
      catch(const posim_slam::SonarPointOutsideSupport&){unsupported=true;}
      assert(unsupported);
    }
  }
  for(double invalid:{0.,-1.,std::numeric_limits<double>::infinity(),std::numeric_limits<double>::quiet_NaN()}) {
    bool malformed=false;
    try {posim_slam::sonarPointRows(V::Zero(),M::Identity(),V::Zero(),M::Identity(),V(4,0,0),invalid,0.,.2,10.,2.1,.35);}
    catch(const std::invalid_argument&){malformed=true;}assert(malformed);
  }
  std::cout<<"PASS native sonar pose/point derivatives, exact residual and FOV rejection\n";
}
