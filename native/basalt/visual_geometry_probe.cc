// SPDX-License-Identifier: Apache-2.0
#include "visual_geometry.hpp"
#include <Eigen/Geometry>
#include <cassert>
int main() {
  auto rows=[](double baseline,double sigma) {
    Eigen::MatrixXd j(4,3);
    j<<200./4/sigma,0,200*(-baseline/2)/16/sigma,
       0,200./4/sigma,0,
       200./4/sigma,0,200*(baseline/2)/16/sigma,
       0,200./4/sigma,0;
    return j;
  };
  for(double b:{.001,.01,1.}) {
    const Eigen::Vector3d ray(b/2,0,4);const double r=ray.norm(),f=200,z=4,sigma=.5;
    const double xx=sigma*sigma*z*z/(2*f*f),zz=2*sigma*sigma*std::pow(z,4)/(f*f*b*b);
    const double expected=std::sqrt((b*b/4*xx+z*z*zz)/(r*r))/r;
    const auto value=posim_slam::visualPointSupport(rows(b,sigma),ray,2);
    assert(std::abs(value.relative_range_sigma-expected)<1e-10*(1+expected));
    assert(value.eligible==(3*expected<1));
    const auto rotation=Eigen::AngleAxisd(.7,Eigen::Vector3d(1,2,3).normalized()).toRotationMatrix();
    const auto rotated=posim_slam::visualPointSupport(rows(b,sigma)*rotation.transpose(),rotation*ray,2);
    assert(std::abs(rotated.relative_range_sigma-value.relative_range_sigma)<1e-9*(1+expected));
    const auto scaled=posim_slam::visualPointSupport(rows(b,sigma)/10,ray*10,2);
    assert(std::abs(scaled.relative_range_sigma-value.relative_range_sigma)<1e-10*(1+expected));
  }
  Eigen::MatrixXd zero=rows(0,.5);
  assert(!posim_slam::visualPointSupport(zero,{0,0,4},2).eligible);
  assert(!posim_slam::visualPointSupport(rows(1,.5),{0,0,4},1).eligible);
}
