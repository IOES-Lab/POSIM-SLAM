// SPDX-License-Identifier: Apache-2.0
#include "mapping_alignment.hpp"
#include <cassert>
#include <fstream>
#include <iostream>
#include <limits>
int main(int argc,char** argv) {
  if(argc==2) {
    nlohmann::json input;std::ifstream file(argv[1]);file>>input;
    Eigen::Matrix4d mapped,local;
    for(int i=0;i<4;++i)for(int j=0;j<4;++j) {
      mapped(i,j)=input.at("mapped").at(i).at(j);local(i,j)=input.at("local").at(i).at(j);
    }
    std::cout<<posim_slam::mappingAlignment(input.at("epoch"),input.at("revision"),input.at("stamp_ns"),mapped,local).dump()<<'\n';return 0;
  }
  Eigen::Isometry3d local=Eigen::Isometry3d::Identity(),corrected=local;
  local.linear()=Eigen::AngleAxisd(.6,Eigen::Vector3d(1,2,3).normalized()).toRotationMatrix();
  local.translation()<<3,-2,1;
  corrected.linear()=Eigen::AngleAxisd(-.2,Eigen::Vector3d::UnitZ()).toRotationMatrix();
  corrected.translation()<<.4,.15,-.01;
  const Eigen::Matrix4d mapped=(corrected*local).matrix();
  auto value=posim_slam::mappingAlignment("epoch",222,180900000000,mapped,local.matrix());
  Eigen::Matrix4d read;
  for(int i=0;i<4;++i)for(int j=0;j<4;++j)read(i,j)=value.at("map_from_local").at(i).at(j);
  assert((read-corrected.matrix()).norm()<1e-12);
  assert((read*local.matrix()-mapped).norm()<1e-12);
  assert(value.at("navigation_ready")==false && value.at("uncertainty_available")==false);
  for(int mode=0;mode<5;++mode) {
    auto bad=local.matrix().eval();
    if(mode==0)bad(0,0)=std::numeric_limits<double>::quiet_NaN();
    if(mode==1)bad(3,0)=1;
    if(mode==2)bad.topLeftCorner<3,3>()=-Eigen::Matrix3d::Identity();
    bool rejected=false;
    try {posim_slam::mappingAlignment(mode==3?"":"epoch",mode==4?0:1,1,mapped,bad);}
    catch(const std::invalid_argument&){rejected=true;}
    assert(rejected);
  }
  std::cout<<value.dump()<<'\n';
}
