// SPDX-License-Identifier: Apache-2.0
#include "mapping_handoff.hpp"
#include <cassert>
#include <iostream>
struct Owner {
 std::map<int64_t,basalt::PoseStateWithLin<double>> poses;
 const auto& getPoseStateWithLin(int64_t stamp)const{return poses.at(stamp);}
};
int main() {
 using namespace posim_slam;
 Owner owner;owner.poses.emplace(10,basalt::PoseStateWithLin<double>(10,Sophus::SE3d()));
 owner.poses.emplace(20,basalt::PoseStateWithLin<double>(20,Sophus::SE3d()));
 basalt::AbsOrderMap order;order.abs_order_map={{10,{0,15}},{20,{15,6}}};order.total_size=21;order.items=2;
 MappingHandoffRequest request;request.keyframes={10,20};request.retired={10};
 for(int64_t stamp:request.keyframes) {
  auto flow=std::make_shared<basalt::OpticalFlowResult>();flow->t_ns=stamp;
  flow->observations.resize(1);flow->observations[0][7]=Eigen::AffineCompact2f::Identity();
  flow->input_images=std::make_shared<basalt::OpticalFlowInput>();flow->input_images->t_ns=stamp;
  flow->input_images->img_data.resize(1);
  flow->input_images->img_data[0].img=std::make_shared<basalt::ManagedImage<uint16_t>>(4,4);
  flow->input_images->img_data[0].img->Fill(100);
  request.images.push_back(flow);
 }
 Eigen::MatrixXd j=Eigen::MatrixXd::Zero(18,22);
 for(int i=0;i<6;++i){j(i,i)=-1;j(i,15+i)=1;}
 j(6,2)=j(6,21)=j(7,17)=j(7,21)=1;
 for(int i=0;i<9;++i)j(8+i,6+i)=.8+.1*i;
 j(17,3)=.3;
 const Eigen::VectorXd r=Eigen::VectorXd::LinSpaced(18,-.2,.3);
 const auto snapshot=prepareMappingHandoff(owner,order,j,r,request);
 assert(!request.committed && snapshot->aom.total_size==12 && snapshot->frame_states.empty());
 Eigen::MatrixXd retained(18,12),nuisance(18,10);
 retained<<j.leftCols(6),j.middleCols(15,6);nuisance<<j.middleCols(6,9),j.rightCols(1);
 const Eigen::MatrixXd hnn=nuisance.transpose()*nuisance,hkn=retained.transpose()*nuisance;
 const Eigen::MatrixXd expected=retained.transpose()*retained-hkn*hnn.inverse()*hkn.transpose();
 const Eigen::VectorXd gradient=retained.transpose()*r-hkn*hnn.inverse()*nuisance.transpose()*r;
 assert((snapshot->abs_H-expected).norm()<1e-12 && (snapshot->abs_b-gradient).norm()<1e-12);
 Eigen::VectorXd gauge=Eigen::VectorXd::Zero(12);gauge[2]=gauge[8]=1;
 assert((snapshot->abs_H*gauge).norm()<1e-12);
 const auto fixed=prepareMappingHandoff(owner,order,j.leftCols(21),r,request);
 assert((fixed->abs_H*gauge).norm()>.5);
 (*request.images[0]->input_images->img_data[0].img)(0,0)=900;
 request.images[0]->observations[0][7].translation().x()=8;
 assert((*snapshot->opt_flow_res[0]->input_images->img_data[0].img)(0,0)==100);
 assert(snapshot->opt_flow_res[0]->observations[0].at(7).translation().x()==0);
 auto batch=request;batch.retired={10,20};
 const auto batched=prepareMappingHandoff(owner,order,j,r,batch);
 assert(batched->kfs_to_marg==batch.retired && (batched->abs_H-snapshot->abs_H).norm()<1e-12);
 request.committed=snapshot;
 auto rejects=[&](const MappingHandoffRequest& q,const Eigen::MatrixXd& rows){
  bool rejected=false;try{prepareMappingHandoff(owner,order,rows,r,q);}catch(const std::exception&){rejected=true;}assert(rejected);
 };
 rejects(request,j);request.committed.reset();
 auto invalid=request;invalid.images[1]=invalid.images[0];rejects(invalid,j);
 invalid=request;invalid.retired={99};rejects(invalid,j);
 auto nonfinite=j;nonfinite(0,0)=std::numeric_limits<double>::quiet_NaN();rejects(request,nonfinite);
 std::cout<<"Mapping handoff: dense Schur, pressure datum, gauge, detached images and invalid-input controls pass.\n";
}

