// SPDX-License-Identifier: Apache-2.0
// Independent inverse-compositional warp-model diagnostic, not an estimator.
#include <basalt/optical_flow/patch.h>
#include <basalt/image/image.h>
#include <nlohmann/json.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <Eigen/SVD>
#include <iostream>
#include <limits>
#include <string>
using Json=nlohmann::json;
using Patch=basalt::OpticalFlowPatch<double,basalt::Pattern51<double>>;
using Matrix=Eigen::Matrix3d;
using Vector=Eigen::VectorXd;
struct Image {
 cv::Mat gray;basalt::Image<const uint16_t> view;
 Image(int w,int h,int pw,int ph) {
  cv::Mat input(h,w,CV_8UC1),resized;
  for(int y=0;y<h;++y)std::cin.read(reinterpret_cast<char*>(input.ptr<uint8_t>(y)),w);
  if(!std::cin)throw std::runtime_error("pixels_truncated");
  cv::resize(input,resized,cv::Size(pw,ph),0,0,cv::INTER_AREA);resized.convertTo(gray,CV_16UC1,256.);
  view=basalt::Image<const uint16_t>(gray.ptr<uint16_t>(),pw,ph,gray.step);
 }
};
struct Model {
 const basalt::Image<const uint16_t>& target;
 std::vector<int> indices;Vector reference;Eigen::MatrixXd jacobian;
 Model(const basalt::Image<const uint16_t>& source,const basalt::Image<const uint16_t>& target,
       const Eigen::Vector2d& birth,const Matrix& initial,bool affine):target(target) {
  const int dimensions=affine?6:3;std::vector<double> values;std::vector<Eigen::RowVectorXd> gradients;
  for(int i=0;i<Patch::PATTERN_SIZE;++i) {
   const Eigen::Vector2d offset=Patch::pattern2.col(i),p=birth+offset;
   const Eigen::Vector2d q=(initial*Eigen::Vector3d(offset[0],offset[1],1)).head<2>();
   if(!source.InBounds(p,2) || !target.InBounds(q,2))continue;
   const auto value=source.interpGrad<double>(p);Eigen::RowVectorXd j(dimensions);
   j[0]=value[1];j[1]=value[2];
   if(affine)j.tail<4>()<<value[1]*offset[0],value[1]*offset[1],value[2]*offset[0],value[2]*offset[1];
   else j[2]=-value[1]*offset[1]+value[2]*offset[0];
   values.push_back(value[0]);gradients.push_back(j);indices.push_back(i);
  }
  reference=Eigen::Map<Vector>(values.data(),values.size());jacobian.resize(values.size(),dimensions);
  if(values.empty())return;
  for(int i=0;i<int(values.size());++i)jacobian.row(i)=gradients[i];
  const double mean=reference.mean();
  const Eigen::RowVectorXd mean_j=jacobian.colwise().mean();
  jacobian=(jacobian/mean-reference*mean_j/(mean*mean)).eval();reference/=mean;
 }
 bool residual(const Matrix& transform,Vector& r)const {
  if(indices.size()<=Patch::PATTERN_SIZE/2 || !reference.array().isFinite().all())return false;
  r.resize(indices.size());
  for(int i=0;i<int(indices.size());++i) {
   const auto offset=Patch::pattern2.col(indices[i]);
   const Eigen::Vector2d p=(transform*Eigen::Vector3d(offset[0],offset[1],1)).head<2>();
   if(!target.InBounds(p,2))return false;
   r[i]=target.interp<double>(p);
  }
  const double mean=r.mean();if(mean<=std::numeric_limits<double>::epsilon())return false;
  r=r/mean-reference;return r.array().isFinite().all();
 }
};
Json fit(const Model& model,const Matrix& initial,bool affine) {
 Matrix transform=initial;Vector residual;
 if(!model.residual(transform,residual) || !model.jacobian.array().isFinite().all())return {{"valid",false}};
 Eigen::JacobiSVD<Eigen::MatrixXd> solver(model.jacobian,Eigen::ComputeThinU|Eigen::ComputeThinV);
 const double before=residual.squaredNorm();double cost=before;int accepted=0;
 for(int iteration=0;iteration<30;++iteration) {
  Vector delta=solver.solve(residual);if(!delta.array().isFinite().all())break;
  bool improved=false;
  for(double rate:{1.,.5,.25,.125,.0625,.03125}) {
   Matrix step=Matrix::Identity();Vector d=delta*rate;
   if(affine){step(0,0)+=d[2];step(0,1)=d[3];step(1,0)=d[4];step(1,1)+=d[5];step(0,2)=d[0];step(1,2)=d[1];}
   else step=Sophus::SE2d::exp(d.head<3>()).matrix();
   if(step.topLeftCorner<2,2>().determinant()<=0)continue;
   Matrix candidate=transform*step.inverse();Vector candidate_residual;
   if(!model.residual(candidate,candidate_residual))continue;
   double candidate_cost=candidate_residual.squaredNorm();
   if(candidate_cost+1e-12<cost){transform=candidate;residual=candidate_residual;cost=candidate_cost;improved=true;++accepted;break;}
  }
  if(!improved || delta.lpNorm<Eigen::Infinity>()<1e-6)break;
 }
 return {{"valid",true},{"samples",model.indices.size()},{"rank",solver.rank()},{"accepted_iterations",accepted},
  {"before_rmse",std::sqrt(before/model.indices.size())},{"after_rmse",std::sqrt(cost/model.indices.size())},
  {"pixel",{transform(0,2),transform(1,2)}},{"linear",{{transform(0,0),transform(0,1)},{transform(1,0),transform(1,1)}}}};
}
int main() {
 try {
  std::string line;
  while(std::getline(std::cin,line)) {
   const auto r=Json::parse(line);int w=r.at("width"),h=r.at("height"),pw=r.value("processed_width",w),ph=r.value("processed_height",h);
   if(w<32 || h<32 || w>1920 || h>1080 || pw<32 || ph<32 || pw>w || ph>h)throw std::runtime_error("image_budget");
   Image source(w,h,pw,ph),target(w,h,pw,ph);Matrix initial=Matrix::Identity();
   initial(0,2)=r.at("pixel")[0];initial(1,2)=r.at("pixel")[1];
   for(int i=0;i<2;++i)for(int j=0;j<2;++j)initial(i,j)=r.at("linear")[i][j];
   Eigen::Vector2d birth(r.at("birth")[0],r.at("birth")[1]);
   Model rigid(source.view,target.view,birth,initial,false),affine(source.view,target.view,birth,initial,true);
   std::cout<<Json({{"id",r.at("id")},{"se2",fit(rigid,initial,false)},{"affine",fit(affine,initial,true)}}).dump()<<std::endl;
  }
 }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
}
