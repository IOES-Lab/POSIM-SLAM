// SPDX-License-Identifier: Apache-2.0
#include <basalt/optical_flow/patch.h>
#include <basalt/image/image.h>
#include <nlohmann/json.hpp>
#include <Eigen/Eigenvalues>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>
#include <string>
using Json=nlohmann::json;
using Patch=basalt::OpticalFlowPatch<float,basalt::Pattern51<float>>;
template<class Matrix> Json rows(const Matrix& m) {
 Json out=Json::array();for(int i=0;i<m.rows();++i){Json row=Json::array();for(int j=0;j<m.cols();++j)row.push_back(m(i,j));out.push_back(row);}return out;
}
int main() {
 try {
  std::string line;
  while(std::getline(std::cin,line)) {
   const auto input=Json::parse(line);int width=input.at("width"),height=input.at("height");
   if(width<32 || height<32 || width>1920 || height>1080 || input.at("points").size()>4096)throw std::runtime_error("input_budget");
   cv::Mat raw(height,width,CV_8UC1);
   for(int y=0;y<height;++y)std::cin.read(reinterpret_cast<char*>(raw.ptr<uint8_t>(y)),width);
   if(!std::cin)throw std::runtime_error("pixels_truncated");
   int processed_width=input.value("processed_width",width),processed_height=input.value("processed_height",height);
   if(processed_width<32 || processed_height<32 || processed_width>width || processed_height>height)throw std::runtime_error("resize_budget");
   cv::Mat resized;
   cv::resize(raw,resized,cv::Size(processed_width,processed_height),0,0,cv::INTER_AREA);
   basalt::ManagedImage<uint16_t> image(processed_width,processed_height);
   for(int y=0;y<processed_height;++y)for(int x=0;x<processed_width;++x)image(x,y)=uint16_t(resized.at<uint8_t>(y,x))<<8;
   Json output=Json::array();
   for(const auto& point:input.at("points")) {
    Eigen::Vector2f pos(point.at("pixel")[0],point.at("pixel")[1]);
    Patch::VectorP data;Patch::MatrixP3 jac;float mean;
    Patch::setDataJacSe2(image,pos,mean,data,jac);
    Json result={{"id",point.at("id")},{"mean_gray8",mean/256.f},{"finite",jac.array().isFinite().all()}};
    if(jac.array().isFinite().all()) {
     Eigen::Matrix3d h=jac.cast<double>().transpose()*jac.cast<double>();
     Eigen::Matrix2d s=h.topLeftCorner<2,2>();
     if(h(2,2)>0)s-=h.topRightCorner<2,1>()*h.bottomLeftCorner<1,2>()/h(2,2);
     s=(s+s.transpose()).eval()*.5;
     Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> eigen(s);
     result["se2_hessian"]=rows(h);result["translation_information_rotation_eliminated"]=rows(s);
     result["translation_eigenvalues"]=rows(eigen.eigenvalues());
     result["weak_translation_direction"]=rows(eigen.eigenvectors().col(0));
     if(eigen.eigenvalues()[1]>0)result["weak_strong_ratio"]=eigen.eigenvalues()[0]/eigen.eigenvalues()[1];
    }
    output.push_back(result);
   }
   std::cout<<output.dump()<<std::endl;
  }
 }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
}
