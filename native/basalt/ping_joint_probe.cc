// SPDX-License-Identifier: Apache-2.0
// Text protocol for cross-language qualification of native grouped elimination.
#include "ping_joint.hpp"
#include <iostream>
#include <iomanip>

int main(int argc,char** argv) {
  try {
    Eigen::Index rows,cols,retained;
    if (!(std::cin>>rows>>cols>>retained) || rows<1 || rows>4096 || cols<2 || cols>posim_slam::kMaxJointColumns)
      return 2;
    Eigen::MatrixXd j(rows,cols);Eigen::VectorXd r(rows);
    for (Eigen::Index row=0;row<rows;++row) {
      for (Eigen::Index col=0;col<cols;++col)if (!(std::cin>>j(row,col)))return 2;
      if (!(std::cin>>r[row]))return 2;
    }
    if(argc>2)return 2;
    const double damping=argc==2 ? std::stod(argv[1]) : 0.;
    auto reduced=posim_slam::eliminateJointPatch(j,r,retained,damping);
    std::cout<<std::setprecision(17)<<reduced.nuisance_rank<<' '<<reduced.nuisance_nullity<<'\n';
    // Compare normal equations, not arbitrary signs of singular vectors.
    std::cout<<reduced.jacobian.transpose()*reduced.jacobian<<'\n';
    std::cout<<(reduced.jacobian.transpose()*reduced.residual).transpose()<<'\n';
    std::cout<<reduced.residual.squaredNorm()<<'\n';
    std::cout<<reduced.backsub_matrix<<'\n'<<reduced.backsub_offset.transpose()<<'\n';
    return 0;
  } catch(const std::exception& error) {std::cerr<<error.what()<<'\n';return 2;}
}
