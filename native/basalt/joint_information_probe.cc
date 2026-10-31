// SPDX-License-Identifier: Apache-2.0
#include "ping_joint.hpp"
#include "surface_prior_probe_io.hpp"
#include <iostream>
int main() {try {
  nlohmann::json input;std::cin>>input;namespace io=posim_slam::surface_prior_probe;
  const auto j=io::rows(input.at("jacobian"),input.at("columns").get<Eigen::Index>());
  const auto result=posim_slam::splitJointInformation(j,io::vector(input.at("residual")),input.at("retained").get<Eigen::Index>());
  const auto& r=result.reduced;
  std::cout<<nlohmann::json({{"jacobian",io::probe::jsonMatrix(r.jacobian)},{"residual",io::probe::jsonVector(r.residual)},
    {"conditional_jacobian",io::probe::jsonMatrix(result.conditional_jacobian)},
    {"conditional_residual",io::probe::jsonVector(result.conditional_residual)},
    {"noise",io::probe::jsonMatrix(result.observable_noise_factor)},{"unbounded",io::probe::jsonMatrix(result.unobservable_basis)},
    {"regression",io::probe::jsonMatrix(r.backsub_matrix)},{"offset",io::probe::jsonVector(r.backsub_offset)},
    {"rank",r.nuisance_rank},{"nullity",r.nuisance_nullity}}).dump()<<'\n';return 0;
}catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 2;}}
