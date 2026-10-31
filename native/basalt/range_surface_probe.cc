// SPDX-License-Identifier: Apache-2.0
// JSON boundary for source/native derivative and ownership controls only.
#include "range_surface_probe_io.hpp"
#include <iostream>
int main() {
  try {
    nlohmann::json json;std::cin>>json;
    const auto input=posim_slam::range_surface_probe::read(json);
    const auto block=posim_slam::surface_ranges::linearize(input.poses,input.planes,input.observations,
      input.covariance,input.uncertainty,input.require_support);
    std::cout<<posim_slam::range_surface_probe::write(block,input.require_support).dump()<<'\n';return 0;
  } catch(const std::exception& error) {std::cerr<<error.what()<<'\n';return 2;}
}
