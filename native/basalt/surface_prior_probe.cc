// SPDX-License-Identifier: Apache-2.0
#include "surface_prior_probe_io.hpp"
#include <iostream>
using namespace posim_slam::surface_prior_probe;
namespace probe=posim_slam::range_surface_probe;
namespace sp=posim_slam::surface_prior;
int main() {
  try {
    Json input;std::cin>>input;const auto variables=order(input.at("order"));const auto count=sp::layout(variables).count;
    const auto delta=vector(input.at("delta"));const auto epoch=input.at("epoch").get<std::string>();
    const auto operation=input.value("operation",std::string("marginalize"));
    if(operation=="linearize") {
      const auto result=sp::linearize(prior(input.at("previous")),variables,delta,epoch);
      std::cout<<Json({{"jacobian",probe::jsonMatrix(result.jacobian)},{"residual",probe::jsonVector(result.residual)}}).dump()<<'\n';return 0;
    }
    if(operation!="marginalize")throw std::invalid_argument("unknown_surface_prior_operation");
    std::vector<sp::Retain> retain;
    for(const auto& v:input.at("retain")) {
      if(!v.is_array() || v.size()!=3)throw std::invalid_argument("retained_block_shape");
      retain.push_back({kind(v.at(0).get<std::string>()),probe::integer(v.at(1)),probe::integer(v.at(2))});
    }
    std::optional<sp::Prior> previous;
    if(input.contains("previous"))previous=prior(input.at("previous"));
    std::optional<int64_t> cutoff;
    if(input.contains("close_through_ns"))cutoff=probe::integer(input.at("close_through_ns"));
    const auto result=sp::marginalize(variables,delta,rows(input.at("jacobian"),count),vector(input.at("residual")),retain,epoch,
      input.contains("range_keys") ? keys(input.at("range_keys")) : std::vector<sp::RangeKey>{},previous ? &*previous : nullptr,cutoff);
    std::cout<<write(result).dump()<<'\n';return 0;
  } catch(const std::exception& error) {std::cerr<<error.what()<<'\n';return 2;}
}
