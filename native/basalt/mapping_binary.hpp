// SPDX-License-Identifier: Apache-2.0
#pragma once
#include <nlohmann/json.hpp>
#include <cmath>
#include <cstring>
#include <stdexcept>

namespace posim_slam::mapping_binary {
using Json=nlohmann::json;
inline Json doubles(const double* values,size_t count) {
  std::vector<uint8_t> bytes(8*count);
  for(size_t i=0;i<count;++i) {
    uint64_t bits;std::memcpy(&bits,values+i,8);
    for(int j=0;j<8;++j)bytes[8*i+j]=uint8_t(bits>>(8*j));
  }
  return Json::binary(std::move(bytes));
}
inline std::vector<double> doubles(const Json& value,size_t count,bool finite=true) {
  if(!value.is_binary() || value.get_binary().size()!=8*count)throw std::invalid_argument("mapping_graph_binary_shape");
  const auto& bytes=value.get_binary();std::vector<double> out(count);
  for(size_t i=0;i<count;++i) {
    uint64_t bits=0;for(int j=0;j<8;++j)bits|=uint64_t(bytes[8*i+j])<<(8*j);
    std::memcpy(out.data()+i,&bits,8);
    if(finite && !std::isfinite(out[i]))throw std::invalid_argument("mapping_graph_nonfinite_input");
  }
  return out;
}
inline int64_t integer(const Json& value,int64_t low,int64_t high) {
  if(!value.is_number_integer())throw std::invalid_argument("mapping_graph_integer");
  const auto number=value.get<int64_t>();
  if(number<low || number>high)throw std::invalid_argument("mapping_graph_integer");return number;
}
} // namespace posim_slam::mapping_binary
