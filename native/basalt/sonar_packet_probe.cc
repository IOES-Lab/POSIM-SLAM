// SPDX-License-Identifier: Apache-2.0
#include "sonar_packet.hpp"
#include <nlohmann/json.hpp>
#include <iostream>
#include <iterator>
int main() {
  try {
    std::vector<char> bytes((std::istreambuf_iterator<char>(std::cin)),{});
    const auto packet=posim_slam::decodeSonarSnapshot(bytes);
    nlohmann::json tracks=nlohmann::json::array();
    for(const auto& track:packet.tracks) {
      nlohmann::json observations=nlohmann::json::array(),covariance=nlohmann::json::array();
      for(const auto& value:track.observations)
        observations.push_back({{"stamp_ns",value.stamp},{"range_m",value.range},{"azimuth_rad",value.azimuth}});
      for(Eigen::Index i=0;i<track.covariance.rows();++i) {
        nlohmann::json row=nlohmann::json::array();
        for(Eigen::Index j=0;j<track.covariance.cols();++j)row.push_back(track.covariance(i,j));
        covariance.push_back(row);
      }
      tracks.push_back({{"id",track.id},{"observations",observations},{"covariance",covariance}});
    }
    std::cout<<nlohmann::json({{"stamp_ns",packet.stamp},{"tracks",tracks}}).dump()<<'\n';
  }catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}
}
