// SPDX-License-Identifier: Apache-2.0
#pragma once
#include <set>
#include <algorithm>
#include <cstddef>
namespace posim_slam {
// Called by the frontend owner after tracking/filtering all camera views.
template<class Patches,class Views>
std::size_t retain_observed_references(Patches& patches,const Views& views) {
  std::set<typename Patches::key_type> active;
  for(const auto& view:views)for(const auto& item:view)active.insert(item.first);
  const auto before=patches.size();
  for(auto it=patches.begin();it!=patches.end();) {
    if(active.count(it->first))++it;
    else it=patches.erase(it);
  }
  return before-patches.size();
}
}
