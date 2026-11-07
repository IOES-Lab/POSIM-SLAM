// SPDX-License-Identifier: Apache-2.0
#pragma once
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <utility>

namespace posim_slam {
// Own the original frontend result until its single chronological handoff.
// Bounds match the existing startup-image protocol, independently of the VIO
// state window. No observation is regenerated or silently evicted.
template<class Result> class StartupHistory {
 public:
  void retain(int64_t stamp,std::shared_ptr<Result> result) {
    std::lock_guard<std::mutex> lock(mutex_);
    if(!result || stamp<=0 || consuming_ || results_.size()>=32 ||
        (last_ && (stamp<=last_ || stamp-last_>500'000'000LL || stamp-first_>10'000'000'000LL)))
      throw std::invalid_argument("startup_history_retention_contract");
    if(!first_)first_=stamp;
    last_=stamp;results_.emplace_back(stamp,std::move(result));
  }

  std::shared_ptr<Result> take(int64_t stamp) {
    std::lock_guard<std::mutex> lock(mutex_);
    if(results_.empty() || results_.front().first!=stamp)
      throw std::invalid_argument("startup_history_handoff_order");
    consuming_=true;auto result=std::move(results_.front().second);results_.pop_front();return result;
  }

  bool pending() const {
    std::lock_guard<std::mutex> lock(mutex_);return !results_.empty();
  }

 private:
  mutable std::mutex mutex_;
  std::deque<std::pair<int64_t,std::shared_ptr<Result>>> results_;
  int64_t first_=0,last_=0;
  bool consuming_=false;
};
}  // namespace posim_slam
