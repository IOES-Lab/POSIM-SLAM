// SPDX-License-Identifier: Apache-2.0
#include "startup_history.hpp"
#include <cassert>
#include <iostream>

int main() {
  using History=posim_slam::StartupHistory<int>;
  History history;auto a=std::make_shared<int>(7),b=std::make_shared<int>(8);
  history.retain(1'000'000'000,a);history.retain(1'200'000'000,b);
  int refused=0;
  const auto reject=[&](auto action){try{action();}catch(const std::invalid_argument&){++refused;return;}assert(false);};
  reject([&]{history.take(1'200'000'000);});
  assert(history.take(1'000'000'000)==a && history.pending());
  reject([&]{history.take(1'000'000'000);});
  reject([&]{history.retain(1'400'000'000,std::make_shared<int>(9));});
  assert(history.take(1'200'000'000)==b && !history.pending());
  reject([&]{history.take(1'200'000'000);});
  for(int64_t invalid:{0,-1}) {
    History empty;reject([&]{empty.retain(invalid,a);});
  }
  {History empty;reject([&]{empty.retain(1,nullptr);});}
  {History clock;clock.retain(1'000'000'000,a);
   reject([&]{clock.retain(1'000'000'000,b);});
   reject([&]{clock.retain(999'999'999,b);});
   reject([&]{clock.retain(1'500'000'001,b);});}
  {History capacity;for(int64_t i=0;i<32;++i)capacity.retain(1'000'000'000+i*200'000'000,a);
   reject([&]{capacity.retain(7'400'000'000,b);});
   for(int64_t i=0;i<32;++i)assert(capacity.take(1'000'000'000+i*200'000'000)==a);
   assert(!capacity.pending());}
  {History duration;for(int64_t i=0;i<=20;++i)duration.retain(1'000'000'000+i*500'000'000,a);
   reject([&]{duration.retain(11'000'000'001,b);});}
  assert(refused==12);
  std::cout<<"PASS original frontend identity, chronological single handoff, unchanged startup bounds and 12 refusals\n";
}
