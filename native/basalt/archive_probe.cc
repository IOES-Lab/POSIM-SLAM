// SPDX-License-Identifier: Apache-2.0
#include "landmark_archive.hpp"
#include <cassert>
#include <limits>

int main() {
  using Point=LandmarkArchive::Point;
  LandmarkArchive archive(3);
  const std::vector<int> ids{1,2};
  const std::vector<Point> points{{1,0,0},{2,0,0}};
  archive.update(1,ids,points,{1,2});archive.update(2,ids,points,{1,2});
  assert(archive.points().empty());
  archive.update(3,ids,points,{1});
  assert(archive.points().size()==1); // Unconfirmed second track is excluded.
  archive.update(4,{1},{{1.5,0,0}},{1});
  assert(archive.retained()==1 && archive.points()[0][0]==1.5);
  archive.update(5,{}, {}, {});
  assert(archive.points().size()==1); // Mature marginalized point survives.
  archive.update(6,{3,4},{{3,0,0},{4,0,0}},{3,4});
  archive.update(7,{3,4},{{3,0,0},{4,0,0}},{3,4});
  archive.update(8,{3,4},{{3,0,0},{4,0,0}},{3,4});
  assert(archive.points().size()==3);
  archive.update(9,{5},{{5,0,0}},{5});
  assert(archive.retained()==3 && archive.evicted()==1);
  auto rejected=[&](int64_t stamp,std::vector<int> keys,std::vector<Point> coordinates) {
    bool threw=false;try {archive.update(stamp,keys,coordinates,{});}catch(const std::runtime_error&) {threw=true;}
    assert(threw && archive.retained()==3 && archive.evicted()==1);
  };
  rejected(9,{},{});rejected(10,{6,6},{{0,0,0},{0,0,0}});
  rejected(10,{6},{{std::numeric_limits<float>::quiet_NaN(),0,0}});
  rejected(10,{-1},{{0,0,0}});rejected(10,{6},{});
  std::vector<Point> large;
  for(int i=0;i<1001;++i)large.push_back({float(i),0,0});
  const auto sample=LandmarkArchive::display_sample(large);
  assert(sample.size()==1000 && sample.front()[0]==0 && sample.back()[0]==1000);
  for(size_t i=1;i<sample.size();++i)assert(sample[i][0]>sample[i-1][0]);
  assert(LandmarkArchive::display_sample({}).empty());
  assert(LandmarkArchive::display_sample({{1,2,3}}).size()==1);
  LandmarkArchive next_segment;next_segment.update(1,{1},{{99,0,0}},{1});
  assert(next_segment.points().empty());
}
