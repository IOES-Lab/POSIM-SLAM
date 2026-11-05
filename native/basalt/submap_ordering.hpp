// SPDX-License-Identifier: Apache-2.0
#pragma once
#include <cstddef>
#include <cstdint>
#include <utility>
#include <vector>

namespace posim_slam {
// Symbolic block elimination only. Weights count scalar coordinates; retained
// coordinates participate in fill but the caller never selects them to retire.
class SubmapOrderingGraph {
 public:
  explicit SubmapOrderingGraph(std::vector<size_t> weights)
      :weights_(std::move(weights)),words_((weights_.size()+63)/64),
       rows_(weights_.size()*words_),degrees_(weights_.size()) {}
  void connect(size_t a,size_t b) {
    if(a==b || (rows_[a*words_+b/64]&(uint64_t(1)<<(b%64))))return;
    rows_[a*words_+b/64]|=uint64_t(1)<<(b%64);rows_[b*words_+a/64]|=uint64_t(1)<<(a%64);
    degrees_[a]+=weights_[b];degrees_[b]+=weights_[a];
  }
  size_t degree(size_t node) const {return degrees_[node];}
  size_t width(size_t node) const {return weights_[node]+degrees_[node];}
  size_t eliminate(size_t node) {
    const size_t columns=width(node);std::vector<size_t> neighbors;
    for(size_t w=0;w<words_;++w)bits(rows_[node*words_+w],[&](size_t b){neighbors.push_back(64*w+b);});
    for(auto other:neighbors) {
      for(size_t w=0;w<words_;++w) {
        const auto incoming=rows_[node*words_+w];auto& row=rows_[other*words_+w];
        auto added=incoming&~row;if(w==other/64)added&=~(uint64_t(1)<<(other%64));
        bits(added,[&](size_t b){degrees_[other]+=weights_[64*w+b];});row|=incoming;
      }
      rows_[other*words_+other/64]&=~(uint64_t(1)<<(other%64));
      rows_[other*words_+node/64]&=~(uint64_t(1)<<(node%64));degrees_[other]-=weights_[node];
    }
    for(size_t w=0;w<words_;++w)rows_[node*words_+w]=0;
    degrees_[node]=0;return columns;
  }
 private:
  template<class F> static void bits(uint64_t value,F visit) {
    while(value) {
      uint64_t low=value;size_t bit=0;
      if(!(low&0xffffffffu)){bit+=32;low>>=32;}
      if(!(low&0xffffu)){bit+=16;low>>=16;}
      if(!(low&0xffu)){bit+=8;low>>=8;}
      if(!(low&0xfu)){bit+=4;low>>=4;}
      if(!(low&0x3u)){bit+=2;low>>=2;}
      if(!(low&0x1u))++bit;
      visit(bit);value&=value-1;
    }
  }
  std::vector<size_t> weights_;size_t words_;
  std::vector<uint64_t> rows_;std::vector<size_t> degrees_;
};
}
