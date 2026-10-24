// SPDX-License-Identifier: Apache-2.0
#include "ping_return.hpp"
#include <iostream>
#include <numeric>
#include <random>
int main() {
  using namespace posim_slam;
  auto one=pingReturnMoments({3.},.005,.1,.2,50.);
  auto two=pingReturnMoments({3.,3.},.005,.1,.2,50.);
  const double pi=std::acos(-1.);
  bool pass=std::abs(one.mean-3.1)<1e-12 && std::abs(one.variance-.000025)<1e-14 &&
    std::abs(two.mean-(3.1-.005/std::sqrt(pi)))<1e-12 &&
    std::abs(two.variance-.000025*(1-1/pi))<1e-14;
  std::vector<double> r;for(int i=0;i<25;++i)r.push_back(3.+.0004*i);
  auto moments=pingReturnMoments(r,.005,.1,.2,50.);
  double jac_error=0.;
  for(size_t i=0;i<r.size();++i) {
    auto plus=r,minus=r;plus[i]+=1e-6;minus[i]-=1e-6;
    double derivative=(pingReturnMoments(plus,.005,.1,.2,50.).mean-pingReturnMoments(minus,.005,.1,.2,50.).mean)/2e-6;
    jac_error=std::max(jac_error,std::abs(derivative-moments.winner_probability[i]));
  }
  std::mt19937 generator(47);std::normal_distribution<double> noise(0.,.005);
  double sum=0.,squared=0.;constexpr int n=200000;
  for(int k=0;k<n;++k) {
    double v=50.;for(double d:r)v=std::min(v,d+noise(generator));v+=.1;
    const double centered=v-moments.mean;sum+=centered;squared+=centered*centered;
  }
  bool clipped=false;try{pingReturnMoments({.21},.005,0.,.2,50.);}catch(const std::invalid_argument&){clipped=true;}
  bool outside=false;try{pingReturnMoments({.19},.005,.1,.2,50.);}catch(const std::invalid_argument&){outside=true;}
  bool zero_noise=false;try{pingReturnMoments({3.},0.,0.,.2,50.);}catch(const std::invalid_argument&){zero_noise=true;}
  auto many=pingReturnMoments(std::vector<double>(256,3.),.005,0.,.2,50.);
  bool symmetric=std::all_of(many.winner_probability.begin(),many.winner_probability.end(),[](double p){return std::abs(p-1./256)<1e-12;});
  pass=pass && outside && zero_noise && symmetric && jac_error<1e-7 && std::abs(sum/n)<.00003 && std::abs(squared/n/moments.variance-1)<.02 && clipped;
  std::cout<<"{\"passed\":"<<(pass?"true":"false")<<",\"jacobian_max_error\":"<<jac_error
    <<",\"simulation_mean_error_m\":"<<sum/n<<",\"simulation_variance_ratio\":"<<squared/n/moments.variance
    <<",\"minimum_mean_m\":"<<moments.mean<<",\"minimum_variance_m2\":"<<moments.variance<<"}\n";
  return pass?0:1;
}
