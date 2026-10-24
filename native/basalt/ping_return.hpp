// SPDX-License-Identifier: Apache-2.0
#pragma once
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>

namespace posim_slam {
struct PingReturnMoments {
  double mean,variance;
  // Derivative of expected return with respect to each noiseless ray range.
  std::vector<double> winner_probability;
};
// Independent, equal-sigma Gaussian ray noise, followed by minimum selection.
// Excludes clipping, missing returns and shared/dynamic bias uncertainty.
inline PingReturnMoments pingReturnMoments(const std::vector<double>& ranges,
    double sigma,double bias,double minimum,double maximum) {
  if(ranges.empty() || ranges.size()>256 || !std::isfinite(sigma) || sigma<=0 ||
     !std::isfinite(bias) || !std::isfinite(minimum) || !std::isfinite(maximum) ||
     minimum<0 || minimum>=maximum)
    throw std::invalid_argument("invalid_ping_return_model");
  for(double r:ranges)if(!std::isfinite(r) || r<=minimum || r>=maximum || r+bias-10*sigma<=minimum || r+bias+10*sigma>=maximum)
    throw std::invalid_argument("ping_return_clipping_not_supported");
  const double base=*std::min_element(ranges.begin(),ranges.end());
  PingReturnMoments out{0.,0.,std::vector<double>(ranges.size(),0.)};
  std::vector<double> log_survival(ranges.size()),z(ranges.size());
  double mass=0.,first=0.,second=0.;
  constexpr int steps=512;constexpr double h=20./steps;
  const double log_normalizer=.5*std::log(2*std::acos(-1.));
  for(int k=0;k<=steps;++k) {
    const double t=-10.+h*k,weight=h/3.*((k==0 || k==steps)?1:(k%2?4:2));
    double total=0.;
    for(size_t i=0;i<ranges.size();++i) {
      z[i]=t-(ranges[i]-base)/sigma;
      log_survival[i]=std::log(.5*std::erfc(z[i]/std::sqrt(2.)));
      total+=log_survival[i];
    }
    double density=0.;
    for(size_t i=0;i<ranges.size();++i) {
      const double contribution=std::exp(-.5*z[i]*z[i]-log_normalizer+total-log_survival[i]);
      out.winner_probability[i]+=weight*contribution;density+=contribution;
    }
    mass+=weight*density;first+=weight*density*t;second+=weight*density*t*t;
  }
  if(!std::isfinite(mass) || std::abs(mass-1.)>1e-8)
    throw std::runtime_error("ping_return_quadrature_failed");
  first/=mass;second/=mass;
  out.mean=base+bias+sigma*first;out.variance=sigma*sigma*(second-first*first);
  for(double& p:out.winner_probability)p/=mass;
  if(!std::isfinite(out.mean) || !std::isfinite(out.variance) || out.variance<=0)
    throw std::runtime_error("invalid_ping_return_moments");
  return out;
}
}
