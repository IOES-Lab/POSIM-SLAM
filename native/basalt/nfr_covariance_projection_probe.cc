// SPDX-License-Identifier: Apache-2.0
#include "nfr_covariance_projection.hpp"
#include <Eigen/LU>
#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>

using posim_slam::NfrCovarianceProjection;

template <class Function> void rejects(Function run) {
  bool rejected = false;
  try { run(); } catch (const std::invalid_argument&) { rejected = true; }
  assert(rejected);
}

void close(const Eigen::MatrixXd& actual, const Eigen::MatrixXd& expected) {
  assert(actual.allFinite());
  assert((actual.array() == actual.transpose().array()).all());
  assert((actual - expected).norm() <= 1e-12 * expected.norm());
  assert(Eigen::LLT<Eigen::MatrixXd>(actual).info() == Eigen::Success);
}

int main() {
  // Two overlapping measurements with an analytic correlated 2x2 covariance.
  Eigen::Matrix<double, 2, 3> j;
  j << 1, 1, 0, 0, 1, 1;
  const double a = 1.0 / 4 + 1.0 / 9, b = 1.0 / 9, c = 1.0 / 9 + 1.0 / 25;
  Eigen::Matrix2d expected;
  expected << c, -b, -b, a;
  expected /= a * c - b * b;
  int cases = 0;
  std::array<int, 3> order{0, 1, 2};
  do {
    for (double scale : {0.01, 0.1, 1.0, 10.0, 100.0}) {
      Eigen::Matrix3d transform = Eigen::Matrix3d::Zero();
      transform(order[0], 0) = scale;
      transform(order[1], 1) = 1;
      transform(order[2], 2) = 1 / scale;
      const Eigen::Matrix3d h = transform.transpose() *
          Eigen::Vector3d(4, 9, 25).asDiagonal() * transform;
      close(NfrCovarianceProjection(h).information(j * transform), expected);
      ++cases;
    }
  } while (std::next_permutation(order.begin(), order.end()));

  // Six-dimensional relative factors: inverse of a unit upper bidiagonal J
  // is the finite geometric series, independent of any numerical factorization.
  Eigen::Matrix<double, 6, 6> shift = Eigen::Matrix<double, 6, 6>::Zero();
  for (int i = 0; i != 5; ++i) shift(i, i + 1) = 0.5;
  const Eigen::Matrix<double, 6, 6> identity = Eigen::Matrix<double, 6, 6>::Identity();
  Eigen::Matrix<double, 6, 6> inverse = identity, power = identity;
  for (int i = 1; i != 6; ++i) { power = (-power * shift).eval(); inverse += power; }
  const Eigen::Matrix<double, 6, 6> h =
      (Eigen::Matrix<double, 6, 1>() << 1, 4, 9, 16, 25, 36).finished().asDiagonal();
  close(NfrCovarianceProjection(h).information(identity + shift), inverse.transpose() * h * inverse);
  ++cases;

  rejects([] { NfrCovarianceProjection value(Eigen::MatrixXd::Zero(3, 3)); });
  rejects([] { NfrCovarianceProjection value(-Eigen::Matrix3d::Identity()); });
  rejects([] { NfrCovarianceProjection value(Eigen::MatrixXd::Ones(2, 3)); });
  Eigen::Matrix3d asymmetric = Eigen::Matrix3d::Identity(); asymmetric(0, 1) = 0.1;
  rejects([&] { NfrCovarianceProjection value(asymmetric); });
  Eigen::Matrix3d nonfinite = Eigen::Matrix3d::Identity();
  nonfinite(0, 0) = std::numeric_limits<double>::quiet_NaN();
  rejects([&] { NfrCovarianceProjection value(nonfinite); });
  const NfrCovarianceProjection valid(Eigen::Matrix3d::Identity());
  rejects([&] { valid.information(Eigen::MatrixXd::Zero(2, 3)); });
  rejects([&] { valid.information(Eigen::MatrixXd::Zero(2, 4)); });
  std::cout << cases << " analytic projection cases; 7 invalid-input refusals\n";
}
