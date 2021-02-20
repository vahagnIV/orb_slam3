//
// Created by vahagn on 08/02/21.
//

// == orb-slam3 ===
#include "geometry/fundamental_matrix_estimator.h"

namespace orb_slam3 {
namespace geometry {

const precision_t FundamentalMatrixEstimator::FUNDAMENTAL_THRESHOLD = 3.841;
const precision_t FundamentalMatrixEstimator::FUNDAMENTAL_THRESHOLD_SCORE = 5.991;

precision_t FundamentalMatrixEstimator::ComputeFundamentalReprojectionError(const TMatrix33 & f,
                                                                            const std::vector<HomogenousPoint> & kp1,
                                                                            const std::vector<HomogenousPoint> & kp2,
                                                                            const std::vector<features::Match> & matches,
                                                                            std::vector<bool> & out_inliers) const {
  precision_t error = 0;
  out_inliers.resize(matches.size(), true);
  std::fill(out_inliers.begin(), out_inliers.end(), true);
  for (size_t i = 0; i < matches.size(); ++i) {
    const auto & match = matches[i];

    const HomogenousPoint point_from = kp2[match.from_idx];
    const HomogenousPoint point_to = kp1[match.to_idx];

    HomogenousPoint f_from = f * point_from;
    HomogenousPoint to_f = point_to.transpose() * f;

    const precision_t err = point_to.dot(f_from);

    const precision_t chi_square1 = err / (f_from[0] * f_from[0] + f_from[1] * f_from[1]) * sigma_squared_inv_;

    const precision_t chi_square2 = err / (to_f[0] * to_f[0] + to_f[1] * to_f[1]) * sigma_squared_inv_;

    if (chi_square1 > FUNDAMENTAL_THRESHOLD || chi_square2 > FUNDAMENTAL_THRESHOLD) {
      out_inliers[i] = false;
      continue;
    }
    error += FUNDAMENTAL_THRESHOLD_SCORE - chi_square1;
    error += FUNDAMENTAL_THRESHOLD_SCORE - chi_square2;
  }
  return error;
}

void FundamentalMatrixEstimator::FindFundamentalMatrix(const std::vector<HomogenousPoint> & kp1,
                                                       const std::vector<HomogenousPoint> & kp2,
                                                       const std::vector<features::Match> & matches,
                                                       const std::vector<size_t> & good_match_random_idx,
                                                       TMatrix33 & out_fundamental) const {
  Eigen::Matrix<precision_t, Eigen::Dynamic, 9> L;
  L.resize(good_match_random_idx.size(), Eigen::NoChange);

  for (size_t i = 0; i < good_match_random_idx.size(); ++i) {
    const auto & X = kp1[matches[good_match_random_idx[i]].to_idx];
    const auto & U = kp2[matches[good_match_random_idx[i]].from_idx];

    L(i, 0) = X[0] * U[0];
    L(i, 1) = X[0] * U[1];
    L(i, 2) = X[0] * U[2];
    L(i, 3) = X[1] * U[0];
    L(i, 4) = X[1] * U[1];
    L(i, 5) = X[1] * U[2];
    L(i, 6) = X[2] * U[0];
    L(i, 7) = X[2] * U[1];
    L(i, 8) = X[2] * U[2];

  }

  Eigen::JacobiSVD<Eigen::Matrix<precision_t, Eigen::Dynamic, 9>> svd(L, Eigen::ComputeFullU | Eigen::ComputeFullV);
  if (!svd.computeV())
    return;

  const Eigen::Matrix<precision_t, 9, 9> & v = svd.matrixV();

//  Eigen::JacobiSVD<Eigen::Matrix<precision_t, Eigen::Dynamic, 9>> svd2(v, Eigen::ComputeFullU | Eigen::ComputeFullV);
  if (!svd.computeV())
    return;
//  auto S = svd2.singularValues();
//  S[2] = 0;

  out_fundamental << v(0, 8), v(1, 8), v(2, 8),
      v(3, 8), v(4, 8), v(5, 8),
      v(6, 8), v(7, 8), v(8, 8);
  out_fundamental /= out_fundamental(2, 2);

}

void FundamentalMatrixEstimator::FindBestFundamentalMatrix(const std::vector<HomogenousPoint> & kp1,
                                                           const std::vector<HomogenousPoint> & kp2,
                                                           const std::vector<features::Match> & matches,
                                                           const std::vector<std::vector<size_t>> & good_match_random_idx,
                                                           TMatrix33 & out_fundamental,
                                                           std::vector<bool> & out_inliers,
                                                           precision_t & out_error) const {
  out_error = std::numeric_limits<precision_t>::max();
  TMatrix33 tmp_fundamental;
  std::vector<bool> tmp_inliers;
  for (size_t i = 0; i < good_match_random_idx.size(); ++i) {
    const std::vector<size_t> & good_matches_rnd = good_match_random_idx[i];
    FindFundamentalMatrix(kp1, kp2, matches, good_matches_rnd, tmp_fundamental);
    precision_t error = ComputeFundamentalReprojectionError(tmp_fundamental, kp1, kp2, matches, tmp_inliers);
    if (error > 0 && out_error > error) {
      out_fundamental = tmp_fundamental;
      out_inliers = tmp_inliers;
      out_error = error;
    }
  }

}

}
}