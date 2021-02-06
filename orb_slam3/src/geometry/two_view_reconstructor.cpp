//
// Created by vahagn on 03/02/21.
//
#include <random>
#include <unordered_set>

#include <geometry/two_view_reconstructor.h>

namespace orb_slam3 {
namespace geometry {

const precision_t TwoViewReconstructor::THRESHOLD = 3.841;
const precision_t TwoViewReconstructor::THRESHOLD_SCORE = 5.991;

TwoViewReconstructor::TwoViewReconstructor(const std::shared_ptr<camera::MonocularCamera> & left,
                                           const std::shared_ptr<camera::MonocularCamera> & right,
                                           const unsigned number_of_ransac_iterations,
                                           precision_t sigma_threshold)
    : left_(left),
      right_(right),
      number_of_ransac_iterations_(number_of_ransac_iterations),
      sigma_threshold_(sigma_threshold),
      sigma_squared_inv_(1 / sigma_threshold / sigma_threshold) {

}

void TwoViewReconstructor::Reconstruct(const std::vector<features::KeyPoint> & kp1,
                                       const std::vector<features::KeyPoint> & kp2,
                                       const std::vector<int> & matches12,
                                       TPose & out_pose,
                                       std::vector<TPoint3D> & out_points,
                                       std::vector<bool> & out_outliers,
                                       const size_t number_of_matches) const {
  std::vector<std::pair<size_t, size_t>> pre_matches;
  out_outliers.resize(matches12.size(), false);
  FilterGoodMatches(matches12, number_of_matches, pre_matches);
  std::vector<std::vector<size_t>> random_match_subset_idx;
  GenerateRandomSubsets(0, pre_matches.size(), 8, number_of_ransac_iterations_, random_match_subset_idx);
  precision_t h_error;
  precision_t f_error;
  TMatrix33 homography, fundamental;
  FindBestFundamentalMatrix(kp1, kp2, pre_matches, random_match_subset_idx, fundamental, f_error);
  FindBestHomographyMatrix(kp1, kp2, pre_matches, random_match_subset_idx, homography, h_error);

}

void TwoViewReconstructor::FindBestHomographyMatrix(const std::vector<features::KeyPoint> & kp1,
                                                    const std::vector<features::KeyPoint> & kp2,
                                                    const std::vector<std::pair<size_t, size_t>> & good_matches,
                                                    const std::vector<std::vector<size_t>> & good_match_random_idx,
                                                    TMatrix33 & out_homography,
                                                    precision_t & out_error) const {
  out_error = std::numeric_limits<precision_t>::max();
  TMatrix33 tmp_homography;
  for (const std::vector<size_t> & good_matches_rnd: good_match_random_idx) {
    FindHomographyMatrix(kp1, kp2, good_matches, good_matches_rnd, tmp_homography);
    precision_t error = ComputeHomographyReprojectionError(tmp_homography, kp1, kp2, good_matches);
    if (out_error > error) {
      out_homography = tmp_homography;
      out_error = error;
    }
  }
}

precision_t TwoViewReconstructor::ComputeHomographyReprojectionError(const TMatrix33 & h,
                                                                     const std::vector<features::KeyPoint> & kp1,
                                                                     const std::vector<features::KeyPoint> & kp2,
                                                                     const std::vector<std::pair<size_t,
                                                                                                 size_t>> & good_matches) const {
  precision_t error = 0;
  for (const auto & match: good_matches) {
    const TPoint2D & point_from = kp2[match.second].pt;
    const TPoint2D & point_to = kp1[match.second].pt;
    precision_t new_z = h(2, 0) * point_from[0] + h(2, 1) * point_from[1] + h(2, 2);
    TPoint2D reprojected21;
    reprojected21 << (h(0, 0) * point_from[0] + h(0, 1) * point_from[1] + h(0, 2)) / new_z,
        (h(1, 0) * point_from[0] + h(1, 1) * point_from[1] + h(1, 2)) / new_z;
    error += (reprojected21 - point_to).squaredNorm();
  }
  return error / good_matches.size();

}

void TwoViewReconstructor::FindHomographyMatrix(const std::vector<features::KeyPoint> & kp1,
                                                const std::vector<features::KeyPoint> & kp2,
                                                const std::vector<std::pair<size_t, size_t>> & good_matches,
                                                const std::vector<size_t> & good_match_random_idx,
                                                TMatrix33 & out_homography) const {

  Eigen::Matrix<precision_t, Eigen::Dynamic, 9> L;
  L.resize(good_match_random_idx.size() * 2, Eigen::NoChange);

  for (size_t i = 0; i < good_match_random_idx.size(); ++i) {
    const auto & U = kp1[good_matches[good_match_random_idx[i]].first].pt;
    const auto & X = kp2[good_matches[good_match_random_idx[i]].second].pt;

    L(2 * i, 0) = X[0];
    L(2 * i, 1) = X[1];
    L(2 * i, 2) = 1;
    L(2 * i, 3) = 0;
    L(2 * i, 4) = 0;
    L(2 * i, 5) = 0;
    L(2 * i, 6) = -U[0] * X[0];
    L(2 * i, 7) = -U[0] * X[1];
    L(2 * i, 8) = -U[0];

    L(2 * i + 1, 0) = 0;
    L(2 * i + 1, 1) = 0;
    L(2 * i + 1, 2) = 0;
    L(2 * i + 1, 3) = X[0];
    L(2 * i + 1, 4) = X[1];
    L(2 * i + 1, 5) = 1;
    L(2 * i + 1, 6) = -U[1] * X[0];
    L(2 * i + 1, 7) = -U[1] * X[1];
    L(2 * i + 1, 8) = -U[1];
  }

  Eigen::JacobiSVD<Eigen::Matrix<precision_t, Eigen::Dynamic, 9>> svd(L, Eigen::ComputeFullU | Eigen::ComputeFullV);
  if (!svd.computeV())
    return;

  const Eigen::Matrix<precision_t, 9, 9> & v = svd.matrixV();

  out_homography << v(0, 8), v(1, 8), v(2, 8),
      v(3, 8), v(4, 8), v(5, 8),
      v(6, 8), v(7, 8), v(8, 8);
  out_homography /= out_homography(2, 2);

}

precision_t TwoViewReconstructor::ComputeFundamentalReprojectionError(const TMatrix33 & f,
                                                                      const std::vector<features::KeyPoint> & kp1,
                                                                      const std::vector<features::KeyPoint> & kp2,
                                                                      const std::vector<std::pair<size_t,
                                                                                                  size_t>> & good_matches,
                                                                      std::vector<bool> & inliers) const {
  precision_t error = 0;
  inliers.resize(good_matches.size(), true);
  for (size_t i = 0; i < good_matches.size(); ++i) {
    const auto & match = good_matches[i];
    const TPoint2D & point_from = kp2[match.second].pt;
    const TPoint2D & point_to = kp1[match.second].pt;

    TPoint3D pt_from_3{point_from[0], point_from[1], 1};
    TPoint3D pt_to_3{point_to[0], point_to[1], 1};

    TPoint3D F_pt_from = f * pt_from_3;
    TPoint3D pt_to_F = pt_to_3.transpose() * f;

    const precision_t err = pt_to_3.dot(F_pt_from);;

    const precision_t
        chi_square1 = err / (F_pt_from[0] * F_pt_from[0] + F_pt_from[1] * F_pt_from[1]) * sigma_squared_inv_;
    const precision_t chi_square2 = err / (pt_to_F[0] * pt_to_F[0] + pt_to_F[1] * pt_to_F[1]) * sigma_squared_inv_;
    if (chi_square1 > THRESHOLD)
      inliers[i] = false;
    else
      error += THRESHOLD_SCORE - chi_square1;

    if (chi_square2 > THRESHOLD)
      inliers[i] = false;
    else
      error += THRESHOLD_SCORE - chi_square2;
  }
  return error;
}

void TwoViewReconstructor::FindFundamentalMatrix(const std::vector<features::KeyPoint> & kp1,
                                                 const std::vector<features::KeyPoint> & kp2,
                                                 const std::vector<std::pair<size_t, size_t>> & good_matches,
                                                 const std::vector<size_t> & good_match_random_idx,
                                                 TMatrix33 & out_fundamental) const {
  Eigen::Matrix<precision_t, Eigen::Dynamic, 9> L;
  L.resize(good_match_random_idx.size(), Eigen::NoChange);

  for (size_t i = 0; i < good_match_random_idx.size(); ++i) {
    const auto & U = kp1[good_matches[good_match_random_idx[i]].first].pt;
    const auto & X = kp2[good_matches[good_match_random_idx[i]].second].pt;

    // u2 = X[0]
    // v2 = X[1]
    ///u1 U[0]
    // v1 U[1]

    L(i, 0) = X[0] * U[0];
    L(i, 1) = X[0] * U[1];
    L(i, 2) = X[0];
    L(i, 3) = X[1] * U[0];
    L(i, 4) = X[1] * U[1];
    L(i, 5) = X[1];
    L(i, 6) = U[0];
    L(i, 7) = U[1];
    L(i, 8) = 1;

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
//  auto m = svd2.matrixU() * S.asDiagonal() * svd2.matrixV();
//  std::cout << m << std::endl;
  out_fundamental << v(0, 8), v(1, 8), v(2, 8),
      v(3, 8), v(4, 8), v(5, 8),
      v(6, 8), v(7, 8), v(8, 8);
  out_fundamental /= out_fundamental(2, 2);

}

void TwoViewReconstructor::FindBestFundamentalMatrix(const std::vector<features::KeyPoint> & kp1,
                                                     const std::vector<features::KeyPoint> & kp2,
                                                     const std::vector<std::pair<size_t, size_t>> & good_matches,
                                                     const std::vector<std::vector<size_t>> & good_match_random_idx,
                                                     TMatrix33 & out_fundamental,
                                                     precision_t & out_error) const {
  out_error = std::numeric_limits<precision_t>::max();
  TMatrix33 tmp_fundamental;
  std::vector<std::vector<bool>> inliers(good_match_random_idx.size());
  for (size_t i = 0; i < good_match_random_idx.size(); ++i) {
    const std::vector<size_t> & good_matches_rnd = good_match_random_idx[i];
    FindFundamentalMatrix(kp1, kp2, good_matches, good_matches_rnd, tmp_fundamental);
    precision_t error = ComputeFundamentalReprojectionError(tmp_fundamental, kp1, kp2, good_matches, inliers[i]);
    if (out_error > error) {
      out_fundamental = tmp_fundamental;
      out_error = error;
    }
  }

}

void TwoViewReconstructor::GenerateRandomSubsets(const size_t min,
                                                 const size_t max,
                                                 const size_t count,
                                                 size_t subset_count,
                                                 std::vector<std::vector<size_t>> & out_result) const {
  out_result.resize(subset_count);
  do { GenerateRandomSubset(min, max, count, out_result[subset_count - 1]); }
  while (--subset_count);
}

void TwoViewReconstructor::FilterGoodMatches(const std::vector<int> & matches12,
                                             const size_t number_of_matches,
                                             std::vector<std::pair<size_t, size_t>> & out_good_matches) const {
  out_good_matches.resize(number_of_matches);
  size_t idx = 0;
  for (size_t i = 0; i < matches12.size(); ++i) {
    if (matches12[i] >= 0) {
      out_good_matches[idx].first = i;
      out_good_matches[idx++].second = static_cast<size_t >(matches12[i]);
    }
  }
}

void TwoViewReconstructor::GenerateRandomSubset(const size_t min,
                                                const size_t max,
                                                const size_t count,
                                                std::vector<size_t> & out_result) const {
  std::random_device rand_dev;
  std::mt19937 generator(rand_dev());
  std::uniform_int_distribution<size_t> distr(min, max);
  std::unordered_set<size_t> chosen;
  out_result.reserve(count);
  while (chosen.size() < count)
    chosen.insert(distr(generator));
  std::copy(chosen.begin(), chosen.end(), std::back_inserter(out_result));
}

}
}