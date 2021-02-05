//
// Created by vahagn on 03/02/21.
//
#include <random>
#include <unordered_set>

#include <geometry/two_view_reconstructor.h>

namespace orb_slam3 {
namespace geometry {

TwoViewReconstructor::TwoViewReconstructor(const std::shared_ptr<camera::MonocularCamera> & left,
                                           const std::shared_ptr<camera::MonocularCamera> & right,
                                           const unsigned number_of_ransac_iterations)
    : left_(left), right_(right), number_of_ransac_iterations_(number_of_ransac_iterations) {

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
  precision_t error;
  TMatrix33 homography;
  FindBestHomographyMatrix(kp1, kp2, pre_matches, random_match_subset_idx, homography, error);

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