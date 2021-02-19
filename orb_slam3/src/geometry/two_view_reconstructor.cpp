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
                                           const unsigned number_of_ransac_iterations,
                                           precision_t sigma_threshold)
    : left_(left),
      right_(right),
      number_of_ransac_iterations_(number_of_ransac_iterations),
      fundamental_matrix_estimator_(sigma_threshold),
      homography_matrix_sstimator_(sigma_threshold) {
}

bool TwoViewReconstructor::Reconstruct(const std::vector<HomogenousPoint> & points_to,
                                       const std::vector<HomogenousPoint> & points_from,
                                       const std::vector<int> & matches12,
                                       Pose & out_pose,
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
  std::vector<bool> homography_inliers, fundamental_inliers;
  fundamental_matrix_estimator_.FindBestFundamentalMatrix(points_to,
                                                          points_from,
                                                          pre_matches,
                                                          random_match_subset_idx,
                                                          fundamental,
                                                          fundamental_inliers,
                                                          f_error);
  homography_matrix_sstimator_.FindBestHomographyMatrix(points_to,
                                                        points_from,
                                                        pre_matches,
                                                        random_match_subset_idx,
                                                        homography,
                                                        homography_inliers,
                                                        h_error);
  if (true) {
    return homography_matrix_sstimator_.FindRTTransformation(homography,
                                                             points_to,
                                                             points_from,
                                                             pre_matches,
                                                             out_outliers,
                                                             out_points,
                                                             out_pose);
  } else {
    return false;
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
  std::uniform_int_distribution<size_t> distr(min, max - 1);
  std::unordered_set<size_t> chosen;
  out_result.reserve(count);
  while (chosen.size() < count)
    chosen.insert(distr(generator));
  std::copy(chosen.begin(), chosen.end(), std::back_inserter(out_result));
}

}
}