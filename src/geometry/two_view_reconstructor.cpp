//
// Created by vahagn on 03/02/21.
//

// == stl ===
#include <cassert>
#include <random>
#include <unordered_set>

// == orb-slam3 ===
#include "two_view_reconstructor.h"
#include "utils.h"
#include "random_subset_generator.h"

namespace orb_slam3 {
namespace geometry {

TwoViewReconstructor::TwoViewReconstructor(const unsigned number_of_ransac_iterations,
                                           precision_t sigma_threshold)
    : number_of_ransac_iterations_(number_of_ransac_iterations),
      essential_matrix_estimator_(sigma_threshold),
      homography_matrix_estimator_(sigma_threshold) {
}

bool TwoViewReconstructor::Reconstruct(const features::Features & features_to,
                                       const features::Features & features_from,
                                       const std::unordered_map<std::size_t, std::size_t> & matches,
                                       Pose & out_pose,
                                       std::unordered_map<std::size_t, TPoint3D> & out_points) const {

  std::vector<std::vector<size_t>> random_match_subset_idx;
  // TODO: check count consistency
  GenerateRandomSubsets(0, matches.size(), 12, number_of_ransac_iterations_, matches, random_match_subset_idx);

  precision_t h_score;
  precision_t f_score;
  TMatrix33 homography, essential;
  std::unordered_set<std::size_t> homography_inliers, essential_inliers;

  // TODO: do this in parallel
  essential_matrix_estimator_.FindBestEssentialMatrix(features_to.undistorted_and_unprojected_keypoints,
                                                      features_from.undistorted_and_unprojected_keypoints,
                                                      matches,
                                                      random_match_subset_idx,
                                                      essential,
                                                      essential_inliers,
                                                      f_score);

  homography_matrix_estimator_.FindBestHomographyMatrix(features_to.undistorted_and_unprojected_keypoints,
                                                        features_from.undistorted_and_unprojected_keypoints,
                                                        matches,
                                                        random_match_subset_idx,
                                                        homography,
                                                        homography_inliers,
                                                        h_score);

  if (h_score > f_score) {
    std::cout << "Homography" << std::endl;
    return homography_matrix_estimator_.FindPose(homography,
                                                 features_to,
                                                 features_from,
                                                 matches,
                                                 out_points,
                                                 out_pose);
  }
  std::cout << "Essential" << std::endl;
  return essential_matrix_estimator_.FindPose(essential,
                                              features_to,
                                              features_from,
                                              matches,
                                              out_points,
                                              out_pose);
}

void TwoViewReconstructor::GenerateRandomSubsets(const size_t min,
                                                 const size_t max,
                                                 const size_t count,
                                                 size_t subset_count,
                                                 const std::unordered_map<std::size_t, std::size_t> & matches,
                                                 std::vector<std::vector<size_t>> & out_result) {

  out_result.resize(subset_count);
  RandomSubsetGenerator generator(count, min, max);
  while (subset_count--) {
//    if(!utils::GenerateRandomSubset(min, max, count, out_result[subset_count]))
//      return;
    generator.Generate(out_result[subset_count]);
    std::sort(out_result[subset_count].begin(), out_result[subset_count].end());
    auto b = matches.begin();
    size_t prev = 0;
    for (size_t & j : out_result[subset_count]) {
      assert(matches.size() > j);
      std::advance(b, j - prev);
      prev = j;
      j = b->first;
    }
  }
}

}
}
