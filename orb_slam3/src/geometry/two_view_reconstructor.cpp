//
// Created by vahagn on 03/02/21.
//

// == stl ===
#include <cassert>
#include <random>
#include <unordered_set>

// == orb-slam3 ===
#include <geometry/two_view_reconstructor.h>
#include <features/match.h>

namespace orb_slam3 {
namespace geometry {

TwoViewReconstructor::TwoViewReconstructor(const unsigned number_of_ransac_iterations,
                                           precision_t sigma_threshold)
    : number_of_ransac_iterations_(number_of_ransac_iterations),
      essential_matrix_estimator_(sigma_threshold),
      homography_matrix_estimator_(sigma_threshold) {
}

bool TwoViewReconstructor::Reconstruct(const std::vector<HomogenousPoint> & points_to,
                                       const std::vector<HomogenousPoint> & points_from,
                                       const std::unordered_map<std::size_t, std::size_t> & matches,
                                       Pose & out_pose,
                                       std::unordered_map<std::size_t, TPoint3D> & out_points) const {

  std::vector<std::vector<size_t>> random_match_subset_idx;
  GenerateRandomSubsets(0, matches.size(), 8, number_of_ransac_iterations_, matches, random_match_subset_idx);

  precision_t h_score;
  precision_t f_score;
  TMatrix33 homography, essential;
  std::unordered_set<std::size_t> homography_inliers, essential_inliers;

  // TODO: do this in parallel
  essential_matrix_estimator_.FindBestEssentialMatrix(points_to,
                                                      points_from,
                                                      matches,
                                                      random_match_subset_idx,
                                                      essential,
                                                      essential_inliers,
                                                      f_score);

  homography_matrix_estimator_.FindBestHomographyMatrix(points_to,
                                                        points_from,
                                                        matches,
                                                        random_match_subset_idx,
                                                        homography,
                                                        homography_inliers,
                                                        h_score);

  if (h_score > f_score) {
    std::cout << "Homography: \n" << homography << std::endl;
    return homography_matrix_estimator_.FindPose(homography,
                                                 points_to,
                                                 points_from,
                                                 matches,
                                                 out_points,
                                                 out_pose);
  }
//  std::cout << "Essential: \n" << essential << std::endl;
  return essential_matrix_estimator_.FindPose(essential,
                                              points_to,
                                              points_from,
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

  typedef std::unordered_map<std::size_t, std::size_t>::const_iterator I;

  out_result.resize(subset_count);
  while (subset_count--) {
    GenerateRandomSubset(min, max, count, out_result[subset_count]);
    std::sort(out_result[subset_count].begin(), out_result[subset_count].end());
    I b = matches.begin();
    size_t prev = 0;
    for (unsigned j = 0; j < out_result[subset_count].size(); ++j) {
      assert(matches.size() > out_result[subset_count][j]);
      std::advance(b, out_result[subset_count][j] - prev);
      prev = out_result[subset_count][j];
      out_result[subset_count][j] = b->first;
    }
  }
}

void TwoViewReconstructor::GenerateRandomSubset(const size_t min,
                                                const size_t max,
                                                const size_t count,
                                                std::vector<size_t> & out_result) {
  assert(max - min >= count);
//  std::random_device rand_dev;
//  std::mt19937 generator(rand_dev());
//  std::uniform_int_distribution<size_t> distr(min, max - 1);
//  std::unordered_set<size_t> chosen;
//  out_result.reserve(count);
//  while (chosen.size() < count)
//    chosen.insert(distr(generator));

  std::unordered_set<size_t> chosen;
  out_result.reserve(count);
  while (chosen.size() < count) {
    chosen.insert(rand() % (max));
  }
  std::copy(chosen.begin(), chosen.end(), std::back_inserter(out_result));
}

}
}
