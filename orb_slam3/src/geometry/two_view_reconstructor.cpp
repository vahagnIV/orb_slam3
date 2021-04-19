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
                                       std::unordered_map<std::size_t, TPoint3D> & out_points,
                                       std::unordered_set<std::size_t> & out_inliers) const {

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
  if (h_score >= f_score) {
    return homography_matrix_estimator_.FindPose(homography,
                                                 points_to,
                                                 points_from,
                                                 matches,
                                                 out_inliers,
                                                 out_points,
                                                 out_pose);
  }
  return essential_matrix_estimator_.FindPose(essential,
                                              points_to,
                                              points_from,
                                              matches,
                                              out_inliers,
                                              out_points,
                                              out_pose);
}

void TwoViewReconstructor::GenerateRandomSubsets(const size_t min,
                                                 const size_t max,
                                                 const size_t count,
                                                 const size_t subset_count,
                                                 const std::unordered_map<std::size_t, std::size_t> & matches,
                                                 std::vector<std::vector<size_t>> & out_result) const {

  typedef std::unordered_map<std::size_t, std::size_t>::const_iterator I;

  assert(subset_count > 0);
  out_result.resize(subset_count);
  size_t s = subset_count - 1;
  do {
    GenerateRandomSubset(min, max, count, out_result[s]);
    std::sort(out_result[s].begin(), out_result[s].end());
    I b = matches.begin();
    size_t prev = 0;
    for (unsigned j = 0; j < out_result[s].size(); ++j) {
      assert(matches.size() > out_result[s][j]);
      b = std::next(b, out_result[s][j] - prev);
      prev = out_result[s][j];
      out_result[s][j] = b->first;
    }
  } while (s--);
}

void TwoViewReconstructor::GenerateRandomSubset(const size_t min,
                                                const size_t max,
                                                const size_t count,
                                                std::vector<size_t> & out_result) const {
  assert(max - min >= count);
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
