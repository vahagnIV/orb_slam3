//
// Created by suren on 29.03.21.
//

#include <geometry/transfromation_estimator_base.h>
#include <geometry/utils.h>

namespace orb_slam3 {
namespace geometry {

const precision_t TransfromationEstimatorBase::PARALLAX_THRESHOLD = 0.99998;

size_t TransfromationEstimatorBase::CheckPose(const Pose & solution,
                                              const std::vector<HomogenousPoint> & points_to,
                                              const std::vector<HomogenousPoint> & points_from,
                                              const std::vector<features::Match> & matches,
                                              std::vector<bool> & inliers,
                                              precision_t & out_parallax,
                                              std::vector<TPoint3D> & out_triangulated) const {
  size_t count = 0;
  out_triangulated.resize(matches.size());
  std::vector<precision_t> triangulated_parallax;
  triangulated_parallax.reserve(matches.size());

  for (size_t i = 0; i < matches.size(); ++i) {

    if (!inliers[i])
      continue;

    const auto & match = matches[i];
    const HomogenousPoint point_to = points_to[match.to_idx];
    const HomogenousPoint point_from = points_from[match.from_idx];

    TPoint3D & triangulated = out_triangulated[i];

    precision_t point_parallax;
    if (!(inliers[i] = utils::TriangulateAndValidate(point_from,
                                                     point_to,
                                                     solution,
                                                     4 * sigma_threshold__square_,
                                                     4 * sigma_threshold__square_,
                                                     PARALLAX_THRESHOLD,
                                                     point_parallax,
                                                     triangulated)))
      continue;

    triangulated_parallax.push_back(point_parallax);
    ++count;
  }
  if (!count)
    return count;

  std::sort(triangulated_parallax.begin(), triangulated_parallax.end());
  out_parallax =
      std::acos(triangulated_parallax.size() < 50 ? triangulated_parallax.back() : triangulated_parallax[50]);

  return count;
}

bool TransfromationEstimatorBase::FindCorrectPose(Pose *possible_solutions, int solutions_size,
                                                  const std::vector<HomogenousPoint> & points_to,
                                                  const std::vector<HomogenousPoint> & points_from,
                                                  const std::vector<features::Match> & matches,
                                                  std::vector<bool> & out_inliers,
                                                  std::vector<TPoint3D> & out_triangulated,
                                                  Pose & out_pose) const {
  size_t best_count = 0, second_best_count = 0;
  precision_t best_parallax = -1;
  for (int i = 0; i < solutions_size; ++i) {
    Pose & sol = possible_solutions[i];
    std::vector<TPoint3D> tmp_triangulated;
    std::vector<bool> tmp_inliers(matches.size(), true);
    precision_t parallax;
    size_t no_good = CheckPose(sol, points_to, points_from, matches, tmp_inliers, parallax, tmp_triangulated);
    if (best_count < no_good) {
      second_best_count = best_count;
      best_count = no_good;
      out_triangulated = tmp_triangulated;
      out_inliers = tmp_inliers;
      out_pose = sol;
      best_parallax = parallax;
    } else
      second_best_count = std::max(second_best_count, no_good);
  }
  // TODO: address this issue
  return second_best_count < 0.75 * best_count && best_parallax < 0.995 && best_count > 30;
}

}
}