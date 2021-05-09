//
// Created by suren on 29.03.21.
//

#include <geometry/transfromation_estimator_base.h>
#include <geometry/utils.h>

namespace orb_slam3 {
namespace geometry {

const precision_t TransfromationEstimatorBase::PARALLAX_THRESHOLD = 0.99998;
const int TransfromationEstimatorBase::MIN_TRIANGULATED = 50;
const precision_t TransfromationEstimatorBase::MIN_PARALLAX_DEG = 1.0;
const precision_t TransfromationEstimatorBase::MIN_MATCH_RATIO = 0.75;

size_t TransfromationEstimatorBase::CheckPose(const Pose & solution,
                                              const std::vector<HomogenousPoint> & points_to,
                                              const std::vector<HomogenousPoint> & points_from,
                                              const std::unordered_map<std::size_t, std::size_t> & matches,
                                              precision_t & out_parallax,
                                              std::unordered_map<std::size_t, TPoint3D> & out_triangulated) const {
  size_t count = 0;
  std::vector<precision_t> triangulated_cos_parallax;
  triangulated_cos_parallax.reserve(matches.size());

  typedef std::unordered_map<std::size_t, std::size_t>::const_iterator I;
  for (I i = matches.begin(); i != matches.end(); ++i) {

    const HomogenousPoint & point_to = points_to[i->first];
    const HomogenousPoint & point_from = points_from[i->second];

    precision_t point_cos_parallax;
    TPoint3D triangulated;
    if (! utils::TriangulateAndValidate(point_from,
          point_to,
          solution,
          4 * sigma_threshold__square_,
          4 * sigma_threshold__square_,
          PARALLAX_THRESHOLD,
          point_cos_parallax,
          triangulated)) {

      continue;
    }
    out_triangulated[i->first] = triangulated;

    triangulated_cos_parallax.push_back(point_cos_parallax);
    ++count;
  }
  if (!count)
    return count;

  std::sort(triangulated_cos_parallax.begin(), triangulated_cos_parallax.end());
  out_parallax = std::acos(
      triangulated_cos_parallax.size() < MIN_TRIANGULATED ? triangulated_cos_parallax.back()
                                                          : triangulated_cos_parallax[MIN_TRIANGULATED])
      * 180 / M_PI;

  return count;
}

bool TransfromationEstimatorBase::FindCorrectPose(const std::vector<Pose> & candidate_solutions,
                                                  const std::vector<HomogenousPoint> & points_to,
                                                  const std::vector<HomogenousPoint> & points_from,
                                                  const std::unordered_map<std::size_t, std::size_t> & matches,
                                                  std::unordered_map<std::size_t, TPoint3D> & out_triangulated,
                                                  Pose & out_pose) const {
  size_t best_count = 0, second_best_count = 0;
  precision_t best_parallax = -1;
  for (const auto & candidate : candidate_solutions) {
    std::unordered_map<std::size_t, TPoint3D> tmp_triangulated;
    precision_t parallax;
    size_t no_good = CheckPose(candidate, points_to, points_from, matches, parallax, tmp_triangulated);
    if (best_count < no_good) {
      second_best_count = best_count;
      best_count = no_good;
      out_triangulated = tmp_triangulated;
      out_pose = candidate;
      best_parallax = parallax;
    } else
      second_best_count = std::max(second_best_count, no_good);
  }
  return second_best_count < min_dif_ratio_from_second_best_ * best_count
      && best_parallax > MIN_PARALLAX_DEG
      && best_count > MIN_TRIANGULATED
      && best_count > MIN_MATCH_RATIO * matches.size();
}

}
}
