//
// Created by vahagn on 08/02/21.
//

#ifndef ORB_SLAM3_TRANSFROMATION_ESTIMATOR_BASE_H
#define ORB_SLAM3_TRANSFROMATION_ESTIMATOR_BASE_H


// == stl ===
#include <unordered_set>

// == orb-slam3 ===
#include "../typedefs.h"
#include <features/features.h>
#include "pose.h"


namespace orb_slam3 {
namespace geometry {

class TransfromationEstimatorBase {
 public:
  TransfromationEstimatorBase(precision_t sigma, precision_t min_dif_ratio_from_second_best) :
      sigma_threshold_(sigma),
      sigma_threshold__square_(sigma * sigma),
      sigma_squared_inv_(1 / sigma / sigma),
      min_dif_ratio_from_second_best_(min_dif_ratio_from_second_best) {};

  size_t
  CheckPose(const Pose & solution,
            const features::Features & features_to,
            const features::Features & features_from,
            const std::unordered_map<std::size_t, std::size_t> & matches,
            precision_t & out_parallax,
            std::unordered_map<std::size_t, TPoint3D> & out_triangulated) const;
 protected:
  const precision_t sigma_threshold_;
  const precision_t sigma_threshold__square_;
  const precision_t sigma_squared_inv_;
  const precision_t min_dif_ratio_from_second_best_;
  static const precision_t PARALLAX_THRESHOLD;
  static const int MIN_TRIANGULATED;
  static const precision_t MIN_PARALLAX_DEG;
  static const precision_t MIN_MATCH_RATIO;
  bool FindCorrectPose(const std::vector<Pose> & candidate_solutions,
                       const features::Features & features_to,
                       const features::Features & features_from,
                       const std::unordered_map<std::size_t, std::size_t> & matches,
                       std::unordered_map<std::size_t, TPoint3D> & out_triangulated,
                       Pose & out_pose) const;
};
}
}

#endif //ORB_SLAM3_TRANSFROMATION_ESTIMATOR_BASE_H
