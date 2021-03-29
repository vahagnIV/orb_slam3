//
// Created by vahagn on 08/02/21.
//

#ifndef ORB_SLAM3_TRANSFROMATION_ESTIMATOR_BASE_H
#define ORB_SLAM3_TRANSFROMATION_ESTIMATOR_BASE_H

// == orb-slam3 ===
#include <typedefs.h>
#include <features/match.h>
#include <geometry/pose.h>

namespace orb_slam3 {
namespace geometry {

class TransfromationEstimatorBase {
 public:
  TransfromationEstimatorBase(precision_t sigma) :
      sigma_threshold_(sigma),
      sigma_threshold__square_(sigma * sigma),
      sigma_squared_inv_(1 / sigma / sigma) {};

  size_t CheckPose(const Pose &solution,
                   const std::vector<HomogenousPoint> &points_to,
                   const std::vector<HomogenousPoint> &points_from,
                   const std::vector<features::Match> &matches,
                   std::vector<bool> &inliers,
                   precision_t &out_parallax,
                   std::vector<TPoint3D> &out_triangulated) const;
 protected:
  const precision_t sigma_threshold_;
  const precision_t sigma_threshold__square_;
  const precision_t sigma_squared_inv_;
  static const precision_t PARALLAX_THRESHOLD;

};
}
}

#endif //ORB_SLAM3_TRANSFROMATION_ESTIMATOR_BASE_H
