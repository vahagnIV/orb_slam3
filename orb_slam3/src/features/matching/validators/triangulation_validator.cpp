//
// Created by vahagn on 03.05.21.
//

#include "features/matching/validators/triangulation_validator.h"
#include <geometry/utils.h>
namespace orb_slam3 {
namespace features {
namespace matching {
namespace validators {

TriangulationValidator::TriangulationValidator(const Features * features_to,
                                               const Features * features_from,
                                               const geometry::Pose * relative_pose,
                                               const std::shared_ptr<IFeatureExtractor> & feature_extractor,
                                               const precision_t sigma)
    : features_to_(features_to),
      features_from_(features_from),
      relative_pose_(relative_pose),
      epipole_(ComputeEpipole()),
      feature_extractor_(feature_extractor),
      sigma_(sigma * sigma) {
}

bool TriangulationValidator::Validate(size_t to_id, size_t from_id) const {
  const HomogenousPoint & kp_to = features_to_->undistorted_and_unprojected_keypoints[to_id];
  const HomogenousPoint & kp_from = features_from_->undistorted_and_unprojected_keypoints[from_id];

  precision_t parallax;
  TPoint3D triangulated;
  return EpipolarDistanceIsEnough(to_id) && geometry::utils::TriangulateAndValidate(kp_from,
                                                                                    kp_to,
                                                                                    *relative_pose_,
                                                                                    sigma_,
                                                                                    sigma_,
                                                                                    constants::PARALLAX_THRESHOLD,
                                                                                    parallax,
                                                                                    triangulated);

}

bool TriangulationValidator::EpipolarDistanceIsEnough(size_t to_id) const {
  const HomogenousPoint & kp_to = features_to_->undistorted_and_unprojected_keypoints[to_id];
  precision_t delta_x = kp_to.x() - epipole_.x();
  precision_t delta_y = kp_to.y() - epipole_.y();
  return delta_x * delta_x + delta_y * delta_y
      > 100 * sigma_ * feature_extractor_->GetScaleFactors()[features_to_->keypoints[to_id].level];
}

HomogenousPoint TriangulationValidator::ComputeEpipole() const {
  HomogenousPoint epipole = relative_pose_->T;
  return epipole * (1. / epipole.z());
}

}
}
}
}