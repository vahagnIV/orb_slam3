//
// Created by vahagn on 28/03/2021.
//

#include "features/matching/validators/bow_match_local_mapping_validator.h"
#include <geometry/utils.h>

namespace orb_slam3 {
namespace features {
namespace matching {
namespace validators {

BowMatchLocalMappingValidator::BowMatchLocalMappingValidator(const Features & features_to,
                                                             const Features & features_from,
                                                             const IFeatureExtractor *extractor_to,
                                                             const IFeatureExtractor *extractor_from,
                                                             const precision_t f_to,
                                                             const precision_t f_from,
                                                             const geometry::Pose *pose)
    : features_to_(&features_to),
      features_from_(&features_from),
      extractor_to_(extractor_to),
      extractor_from_(extractor_from),
      f_inv_to_(f_to),
      f_inv_from_(f_from),
      pose_(pose) {

}

bool BowMatchLocalMappingValidator::ValidateIdxTo(size_t idx) const {
  return true;
}

bool BowMatchLocalMappingValidator::ValidateIindices(size_t idx_to, size_t idx_from) const {

  precision_t parallax;
  TPoint3D triangulated;
  return geometry::utils::TriangulateAndValidate(features_from_->undistorted_and_unprojected_keypoints[idx_from],
                                                 features_to_->undistorted_and_unprojected_keypoints[idx_to],
                                                 *pose_,
                                                 f_inv_to_ * f_inv_to_
                                                     * extractor_to_->GetAcceptableSquareError(features_to_->keypoints[idx_to].level),
                                                 f_inv_from_ * f_inv_from_
                                                     * extractor_from_->GetAcceptableSquareError(features_from_->keypoints[idx_from].level),
                                                 0.99998,
                                                 parallax, triangulated);

}

}
}
}
}