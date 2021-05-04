//
// Created by vahagn on 03.05.21.
//

#ifndef ORB_SLAM3_ORB_SLAM3_INCLUDE_FEATURES_MATCHING_VALIDATORS_TRIANGULATION_VALIDATOR_H_
#define ORB_SLAM3_ORB_SLAM3_INCLUDE_FEATURES_MATCHING_VALIDATORS_TRIANGULATION_VALIDATOR_H_

// === stl ===
#include <cstddef>

// === orb_slam3 ===
#include "imatch_validator.h"
#include <features/features.h>
#include <features/ifeature_extractor.h>
#include <geometry/pose.h>

namespace orb_slam3 {
namespace features {
namespace matching {
namespace validators {

class TriangulationValidator : public IMatchValidator<size_t, size_t> {
 public:
  TriangulationValidator(const Features * features_to,
                         const Features * features_from,
                         const geometry::Pose * relative_pose,
                         const std::shared_ptr<IFeatureExtractor> & feature_extractor,
                         const precision_t sigma);
  bool Validate(size_t to_id, size_t from_id) const override;
 private:
  HomogenousPoint ComputeEpipole() const;
  bool EpipolarDistanceIsEnough(size_t to_id) const;
 private:
  const Features * features_to_;
  const Features * features_from_;
  const geometry::Pose * relative_pose_;
  const HomogenousPoint epipole_;
  const std::shared_ptr<IFeatureExtractor> feature_extractor_;
  const precision_t sigma_;

};

}
}
}
}
#endif //ORB_SLAM3_ORB_SLAM3_INCLUDE_FEATURES_MATCHING_VALIDATORS_TRIANGULATION_VALIDATOR_H_
