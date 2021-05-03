//
// Created by vahagn on 03.05.21.
//

#ifndef ORB_SLAM3_ORB_SLAM3_INCLUDE_FEATURES_MATCHING_VALIDATORS_TRIANGULATION_VALIDATOR_H_
#define ORB_SLAM3_ORB_SLAM3_INCLUDE_FEATURES_MATCHING_VALIDATORS_TRIANGULATION_VALIDATOR_H_

// === stl ===
#include <cstddef>

// === orb_slam3 ===
#include "imatch_validator.h"

namespace orb_slam3 {
namespace features {
namespace matching {
namespace validators {

class TriangulationValidator : public IMatchValidator<size_t, size_t> {
 public:
  TriangulationValidator();
  bool Validate(size_t to_id, size_t from_id) const override;

};

}
}
}
}
#endif //ORB_SLAM3_ORB_SLAM3_INCLUDE_FEATURES_MATCHING_VALIDATORS_TRIANGULATION_VALIDATOR_H_
