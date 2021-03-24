//
// Created by vahagn on 24.03.21.
//

#ifndef ORB_SLAM3_ORB_SLAM3_INCLUDE_FEATURES_MATCHING_ITERATORS_BOW_MATCH_VALIDATOR_H_
#define ORB_SLAM3_ORB_SLAM3_INCLUDE_FEATURES_MATCHING_ITERATORS_BOW_MATCH_VALIDATOR_H_

#include <features/matching/validators/iindex_validator.h>
#include <cstddef>

namespace orb_slam3 {
namespace features {
namespace matching {
namespace iterators {

class BowMatchValidator : public IIndexValidator {
 public:
  bool ValidateIdxTo(size_t idx) const override;
  bool ValidateIdxFrom(size_t idx) const override;
  bool ValidateIindices(size_t idx) const override;

};

}
}
}
}

#endif //ORB_SLAM3_ORB_SLAM3_INCLUDE_FEATURES_MATCHING_ITERATORS_BOW_MATCH_VALIDATOR_H_
