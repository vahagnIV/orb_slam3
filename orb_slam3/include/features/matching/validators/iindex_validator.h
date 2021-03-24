//
// Created by vahagn on 24.03.21.
//

#ifndef ORB_SLAM3_ORB_SLAM3_INCLUDE_FEATURES_MATCHING_VALIDATORS_IINDEX_VALIDATOR_H_
#define ORB_SLAM3_ORB_SLAM3_INCLUDE_FEATURES_MATCHING_VALIDATORS_IINDEX_VALIDATOR_H_
#include <cstddef>
namespace orb_slam3 {
namespace features {
namespace matching {

class IIndexValidator {
 public:
  virtual bool ValidateIdxTo(size_t idx) const = 0;
  virtual bool ValidateIdxFrom(size_t idx) const = 0;
  virtual bool ValidateIindices(size_t idx) const = 0;

};

}
}
}

#endif //ORB_SLAM3_ORB_SLAM3_INCLUDE_FEATURES_MATCHING_VALIDATORS_IINDEX_VALIDATOR_H_
