//
// Created by vahagn on 25/03/2021.
//

#ifndef ORB_SLAM3_ORB_SLAM3_INCLUDE_FEATURES_MATCHING_VALIDATORS_I_MATCH_RESULT_VALIDATOR_H_
#define ORB_SLAM3_ORB_SLAM3_INCLUDE_FEATURES_MATCHING_VALIDATORS_I_MATCH_RESULT_VALIDATOR_H_
#include <vector>
namespace orb_slam3 {
namespace features {
namespace matching {
namespace validators {

/*!
 *
 */
class IMatchResultValidator {
 public:
  virtual int Validate(std::vector<int> & inout_matches_12) = 0;
  virtual ~IMatchResultValidator() = default;
};

}
}
}
}

#endif //ORB_SLAM3_ORB_SLAM3_INCLUDE_FEATURES_MATCHING_VALIDATORS_I_MATCH_RESULT_VALIDATOR_H_
