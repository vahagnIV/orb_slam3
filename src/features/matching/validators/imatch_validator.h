//
// Created by vahagn on 03.05.21.
//

#ifndef ORB_SLAM3_ORB_SLAM3_INCLUDE_FEATURES_MATCHING_ITERATORS_I_MATCH_VALIDATOR_H_
#define ORB_SLAM3_ORB_SLAM3_INCLUDE_FEATURES_MATCHING_ITERATORS_I_MATCH_VALIDATOR_H_
namespace orb_slam3 {
namespace features {
namespace matching {
namespace validators {

template<typename ToIdType, typename FromIdType>
class IMatchValidator {
 public:
  virtual bool Validate(ToIdType to_id,
                          FromIdType from_id) const = 0;
};

}
}
}
}
#endif //ORB_SLAM3_ORB_SLAM3_INCLUDE_FEATURES_MATCHING_ITERATORS_I_MATCH_VALIDATOR_H_
