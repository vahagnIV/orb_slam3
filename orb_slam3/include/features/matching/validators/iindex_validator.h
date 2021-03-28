//
// Created by vahagn on 24.03.21.
//

#ifndef ORB_SLAM3_ORB_SLAM3_INCLUDE_FEATURES_MATCHING_VALIDATORS_IINDEX_VALIDATOR_H_
#define ORB_SLAM3_ORB_SLAM3_INCLUDE_FEATURES_MATCHING_VALIDATORS_IINDEX_VALIDATOR_H_
#include <cstddef>
namespace orb_slam3 {
namespace features {
namespace matching {
namespace validators {

/*!
 * Interface for SNNMatcher Match's MatchWithIterator method.
 * At each iteration of the upper loop (see SNNMatcher's documentation ) the
 * to-index is validate against the provided validator whilst the internal loop
 * validates the (to, from) pair of indices
 */
class IIndexValidator {
 public:
  /*!
   * Returns true if the argument is a valid index for snn-matching.
   * @param idx The index of the corresponding descriptor of the "to" coordinate frame
   * @return true if valid
   */
  virtual bool ValidateIdxTo(size_t idx) const = 0;

  /*!
   * Validates the pair of indices for snn-matching
   * @param idx_to The index of the corresponding descriptor of the "to" coordinate frame
   * @param idx_from The index of the corresponding descriptor of the "from" coordinate frame
   * @return true if the pair is valid
   */
  virtual bool ValidateIindices(size_t idx_to, size_t idx_from) const = 0;

  /*!
   * Virtual destructor
   */
  virtual ~IIndexValidator() = default;
};

}
}
}
}

#endif //ORB_SLAM3_ORB_SLAM3_INCLUDE_FEATURES_MATCHING_VALIDATORS_IINDEX_VALIDATOR_H_
