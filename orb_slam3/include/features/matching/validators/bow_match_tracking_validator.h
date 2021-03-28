//
// Created by vahagn on 24.03.21.
//

#ifndef ORB_SLAM3_ORB_SLAM3_INCLUDE_FEATURES_MATCHING_ITERATORS_BOW_MATCH_VALIDATOR_H_
#define ORB_SLAM3_ORB_SLAM3_INCLUDE_FEATURES_MATCHING_ITERATORS_BOW_MATCH_VALIDATOR_H_

#include <features/matching/validators/iindex_validator.h>
#include <map/map_point.h>
#include <cstddef>

namespace orb_slam3 {
namespace features {
namespace matching {
namespace validators {

/*!
 * Matcher's Index validator for tracking algorithm. It validates to-index if
 * if the corresponding map point does not exist and accepts from-index if the corresponding
 * map-point already exists
 */
class BowMatchTrackingValidator : public IIndexValidator {
 public:
  BowMatchTrackingValidator(const std::map<size_t, map::MapPoint *> & map_points_to,
                            const std::map<size_t, map::MapPoint *> & map_points_from,
                            bool to_exists,
                            bool from_exists);
  /*!
   * Returns true id the map-point corresponding to feature idx does not exist yet
   * @param idx The index of the descriptor
   * @return true if idx is valid
   */
  bool ValidateIdxTo(size_t idx) const override;

  /*!
   * Return true if the key-point idx_from corresponds to a map-point
   * @param idx_to ignored
   * @param idx_from The index of a feature in the "from" frame
   * @return true if valid
   */
  bool ValidateIindices(size_t idx_to, size_t idx_from) const override;
 private:
  const std::map<size_t, map::MapPoint *> *map_points_to_;
  const std::map<size_t, map::MapPoint *> *map_points_from_;
  bool to_exists_;
  bool from_exists_;

};

}
}
}
}

#endif //ORB_SLAM3_ORB_SLAM3_INCLUDE_FEATURES_MATCHING_ITERATORS_BOW_MATCH_VALIDATOR_H_
