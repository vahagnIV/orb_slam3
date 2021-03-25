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

class BowMatchValidator : public IIndexValidator {
 public:
  BowMatchValidator(const std::map<size_t, map::MapPoint *> & map_points_to, const std::map<size_t, map::MapPoint *> & map_points_from);
  bool ValidateIdxTo(size_t idx) const override;
  bool ValidateIindices(size_t idx_to, size_t idx_from) const override;
 private:
  const std::map<size_t, map::MapPoint *> * map_points_to_;
  const std::map<size_t, map::MapPoint *> * map_points_from_from_;

};


}
}
}
}

#endif //ORB_SLAM3_ORB_SLAM3_INCLUDE_FEATURES_MATCHING_ITERATORS_BOW_MATCH_VALIDATOR_H_
