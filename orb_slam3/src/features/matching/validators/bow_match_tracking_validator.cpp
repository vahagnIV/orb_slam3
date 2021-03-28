//
// Created by vahagn on 24.03.21.
//

#include "features/matching/validators/bow_match_tracking_validator.h"

namespace orb_slam3 {
namespace features {
namespace matching {
namespace validators {

BowMatchTrackingValidator::BowMatchTrackingValidator(const std::map<size_t, map::MapPoint *> & map_points_to,
                                                     const std::map<size_t, map::MapPoint *> & map_points_from,
                                                     bool to_exists,
                                                     bool from_exists)
    : map_points_to_(&map_points_to),
      map_points_from_(&map_points_from),
      to_exists_(to_exists),
      from_exists_(from_exists) {

}

bool BowMatchTrackingValidator::ValidateIdxTo(size_t idx) const {
  return to_exists_ ? map_points_to_->find(idx) != map_points_to_->end() :
         map_points_to_->find(idx) == map_points_to_->end();
}

bool BowMatchTrackingValidator::ValidateIindices(size_t idx_to, size_t idx_from) const {
  if (from_exists_)
    return map_points_from_->find(idx_from) != map_points_from_->end();
  else
    return map_points_from_->find(idx_from) == map_points_from_->end();
}

}
}
}
}
