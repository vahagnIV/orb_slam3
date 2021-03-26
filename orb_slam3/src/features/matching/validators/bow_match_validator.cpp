//
// Created by vahagn on 24.03.21.
//

#include "features/matching/validators/bow_match_validator.h"

namespace orb_slam3 {
namespace features {
namespace matching {
namespace validators {

BowMatchValidator::BowMatchValidator(const std::map<size_t, map::MapPoint *> &map_points_to,
                                     const std::map<size_t, map::MapPoint *> &map_points_from)
    : map_points_to_(&map_points_to), map_points_from_from_(&map_points_from) {

}

bool BowMatchValidator::ValidateIdxTo(size_t idx) const {
  return map_points_to_->find(idx) == map_points_to_->end();
}

bool BowMatchValidator::ValidateIindices(size_t idx_to, size_t idx_from) const {
  return map_points_from_from_->find(idx_from) != map_points_from_from_->end();
}

}
}
}
}
