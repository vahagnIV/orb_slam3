//
// Created by vahagn on 13/04/2021.
//

#include "projection_search_pointee.h"

namespace orb_slam3 {
namespace features {
namespace matching {
namespace iterators {
ProjectionSearchPointee::ProjectionSearchPointee(const Features * from_features,
                                                 const std::map<std::size_t, map::MapPoint *> * from_map_points)
    : from_features_(from_features),
      from_map_points_(from_map_points) {
}

void ProjectionSearchPointee::SetVisibileMapPoint(const frame::MapPointVisibilityParams & visible_map_point) {
  map_point_ = visible_map_point;
  InitializeIterators();
}

void ProjectionSearchPointee::InitializeIterators() {

  from_features_->ListFeaturesInArea(map_point_.position,
                                     map_point_.window_size,
                                     std::max(map_point_.level - 1, 0),
                                     map_point_.level + 1,
                                     from_indices_);

  begin_iterator_ =
      iterator(from_indices_.begin(), from_indices_.end(), &(from_features_->descriptors));
  end_iterator_ = iterator(from_indices_.end(), from_indices_.end(), &(from_features_->descriptors));
}

const DescriptorType ProjectionSearchPointee::GetDescriptor() const {
  return map_point_.map_point->GetDescriptor();
}

}
}
}
}