//
// Created by vahagn on 13/04/2021.
//

#ifndef ORB_SLAM3_ORB_SLAM3_INCLUDE_FEATURES_MATCHING_ITERATORS_PROJECTION_SEARCH_ITERATOR_H_
#define ORB_SLAM3_ORB_SLAM3_INCLUDE_FEATURES_MATCHING_ITERATORS_PROJECTION_SEARCH_ITERATOR_H_

// === stl =====
#include <unordered_set>

// === orb_slam3 ===
#include "projection_search_pointee.h"
#include "../../../frame/map_point_visibility_params.h"

namespace orb_slam3 {
namespace features {
namespace matching {
namespace iterators {

class ProjectionSearchIterator {
 public:
  typedef ProjectionSearchPointee::id_type id_type;
  typedef ProjectionSearchPointee value_type;
  ProjectionSearchIterator(std::list<frame::MapPointVisibilityParams>::const_iterator begin,
                           std::list<frame::MapPointVisibilityParams>::const_iterator end,
                           const Features * from_features);

  const ProjectionSearchPointee & operator*() const { return pointee_; }
  ProjectionSearchPointee & operator*() { return pointee_; }
  const ProjectionSearchPointee * operator->() const { return &pointee_; }
  ProjectionSearchPointee * operator->() { return &pointee_; }
  ProjectionSearchIterator & operator++();

  friend bool operator==(const ProjectionSearchIterator & a, const ProjectionSearchIterator & b) {
    return a.it_ == b.it_;
  }
  friend bool operator!=(const ProjectionSearchIterator & a, const ProjectionSearchIterator & b) {
    return a.it_ != b.it_;
  }
 private:
  std::list<frame::MapPointVisibilityParams>::const_iterator it_;
  std::list<frame::MapPointVisibilityParams>::const_iterator end_;
  ProjectionSearchPointee pointee_;

};

}
}
}
}
#endif //ORB_SLAM3_ORB_SLAM3_INCLUDE_FEATURES_MATCHING_ITERATORS_PROJECTION_SEARCH_ITERATOR_H_
