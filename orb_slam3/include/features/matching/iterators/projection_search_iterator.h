//
// Created by vahagn on 13/04/2021.
//

#ifndef ORB_SLAM3_ORB_SLAM3_INCLUDE_FEATURES_MATCHING_ITERATORS_PROJECTION_SEARCH_ITERATOR_H_
#define ORB_SLAM3_ORB_SLAM3_INCLUDE_FEATURES_MATCHING_ITERATORS_PROJECTION_SEARCH_ITERATOR_H_

// === stl =====
#include <unordered_set>

// === orb_slam3 ===
#include "projection_search_pointee.h"
#include <frame/visible_map_point.h>

namespace orb_slam3 {
namespace features {
namespace matching {
namespace iterators {

class ProjectionSearchIterator {
 public:
  typedef ProjectionSearchPointee::id_type id_type;
  typedef ProjectionSearchPointee value_type;
  ProjectionSearchIterator(std::list<frame::VisibleMapPoint>::const_iterator begin,
                           std::list<frame::VisibleMapPoint>::const_iterator end,
                           const Features * from_features,
                           const std::map<std::size_t, map::MapPoint *> * from_map_points);

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
  std::list<frame::VisibleMapPoint>::const_iterator it_;
  std::list<frame::VisibleMapPoint>::const_iterator end_;
  ProjectionSearchPointee pointee_;

};

}
}
}
}
#endif //ORB_SLAM3_ORB_SLAM3_INCLUDE_FEATURES_MATCHING_ITERATORS_PROJECTION_SEARCH_ITERATOR_H_
