//
// Created by vahagn on 13/04/2021.
//

#ifndef ORB_SLAM3_ORB_SLAM3_INCLUDE_FEATURES_MATCHING_ITERATORS_PROJECTION_SEARCH_ITERATOR_H_
#define ORB_SLAM3_ORB_SLAM3_INCLUDE_FEATURES_MATCHING_ITERATORS_PROJECTION_SEARCH_ITERATOR_H_

// === stl =====
#include <unordered_set>

// === orb_slam3 ===
#include "projection_search_pointee.h"

namespace orb_slam3 {
namespace features {
namespace matching {
namespace iterators {

class ProjectionSearchIterator {
 public:
  typedef ProjectionSearchPointee::id_type id_type;
  typedef ProjectionSearchPointee value_type ;
  ProjectionSearchIterator(std::unordered_set<map::MapPoint *>::iterator begin,
                           std::unordered_set<map::MapPoint *>::iterator end,
                           const std::unordered_set<map::MapPoint *> *to_map_points,
                           const Features *from_features,
                           const geometry::Pose *pose,
                           const camera::MonocularCamera *camera,
                           const IFeatureExtractor * feature_extractor);
  const ProjectionSearchPointee & operator*() const { return pointee_; }
  ProjectionSearchPointee & operator*() { return pointee_; }
  const ProjectionSearchPointee *operator->() const { return &pointee_; }
  ProjectionSearchPointee *operator->() { return &pointee_; }
  ProjectionSearchIterator & operator++() ;

  friend bool operator==(const ProjectionSearchIterator & a, const ProjectionSearchIterator & b) {
    return a.pointee_.GetId() == b.pointee_.GetId();
  }
  friend bool operator!=(const ProjectionSearchIterator & a, const ProjectionSearchIterator & b) {
    return a.pointee_.GetId() != b.pointee_.GetId();
  }
 private:
  void AdvanceIteratorUntilGood();
  std::unordered_set<map::MapPoint *>::iterator it_;
  std::unordered_set<map::MapPoint *>::iterator end_;
  const std::unordered_set<map::MapPoint *> *to_map_points_;
  const geometry::Pose *pose_;
  const camera::MonocularCamera *camera_;
  ProjectionSearchPointee pointee_;


};

}
}
}
}
#endif //ORB_SLAM3_ORB_SLAM3_INCLUDE_FEATURES_MATCHING_ITERATORS_PROJECTION_SEARCH_ITERATOR_H_
