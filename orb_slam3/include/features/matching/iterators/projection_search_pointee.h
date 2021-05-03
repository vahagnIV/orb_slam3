//
// Created by vahagn on 13/04/2021.
//

#ifndef ORB_SLAM3_ORB_SLAM3_INCLUDE_FEATURES_MATCHING_ITERATORS_PROJECTION_SEARCH_POINTEE_H_
#define ORB_SLAM3_ORB_SLAM3_INCLUDE_FEATURES_MATCHING_ITERATORS_PROJECTION_SEARCH_POINTEE_H_

// === orb_slam3 ===
#include <map/map_point.h>
#include "vector_from_iterator.h"
#include <geometry/pose.h>
#include <camera/monocular_camera.h>
#include <frame/visible_map_point.h>
#include <frame/covisibility_graph_node.h>

namespace orb_slam3 {
namespace features {
namespace matching {
namespace iterators {

class ProjectionSearchIterator;

class ProjectionSearchPointee {

 public:
  typedef map::MapPoint * id_type;
  typedef VectorFromIterator<std::size_t> iterator;

  ProjectionSearchPointee(const Features * from_features, const std::map<std::size_t, map::MapPoint *> * from_map_points);
  id_type GetId() const { return map_point_.map_point; }
  iterator begin() { return begin_iterator_; }
  iterator end() { return end_iterator_; }
  const DescriptorType GetDescriptor() const;
  void SetVisibileMapPoint(const frame::VisibleMapPoint & visible_map_point);
 protected:
  void InitializeIterators();
 private:
  frame::VisibleMapPoint map_point_;
  const Features * from_features_;

  iterator end_iterator_;
  iterator begin_iterator_;
  std::vector<std::size_t> from_indices_;
  const std::map<std::size_t, map::MapPoint *> * from_map_points_;

};

}
}
}
}
#endif //ORB_SLAM3_ORB_SLAM3_INCLUDE_FEATURES_MATCHING_ITERATORS_PROJECTION_SEARCH_POINTEE_H_
