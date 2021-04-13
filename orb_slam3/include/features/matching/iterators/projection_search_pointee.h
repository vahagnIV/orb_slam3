//
// Created by vahagn on 13/04/2021.
//

#ifndef ORB_SLAM3_ORB_SLAM3_INCLUDE_FEATURES_MATCHING_ITERATORS_PROJECTION_SEARCH_POINTEE_H_
#define ORB_SLAM3_ORB_SLAM3_INCLUDE_FEATURES_MATCHING_ITERATORS_PROJECTION_SEARCH_POINTEE_H_

// === orb_slam3 ===
#include <map/map_point.h>
#include "area_from_iterator.h"
#include <geometry/pose.h>
#include <camera/monocular_camera.h>

namespace orb_slam3 {
namespace features {
namespace matching {
namespace iterators {

class ProjectionSearchIterator;

class ProjectionSearchPointee {

  typedef map::MapPoint *id_type;
 public:
  friend class ProjectionSearchIterator;
  id_type GetId() const { return map_point_; }
  AreaFromIterator begin() { return begin_iterator_; }
  AreaFromIterator end() { return end_iterator_; }
  ProjectionSearchPointee(const Features *from_features) ;
 protected:
  void SetId(id_type id) { map_point_ = id; }
  bool SetMapPointAndCompute(map::MapPoint * map_point, const camera::MonocularCamera * camera, const geometry::Pose * pose);
  void InitializeIterators();
  static precision_t RadiusByViewingCos(const float & viewCos) ;
 private:
  id_type map_point_;
  AreaFromIterator end_iterator_;
  AreaFromIterator begin_iterator_;
  precision_t track_view_cos_;
  TPoint2D projected_;
  std::vector<std::size_t> from_indices_;
  const Features *from_features_;

};

}
}
}
}
#endif //ORB_SLAM3_ORB_SLAM3_INCLUDE_FEATURES_MATCHING_ITERATORS_PROJECTION_SEARCH_POINTEE_H_
