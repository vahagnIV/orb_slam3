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

namespace orb_slam3 {
namespace features {
namespace matching {
namespace iterators {

class ProjectionSearchIterator;

class ProjectionSearchPointee {

 public:
  typedef map::MapPoint * id_type;
  typedef VectorFromIterator<std::size_t> iterator;
  friend class ProjectionSearchIterator;
  id_type GetId() const { return map_point_; }
  iterator begin() { return begin_iterator_; }
  iterator end() { return end_iterator_; }
  ProjectionSearchPointee(const Features * from_features,
                          const IFeatureExtractor * feature_extractor,
                          const std::map<std::size_t, map::MapPoint *> * from_map_points,
                          unsigned radius_multiplier);
  const DescriptorType GetDescriptor() const;
 protected:
  void SetId(id_type id) { map_point_ = id; }
  bool SetMapPointAndCompute(map::MapPoint * map_point,
                             const camera::MonocularCamera * camera,
                             const geometry::Pose * pose);
  void InitializeIterators();
  static precision_t RadiusByViewingCos(const float & viewCos);
 private:
  id_type map_point_;
  iterator end_iterator_;
  iterator begin_iterator_;
  precision_t track_view_cos_;
  TPoint2D projected_;
  std::vector<std::size_t> from_indices_;
  const Features * from_features_;
  const IFeatureExtractor * feature_extractor_;
  precision_t window_size_;
  unsigned predicted_level_;
  const std::map<std::size_t, map::MapPoint *> * from_map_points_;
  unsigned radius_multiplier_;

  unsigned patchar1 = 0, patchar2 = 0, patchar3 = 0;
  ~ProjectionSearchPointee() {
    std::cerr << patchar1 << " " << patchar2 << " " << patchar3 << std::endl;
  }

};

}
}
}
}
#endif //ORB_SLAM3_ORB_SLAM3_INCLUDE_FEATURES_MATCHING_ITERATORS_PROJECTION_SEARCH_POINTEE_H_
