//
// Created by vahagn on 11/05/2021.
//

#ifndef ORB_SLAM3_SRC_FRAME_MONOCULAR_BASE_MONOCULAR_H_
#define ORB_SLAM3_SRC_FRAME_MONOCULAR_BASE_MONOCULAR_H_

#include <features/features.h>

namespace orb_slam3 {
namespace map {
class MapPoint;
}
namespace camera {
class MonocularCamera;
}
namespace frame {
namespace monocular {

class BaseMonocular {
 public:
  BaseMonocular(size_t image_width, size_t image_height,
                const camera::MonocularCamera * camera) : features_(image_width, image_height),
                                                          camera_(camera) {}
  virtual ~BaseMonocular() = default;
 public:
  void ListMapPoints(std::unordered_set<map::MapPoint *> & out_map_points) const {
    std::transform(map_points_.begin(),
                   map_points_.end(),
                   std::inserter(out_map_points, out_map_points.begin()),
                   [](const std::pair<size_t, map::MapPoint *> & mp_id) { return mp_id.second; });
  }

  const features::Features & GetFeatures() const { return features_; }
  const camera::MonocularCamera * GetCamera() const { return camera_; }

 protected:
  std::map<size_t, map::MapPoint *> map_points_;
 private:
  features::Features features_;
  const camera::MonocularCamera * camera_;

};

}
}
}

#endif //ORB_SLAM3_SRC_FRAME_MONOCULAR_BASE_MONOCULAR_H_
