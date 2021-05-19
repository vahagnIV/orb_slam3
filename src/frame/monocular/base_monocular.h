//
// Created by vahagn on 11/05/2021.
//

#ifndef ORB_SLAM3_SRC_FRAME_MONOCULAR_BASE_MONOCULAR_H_
#define ORB_SLAM3_SRC_FRAME_MONOCULAR_BASE_MONOCULAR_H_

#include <mutex>

#include <features/features.h>
#include <map/map_point.h>

namespace orb_slam3 {
namespace camera {
class MonocularCamera;
}
namespace frame {
namespace monocular {

class BaseMonocular {
 public:
  typedef std::map<size_t, map::MapPoint *> MonocularMapPoints;
  BaseMonocular(const camera::MonocularCamera * camera)
      : features_(camera),
        camera_(camera) {}

  BaseMonocular(const BaseMonocular & other)
      : map_points_(other.map_points_),
        features_(other.features_),
        camera_(other.camera_),
        map_point_mutex_() {

  }
  virtual ~BaseMonocular() = default;
 public:
  void ListMapPoints(std::unordered_set<map::MapPoint *> & out_map_points) const {
    std::unique_lock<std::mutex> lock(map_point_mutex_);
    for (auto mp_id: map_points_) {
      if (!mp_id.second->IsBad()) {
        out_map_points.insert(mp_id.second);
      }
    }
  }

  MonocularMapPoints GetMapPoints() const {
    std::unique_lock<std::mutex> lock(map_point_mutex_);
    std::map<size_t, map::MapPoint *> result;
    std::copy_if(map_points_.begin(),
                 map_points_.end(),
                 std::inserter(result, result.begin()),
                 [](const std::pair<size_t, map::MapPoint *> & mp_id) { return !mp_id.second->IsBad(); });
    return result;
  }

  const features::Features & GetFeatures() const { return features_; }
  const camera::MonocularCamera * GetCamera() const { return camera_; }
  void ComputeBow() { features_.ComputeBow(); }
  void AddMapPoint(map::MapPoint * map_point, size_t feature_id) {
    assert(!MapPointExists(map_point));
//    assert(map_points_.find(feature_id) == map_points_.end());
    map_points_[feature_id] = map_point;
  }
  void EraseMapPoint(size_t feature_id) {
    assert(map_points_.find(feature_id) != map_points_.end());
    map_points_.erase(feature_id);
  }

  bool MapPointExists(const map::MapPoint * map_point) const {
    for (auto mp: map_points_)
      if (mp.second == map_point)
        return true;
    return false;
  }
 protected:
  MonocularMapPoints map_points_;
  features::Features features_;
 private:
  const camera::MonocularCamera * camera_;
  mutable std::mutex map_point_mutex_;

};

}
}
}

#endif //ORB_SLAM3_SRC_FRAME_MONOCULAR_BASE_MONOCULAR_H_
