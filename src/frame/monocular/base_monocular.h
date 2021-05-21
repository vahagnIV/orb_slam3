//
// Created by vahagn on 11/05/2021.
//

#ifndef ORB_SLAM3_SRC_FRAME_MONOCULAR_BASE_MONOCULAR_H_
#define ORB_SLAM3_SRC_FRAME_MONOCULAR_BASE_MONOCULAR_H_

/// === stl ====
#include <mutex>
#include <unordered_set>


// === orb_slam3 ===
#include <features/features.h>
#include <geometry/pose.h>

namespace orb_slam3 {

namespace camera {
class MonocularCamera;
}

namespace features {
class IFeatureExtractor;
}

namespace map {
class MapPoint;
}

namespace frame {
class VisibleMapPoint;

namespace monocular {

class BaseMonocular {
 public:
  typedef std::map<size_t, map::MapPoint *> MonocularMapPoints;
  explicit BaseMonocular(const camera::MonocularCamera * camera);

  BaseMonocular(const BaseMonocular & other);
  virtual ~BaseMonocular() = default;
 public:
  void ListMapPoints(std::unordered_set<map::MapPoint *> & out_map_points) const;
  MonocularMapPoints GetMapPoints() const;

  const features::Features & GetFeatures() const { return features_; }
  const camera::MonocularCamera * GetCamera() const { return camera_; }
  void ComputeBow() { features_.ComputeBow(); }
  void AddMapPoint(map::MapPoint * map_point, size_t feature_id);
  void EraseMapPoint(size_t feature_id);
  bool MapPointExists(const map::MapPoint * map_point) const;
 protected:
  bool IsVisible(map::MapPoint * map_point,
                 VisibleMapPoint & out_map_point,
                 precision_t radius_multiplier,
                 unsigned int window_size,
                 const geometry::Pose & pose,
                 const geometry::Pose & inverse_position,
                 const features::IFeatureExtractor * feature_extractor) const;
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
