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
class MapPointVisibilityParams;

namespace monocular {

class BaseMonocular {

  /// Special member functions
 public:
  typedef std::map<size_t, map::MapPoint *> MonocularMapPoints;
  explicit BaseMonocular(const camera::MonocularCamera * camera);

  BaseMonocular(const BaseMonocular & other);
  virtual ~BaseMonocular() = default;

  /// Public Functions
 public:
  void ListMapPoints(std::unordered_set<map::MapPoint *> & out_map_points) const;
  const MonocularMapPoints & GetMapPoints() const;
  size_t GetMapPointsCount() const;
  map::MapPoint* GetMapPoint(size_t feature_id) const;
  void ClearMapPoints();

  const camera::MonocularCamera * GetCamera() const { return camera_; }
  bool MapPointExists(const map::MapPoint * map_point) const;
 public:
  virtual void AddMapPoint(map::MapPoint * map_point, size_t feature_id);
  virtual map::MapPoint * EraseMapPoint(size_t feature_id);

  template<typename TMap, typename TSet>
  static void MapToSet(const TMap & map, TSet & out_set) {
    std::transform(map.begin(),
                   map.end(),
                   std::inserter(out_set, out_set.begin()),
                   [](const typename TMap::value_type & pair) {
                     return pair.second;
                   });
  }

 protected:

  /*!
   *
   * @param mp_local_coords - the point coordinates in local cf
   * @param mp_world_coords - the point coordinates in the world cf of the current frame
   * @param min_allowed_distance min invariance distance in local cf
   * @param max_allowed_distance max invariance distance in local cf
   * @param normal The normal in the world cf of the current frame
   * @param local_position - the world coordinates of current cf
   * @param radius_multiplier
   * @param level
   * @param out_map_point
   * @param feature_extractor
   */
  bool PointVisible(const TPoint3D & mp_local_coords,
                    const TPoint3D & mp_world_coords,
                    precision_t min_allowed_distance,
                    precision_t max_allowed_distance,
                    const TVector3D & normal,
                    const TVector3D & local_position,
                    precision_t radius_multiplier,
                    int level,
                    MapPointVisibilityParams & out_map_point,
                    const features::IFeatureExtractor * feature_extractor) const;

  void SerializeToStream(std::ostream & stream) const;
 protected:
  /// Private member variables

 private:
  MonocularMapPoints map_points_;
  const camera::MonocularCamera * camera_;

};

}
}
}

#endif //ORB_SLAM3_SRC_FRAME_MONOCULAR_BASE_MONOCULAR_H_
