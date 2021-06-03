//
// Created by vahagn on 12/8/20.
//

#ifndef ORB_SLAM3_INCLUDE_MAP_POINT_H_
#define ORB_SLAM3_INCLUDE_MAP_POINT_H_
// === stl ===
#include <unordered_map>
#include <mutex>

// === orb-slam3 ===
#include <typedefs.h>
#include <features/features.h>
#include <features/ifeature_extractor.h>
#include <frame/observation.h>

namespace orb_slam3 {

namespace frame {
class KeyFrame;
}

namespace map {

class MapPoint {
 public:
  typedef std::unordered_map<frame::KeyFrame *, frame::Observation> MapType;
  MapPoint(TPoint3D point, size_t first_observed_frame_id, precision_t max_invariance_distance, precision_t min_invariance_distance);

  /*!
   * Adds frame to the map points observations
   * @param frame The frame
   * @param feature_id The id of the corresponding keypoint withing the frame
   */
  void AddObservation(const frame::Observation & observation);

  void EraseObservation(frame::KeyFrame *);

  void Refresh(const features::IFeatureExtractor * feature_extractor);

  void SetPosition(const TPoint3D & position);

  const features::DescriptorType GetDescriptor() const { return descriptor_; }

  const TPoint3D & GetPosition() const { return position_; }
  const TVector3D & GetNormal() const { return normal_; }

  const MapType Observations() const;
  size_t GetObservationCount() const;

  precision_t GetMaxInvarianceDistance() const { return 1.2 * max_invariance_distance_; }
  precision_t GetMinInvarianceDistance() const { return 0.8 * min_invariance_distance_; }
  bool IsInKeyFrame(frame::KeyFrame * keyframe);

  bool IsValid() const { return true; }

  void IncreaseVisible() { ++visible_; }
  void IncreaseFound() { ++found_; }

  size_t GetFirstObservedFrameId() const { return first_observed_frame_id_; }

  unsigned GetVisible() const { return visible_; }
  unsigned GetFound() const { return found_; }
  static size_t GetTotalMapPointCount() { return counter_; }
  inline bool IsBad() const { return bad_flag_; }
  void SetBad();
  ~MapPoint();

  void SetReplaced(map::MapPoint * replaced);
  map::MapPoint * GetReplaced();

  void UpdateNormalAndDepth();
 private:
  void ComputeDistinctiveDescriptor(const features::IFeatureExtractor * feature_extractor);
 private:
  static atomic_uint64_t counter_;
  // Position in the world coordinate system
  TPoint3D position_;
  MapType observations_;
  features::DescriptorType descriptor_;
  TVector3D normal_;
 public:
  precision_t max_invariance_distance_;
  precision_t min_invariance_distance_;
 private:
  unsigned visible_;
  unsigned found_;
  bool bad_flag_;
  size_t first_observed_frame_id_;

  // Mutex for locking observation
  mutable std::mutex feature_mutex_;

  // Mutex for locking position
  std::mutex position_mutex_;
  map::MapPoint * replaced_map_point_;

};

}
}  // namespace orb_slam3

#endif  // ORB_SLAM3_INCLUDE_MAP_POINT_H_
