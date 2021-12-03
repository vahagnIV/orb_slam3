//
// Created by vahagn on 12/8/20.
//

#ifndef ORB_SLAM3_INCLUDE_MAP_POINT_H_
#define ORB_SLAM3_INCLUDE_MAP_POINT_H_
// === stl ===
#include <unordered_map>
#include <mutex>
#include <shared_mutex>
#include <mutex>
#include <atomic>

// === orb-slam3 ===
#include <typedefs.h>
#include <features/features.h>
#include <features/ifeature_extractor.h>
#include <frame/observation.h>

namespace orb_slam3 {

namespace serialization {
class SerializationContext;
}

namespace frame {
class KeyFrame;
}

namespace map {

class Map;

class MapPoint {
 public:
  typedef std::unordered_map<const frame::KeyFrame *, frame::Observation> MapType;
  MapPoint(TPoint3D point,
           size_t first_observed_frame_id,
           precision_t max_invariance_distance,
           precision_t min_invariance_distance,
           Map * map);

  MapPoint(std::istream & istream, serialization::SerializationContext & context);

  /*!
   * Adds frame to the map points observations
   * @param frame The frame
   * @param feature_id The id of the corresponding keypoint withing the frame
   */
  void AddObservation(const frame::Observation & observation);

  void EraseObservation(frame::KeyFrame *);

//  void Refresh(const features::IFeatureExtractor * feature_extractor);

  void SetStagingPosition(const TPoint3D & position);

  void ApplyStaging();

  const features::DescriptorType & GetDescriptor() const { return descriptor_; }

  const TPoint3D GetPosition() const;
  const TVector3D GetNormal() const;

  const TPoint3D GetStagingPosition() const;
  const TVector3D GetStagingNormal() ;

  MapType Observations() const;
  MapType StagingObservations() const;
  size_t GetObservationCount() const;
  size_t GetStagingObservationCount() const;

  precision_t GetMaxInvarianceDistance() const { return 1.2 * max_invariance_distance_; }
  precision_t GetMinInvarianceDistance() const { return 0.8 * min_invariance_distance_; }

  precision_t GetStagingMaxInvarianceDistance() const { return 1.2 * staging_max_invariance_distance_; }
  precision_t GetStagingMinInvarianceDistance() const { return 0.8 * staging_min_invariance_distance_; }

  bool IsInKeyFrame(const frame::KeyFrame * keyframe) const;
  const frame::Observation & Observation(const frame::KeyFrame * key_frame) const;

  bool IsValid() const { return true; }

  void IncreaseVisible() { ++visible_; }
  void IncreaseFound() { ++found_; }

  size_t GetFirstObservedFrameId() const { return first_observed_frame_id_; }

  unsigned GetVisible() const { return visible_; }
  unsigned GetFound() const { return found_; }
  static size_t GetTotalMapPointCount() { return counter_; }
  Map * GetMap() const { return map_; }
  void SetMap(map::Map * map);
  inline bool IsBad() const { return bad_flag_; }
  void SetBad();
  ~MapPoint();

  void SetReplaced(map::MapPoint * replaced);
  map::MapPoint * GetReplaced();



  void SetStagingMaxInvarianceDistance(precision_t max_invariance_distance) {
    staging_max_invariance_distance_ = max_invariance_distance;
  }

  void SetStagingMinInvarianceDistance(precision_t min_invariance_distance) {
    staging_min_invariance_distance_ = min_invariance_distance;
  }

  bool GetObservation(const frame::KeyFrame * key_frame, frame::Observation & out_observation) const;
  bool GetStagingObservation(const frame::KeyFrame * key_frame, frame::Observation & out_observation) const;

  void Serialize(std::ostream & ostream) const;
 private:
  void ComputeDistinctiveDescriptor();
  void CalculateNormalStaging();
 private:
  static std::atomic_uint64_t counter_;
  // Position in the world coordinate system
  TPoint3D position_;
  TPoint3D staging_position_;
  bool position_changed_;
  mutable std::shared_mutex position_mutex_;

  // The keyframe => Observation map of observations
  MapType observations_;
  MapType staging_observations_;
  bool observations_changed_;
  mutable std::recursive_mutex observation_mutex_;

  // Distinctive descriptor of this map point
  features::DescriptorType descriptor_;
  features::DescriptorType staging_descriptor_;
  bool staging_desciptor_calculated_;

  // The normal
  bool staging_normal_calculated_;
  TVector3D normal_;
  TVector3D staging_normal_;

  precision_t max_invariance_distance_;
  precision_t staging_max_invariance_distance_;

  precision_t min_invariance_distance_;
  precision_t staging_min_invariance_distance_;

  // Statistics on how many times the map point should have been
  // visible and how many times it was actually found
  unsigned visible_;
  unsigned found_;

  Map * map_;

  bool bad_flag_;
  size_t first_observed_frame_id_;

  mutable std::recursive_mutex normal_mutex_;

  map::MapPoint * replaced_map_point_;

};

}
}  // namespace orb_slam3

#endif  // ORB_SLAM3_INCLUDE_MAP_POINT_H_
