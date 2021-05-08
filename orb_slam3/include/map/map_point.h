//
// Created by vahagn on 12/8/20.
//

#ifndef ORB_SLAM3_INCLUDE_MAP_POINT_H_
#define ORB_SLAM3_INCLUDE_MAP_POINT_H_
// === stl ===
#include <unordered_map>

// === optimization ===
#include <g2o/types/slam3d/vertex_pointxyz.h>

// === orb-slam3 ===
#include <typedefs.h>
#include <identifiable.h>
#include <features/features.h>
#include <features/ifeature_extractor.h>
#include <frame/observation.h>

namespace orb_slam3 {
namespace frame {
class FrameBase;
}
namespace map {

class MapPoint : public Identifiable {
 public:
  typedef std::unordered_map<frame::FrameBase *, frame::Observation *> MapType;
  MapPoint(const TPoint3D & point, precision_t max_invariance_distance, precision_t min_invariance_distance);

  /*!
   * Adds frame to the map points observations
   * @param frame The frame
   * @param feature_id The id of the corresponding keypoint withing the frame
   */
  void AddObservation(frame::Observation * observation);

  void EraseObservation(frame::FrameBase *);

  void Refresh(const std::shared_ptr<features::IFeatureExtractor> & feature_extractor);

  void SetPosition(const TPoint3D & position);

  const features::DescriptorType GetDescriptor() const { return descriptor_; }

  const TPoint3D & GetPosition() const { return position_; }
  const TVector3D & GetNormal() const { return normal_; }

  const MapType & Observations() const { return observations_; }
  MapType & Observations() { return observations_; }

  precision_t GetMaxInvarianceDistance() const { return 1.2 * max_invariance_distance_; }
  precision_t GetMinInvarianceDistance() const { return 0.8 * min_invariance_distance_; }

  bool IsValid() const { return true; }

  g2o::VertexPointXYZ * CreateVertex() const;
  void IncreaseVisible() { ++visible_; }
  void IncreaseFound() { ++found_; }

  unsigned GetVisible() const { return visible_; }
  unsigned GetFound() const { return found_; }

 private:
  void ComputeDistinctiveDescriptor(const std::shared_ptr<features::IFeatureExtractor> & feature_extractor);
  void UpdateNormalAndDepth();
 private:
  // Position in the world coordinate system
  TPoint3D position_;
  MapType observations_;
  features::DescriptorType descriptor_;
  TVector3D normal_;
  precision_t max_invariance_distance_;
  precision_t min_invariance_distance_;
  unsigned visible_;
  unsigned found_;

};

}
}  // namespace orb_slam3

#endif  // ORB_SLAM3_INCLUDE_MAP_POINT_H_
