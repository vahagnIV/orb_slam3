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
//#include <frame/frame_base.h>

namespace orb_slam3 {
namespace frame {
class FrameBase;
}
namespace map {

class MapPoint : public Identifiable {
 public:
  typedef std::unordered_map<frame::FrameBase *, size_t> MapType;
  MapPoint(const TPoint3D & point, precision_t max_invariance_distance, precision_t min_invariance_distance);

  /*!
   * Adds frame to the map points observations and increases the corresponding weights
   * in the frame covidibility weight counter
   * @param frame The frame
   * @param feature_id The id of the corresponding keypoint withing the frame
   */
  void AddObservation(frame::FrameBase *frame, size_t feature_id);

  void EraseObservation(frame::FrameBase *frame);

  void Refresh();

  void SetPosition(const TPoint3D & position);

  const TPoint3D & GetPosition() const { return position_; }
  const TVector3D & GetNormal() const { return normal_; }
  const MapType & Observations() const { return observations_; }
  precision_t GetMaxInvarianceDistance() const { return 1.2 * max_invariance_distance_; }
  precision_t GetMinInvarianceDistance() const { return 0.8 * min_invariance_distance_; }
  MapType & Observations() { return observations_; }
  bool IsValid() const { return true; }

  g2o::VertexPointXYZ *CreateVertex() const;

 private:
  void ComputeDistinctiveDescriptor();

  void UpdateNormalAndDepth();

 private:
  // Position in the world coordinate system
  TPoint3D position_;
  MapType observations_;
  features::DescriptorType descriptor_;
  TVector3D normal_;
  precision_t max_invariance_distance_;
  precision_t min_invariance_distance_;

};

}
}  // namespace orb_slam3

#endif  // ORB_SLAM3_INCLUDE_MAP_POINT_H_
