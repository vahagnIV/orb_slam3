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
  typedef std::unordered_map<const frame::FrameBase *, size_t> MapType;
  MapPoint(const TPoint3D &point);

  void AddObservation(const frame::FrameBase *frame, size_t feature_id);

  void EraseObservation(const frame::FrameBase *frame);

  void Refresh();

  const TPoint3D &GetPosition() const { return position_; }

  void SetPosition(const TPoint3D &position);

  const TVector3D &GetNormal() const { return normal_; }

  const MapType &Observations() const { return observations_; }

 private:
  void ComputeDistinctiveDescriptor();

  void UpdateNormalAndDepth();

 private:
  // Position in the world coordinate system
  TPoint3D position_;
  MapType observations_;
  features::DescriptorType descriptor_;
  TVector3D normal_;

};

}
}  // namespace orb_slam3

#endif  // ORB_SLAM3_INCLUDE_MAP_POINT_H_
