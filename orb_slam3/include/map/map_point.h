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
#include <features/features.h>
//#include <frame/frame_base.h>

namespace orb_slam3 {
namespace frame {
class FrameBase;
}
namespace map {

class MapPoint : protected g2o::VertexPointXYZ {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  MapPoint(const TPoint3D & point);
  MapPoint() {};

  void AddObservation(const frame::FrameBase * frame, size_t feature_id);
  void EraseObservation(const frame::FrameBase * frame);

  void Refresh();

  const TPoint3D & GetPose() const { return estimate(); }
  const TVector3D & GetNormal() const { return normal_; }

 private:
  void ComputeDistinctiveDescriptor();
  void UpdateNormalAndDepth();
 private:
  std::unordered_map<const frame::FrameBase *, size_t> obsevations_;
  features::DescriptorType descriptor_;
  TVector3D normal_;

};

}
}  // namespace orb_slam3

#endif  // ORB_SLAM3_INCLUDE_MAP_POINT_H_
