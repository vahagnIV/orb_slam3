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

class MapPoint : protected Identifiable, protected g2o::VertexPointXYZ {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  typedef std::unordered_map<const frame::FrameBase *, size_t> MapType;
  MapPoint(const TPoint3D &point);
  MapPoint() : Identifiable() { setId(id_); };
  operator g2o::VertexPointXYZ *() { return this; }
  void AddObservation(const frame::FrameBase *frame, size_t feature_id);
  void EraseObservation(const frame::FrameBase *frame);

  void Refresh();

  const TPoint3D &GetPose() const { return estimate(); }
  const TVector3D &GetNormal() const { return normal_; }
  const MapType &Observations() const {
    return obsevations_;
  }

 private:
  void ComputeDistinctiveDescriptor();
  void UpdateNormalAndDepth();
 private:
  MapType obsevations_;
  features::DescriptorType descriptor_;
  TVector3D normal_;

};

}
}  // namespace orb_slam3

#endif  // ORB_SLAM3_INCLUDE_MAP_POINT_H_
