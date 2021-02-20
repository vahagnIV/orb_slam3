//
// Created by vahagn on 12/8/20.
//

#ifndef ORB_SLAM3_INCLUDE_MAP_POINT_H_
#define ORB_SLAM3_INCLUDE_MAP_POINT_H_
// === stl ===
#include <unordered_set>

// === g2o ===
#include <g2o/types/slam3d/vertex_pointxyz.h>

// === orb-slam3 ===
#include <frame/frame_base.h>

namespace orb_slam3 {
namespace map {

 class MapPoint: protected g2o::VertexPointXYZ {
 public:
  MapPoint() {};
 private:
  std::unordered_set<std::shared_ptr<orb_slam3::frame::FrameBase>> frames_;

};

}
}  // namespace orb_slam3

#endif  // ORB_SLAM3_INCLUDE_MAP_POINT_H_
