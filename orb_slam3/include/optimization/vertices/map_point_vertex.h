//
// Created by vahagn on 08.05.21.
//

#ifndef ORB_SLAM3_ORB_SLAM3_INCLUDE_OPTIMIZATION_VERTICES_MAP_POINT_VERTEX_H_
#define ORB_SLAM3_ORB_SLAM3_INCLUDE_OPTIMIZATION_VERTICES_MAP_POINT_VERTEX_H_
// === orb_slam3 ===
#include <g2o/types/slam3d/vertex_pointxyz.h>

// === orb_slam3 ===
#include <map/map_point.h>

namespace orb_slam3 {
namespace optimization {
namespace vertices {

class MapPointVertex : public g2o::VertexPointXYZ {
 public:
  MapPointVertex() = default;
  explicit MapPointVertex(map::MapPoint * map_point) : map_point_(map_point) {
    setEstimate(map_point->GetPosition());
    setId(map_point->Id());
    setMarginalized(true);
    setFixed(false);
  }

  map::MapPoint * GetMapPoint() { return map_point_; }

 private:
  map::MapPoint * map_point_;

};

}
}
}

#endif //ORB_SLAM3_ORB_SLAM3_INCLUDE_OPTIMIZATION_VERTICES_MAP_POINT_VERTEX_H_
