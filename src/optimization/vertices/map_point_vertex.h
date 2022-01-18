//
// Created by vahagn on 08.05.21.
//

#ifndef ORB_SLAM3_ORB_SLAM3_INCLUDE_OPTIMIZATION_VERTICES_MAP_POINT_VERTEX_H_
#define ORB_SLAM3_ORB_SLAM3_INCLUDE_OPTIMIZATION_VERTICES_MAP_POINT_VERTEX_H_
// === orb_slam3 ===
#include <g2o/types/slam3d/vertex_pointxyz.h>
namespace orb_slam3 {
namespace map{
class MapPoint;
}
namespace optimization {
namespace vertices {

class MapPointVertex : public g2o::VertexPointXYZ {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  MapPointVertex() = default;
  explicit MapPointVertex(map::MapPoint * map_point);

  map::MapPoint * GetMapPoint() { return map_point_; }

 private:
  map::MapPoint * map_point_;

};

}
}
}

#endif //ORB_SLAM3_ORB_SLAM3_INCLUDE_OPTIMIZATION_VERTICES_MAP_POINT_VERTEX_H_
