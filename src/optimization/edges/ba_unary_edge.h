//
// Created by vahagn on 12/05/2021.
//

#ifndef ORB_SLAM3_SRC_OPTIMIZATION_EDGES_BA_UNARY_EDGE_H_
#define ORB_SLAM3_SRC_OPTIMIZATION_EDGES_BA_UNARY_EDGE_H_

// === g2o ===
#include <g2o/core/base_unary_edge.h>

namespace orb_slam3 {

namespace map {
class MapPoint;
}

namespace optimization {

namespace edges {
class BAUnaryEdge : public g2o::BaseUnaryEdge<2, Eigen::Vector2d, vertices::FrameVertex> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  BAUnaryEdge(map::MapPoint * map_point, size_t feature_id) : map_point_(map_point), feature_id_(feature_id) {}
  map::MapPoint * GetMapPoint() const { return map_point_; }
  size_t GetFeatureId() const { return feature_id_; }

 private:
  map::MapPoint * map_point_;
  size_t feature_id_;

};
}
}
}

#endif //ORB_SLAM3_SRC_OPTIMIZATION_EDGES_BA_UNARY_EDGE_H_
