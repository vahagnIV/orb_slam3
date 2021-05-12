//
// Created by vahagn on 12/05/2021.
//

#ifndef ORB_SLAM3_SRC_OPTIMIZATION_EDGES_BA_BINARY_EDGE_H_
#define ORB_SLAM3_SRC_OPTIMIZATION_EDGES_BA_BINARY_EDGE_H_

// === g2o ===
#include <g2o/core/base_binary_edge.h>

// === orb_sam3===
#include <optimization/vertices/frame_vertex.h>
#include <optimization/vertices/map_point_vertex.h>

namespace orb_slam3 {
namespace optimization {
namespace edges {

typedef g2o::BaseBinaryEdge<2, Eigen::Vector2d,
                            vertices::FrameVertex,
                            vertices::MapPointVertex> G2OBinaryEdge;

class BABinaryEdge : public  G2OBinaryEdge {
 public:
  BABinaryEdge() :G2OBinaryEdge(){ }
  virtual bool IsValid() const = 0;
  virtual bool IsDepthPositive() const = 0;

};

}
}
}
#endif //ORB_SLAM3_SRC_OPTIMIZATION_EDGES_BA_BINARY_EDGE_H_
