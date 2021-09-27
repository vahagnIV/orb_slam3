//
// Created by vahagn on 16/08/2021.
//

#ifndef ORB_SLAM3_DRAWER_EDGE_H_
#define ORB_SLAM3_DRAWER_EDGE_H_

#include "node.h"

namespace orb_slam3 {
namespace drawer {

class Edge {
 public:
  Edge(const Node * node1, const Node * node2);
  const Node * Node1() const;
  const Node * Node2() const;
 private:
  const Node * node1_;
  const Node * node2_;

};

}
}

#endif //ORB_SLAM3_DRAWER_EDGE_H_
