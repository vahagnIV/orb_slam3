//
// Created by vahagn on 16/08/2021.
//

#ifndef ORB_SLAM3_DRAWER_GRAPH_H_
#define ORB_SLAM3_DRAWER_GRAPH_H_

#include <unordered_map>

#include "node.h"
#include "edge.h"

namespace orb_slam3 {
namespace drawer {

class Graph {
 public:
  void AddNode(const Node * node);
  void DeleteNode(size_t node_id);
  void AddEdge(Edge * edge);
  void DeleteEdge(size_t node_id1, size_t node_id2);
 private:
  std::unordered_map<size_t, Node *> nodes_;

};

}
}

#endif //ORB_SLAM3_DRAWER_GRAPH_H_
