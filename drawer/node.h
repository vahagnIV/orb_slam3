//
// Created by vahagn on 16/08/2021.
//

#ifndef ORB_SLAM3_DRAWER_NODE_H_
#define ORB_SLAM3_DRAWER_NODE_H_

#include <cstddef>
#include <unordered_map>

namespace orb_slam3 {
namespace drawer {

class Edge;

class Node {
 public:
  Node(size_t id);
  void AddEdge(const Edge * edge);
  const Edge * DeleteEdgeTo(const Node * other);
  size_t Id() const;
 private:
  size_t GetOtherNodeIdFromedge(const Edge * edge) const ;
 private:
  size_t id_;
  std::unordered_map<size_t, const Edge *> edges_;

};

}
}

#endif //ORB_SLAM3_DRAWER_NODE_H_
