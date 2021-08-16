//
// Created by vahagn on 16/08/2021.
//

#include <cassert>

#include "node.h"
#include "edge.h"

namespace orb_slam3 {
namespace drawer {

Node::Node(size_t id) : id_(id) {

}
void Node::AddEdge(const Edge * edge) {
  size_t other_id = GetOtherNodeIdFromedge(edge);
  if (edges_.find(other_id) != edges_.end())
    assert(!"Edge is already in the graph");
  edges_[other_id] = edge;
}

size_t Node::GetOtherNodeIdFromedge(const Edge * edge) const {
  if (edge->Node1() == this) {
    return edge->Node2()->Id();
  }

  if (edge->Node2() == this)
    return edge->Node1()->Id();

  assert(!"Invalid edge");
}

const Edge * Node::DeleteEdgeTo(const Node * other) {
  auto it = edges_.find(other->Id());
  if (it == edges_.end())
    return nullptr;
  const Edge * edge = it->second;
  edges_.erase(it);
  return edge;
}

size_t Node::Id() const {
  return id_;
}

}
}