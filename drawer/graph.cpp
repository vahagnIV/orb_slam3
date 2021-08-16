//
// Created by vahagn on 16/08/2021.
//

#include "graph.h"
#include <cassert>

namespace orb_slam3 {
namespace drawer {

Graph::Graph() {

}

void Graph::AddNode(Node * node) {
  assert(!NodeExists(node->Id()));
  nodes_[node->Id()] = node;
}

void Graph::DeleteNode(size_t node_id) {

}

void Graph::AddEdge(Edge * edge) {

}

void Graph::DeleteEdge(size_t node_id1, size_t node_id2) {

}

void Graph::Draw() {

  for (const auto & node:nodes_) {
    node.second->Draw();
  }
}

const Node * Graph::GetNode(size_t node_id) const {
  auto it = nodes_.find(node_id);
  if (it == nodes_.end())
    return nullptr;
  return it->second;

}

bool Graph::NodeExists(size_t node_id) {
  return nodes_.find(node_id) != nodes_.end();
}

}
}