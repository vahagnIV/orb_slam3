//
// Created by vahagn on 16/08/2021.
//

#include "graph.h"
#include <cassert>
#include <shaders/shader_repository.h>
#include <shaders/color_repository.h>
#include "key_frame_node.h"

namespace orb_slam3 {
namespace drawer {

Graph::Graph() {

}

void Graph::AddMapPoint(MapPointNode *map_point_node) {
  map_points_[map_point_node->Id()] = map_point_node;
}

void Graph::AddKeyFrame(KeyFrameNode *key_frame_node) {
  keyframes_[key_frame_node->Id()] = key_frame_node;
}

void Graph::DeleteMapPoint(size_t id) {
  auto it = map_points_.find(id);
  if (it != map_points_.end())
    map_points_.erase(it);
}

void Graph::DeleteKeyFrane(size_t id) {
  auto it = keyframes_.find(id);
  if (it != keyframes_.end())
    keyframes_.erase(it);
}

MapPointNode *Graph::GetMapPoint(size_t id) {
  auto it = map_points_.find(id);
  if (it != map_points_.end())
    return it->second;
  return nullptr;
}

KeyFrameNode *Graph::GetKeyFrame(size_t id) {
  auto it = keyframes_.find(id);
  if (it != keyframes_.end())
    return it->second;
  return nullptr;
}

void Graph::AddNode(Node *node) {
  assert(!NodeExists(node->Id()));
  nodes_[node->Id()] = node;
}

void Graph::DeleteNode(size_t node_id) {
  if (!NodeExists(node_id))
    return;
//  assert(NodeExists(node_id));
  Node * node = GetNode(node_id);
  nodes_.erase(node_id);
  delete node;
}

void Graph::AddEdge(Edge * edge) {

}

void Graph::DeleteEdge(size_t node_id1, size_t node_id2) {

}

void Graph::Draw() {

  ShaderRepository::UseColor(ColorRepository::Green());
  for (const auto &node:nodes_) {
    if (dynamic_cast<KeyFrameNode *>(node.second))
      node.second->Draw();
  }
  ShaderRepository::UseColor(ColorRepository::Blue());
  for (const auto &node:nodes_) {
    if (nullptr == dynamic_cast<KeyFrameNode *>(node.second))
      node.second->Draw();
  }
}

Node * Graph::GetNode(size_t node_id) const {
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