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

Graph::Graph(size_t size) : size_(size), current_carrette_(0) {
  glGenBuffers(1, &buffer_);
  glBindBuffer(GL_ARRAY_BUFFER, buffer_);
  glBufferData(GL_ARRAY_BUFFER, 3 * sizeof(MapPointNode::Scalar) * size_, nullptr, 0);
}

void Graph::AddMapPoint(MapPointNode * map_point_node) {
  if (current_carrette_ == size_)
    throw std::runtime_error("Imcreasing buffer size is not implemented yet");
  map_points_[map_point_node->Id()] = map_point_node;
  map_point_node->offset = current_carrette_ * 3 * sizeof(MapPointNode::Scalar);
  position_in_buffer_[map_point_node->Id()] = current_carrette_;
  ++current_carrette_;
  glBindBuffer(GL_ARRAY_BUFFER, buffer_);
  glBufferSubData(GL_ARRAY_BUFFER, buffer_, map_point_node->offset, map_point_node->coordinates);

}

void Graph::AddKeyFrame(KeyFrameNode * key_frame_node) {
  keyframes_[key_frame_node->Id()] = key_frame_node;
}

void Graph::DeleteMapPoint(size_t id) {

  auto it = map_points_.find(id);
  if (it == map_points_.end())
    return;

  MapPointNode * mp_to_delete = it->second;
  if (current_carrette_ > 1) {
    --current_carrette_;
    MapPointNode * mp_to_move = position_in_buffer_[current_carrette_];
    if(mp_to_move != it->second){

    }

    glCopyBufferSubData(GL_COPY_READ_BUFFER,
                        GL_COPY_WRITE_BUFFER,
                        (current_carrette_ - 1) * 3,
                        GLintptr
    writeOffset,
        GLsizeiptr
    size);
  }
  map_points_.erase(it);
}

void Graph::DeleteKeyFrame(size_t id) {
  auto it = keyframes_.find(id);
  if (it != keyframes_.end())
    keyframes_.erase(it);
}

MapPointNode * Graph::GetMapPoint(size_t id) {
  auto it = map_points_.find(id);
  if (it != map_points_.end())
    return it->second;
  return nullptr;
}

KeyFrameNode * Graph::GetKeyFrame(size_t id) {
  auto it = keyframes_.find(id);
  if (it != keyframes_.end())
    return it->second;
  return nullptr;
}

void Graph::AddEdge(Edge * edge) {

}

void Graph::DeleteEdge(size_t node_id1, size_t node_id2) {

}

void Graph::Draw() {

//  ShaderRepository::UseColor(ColorRepository::Green());
//  for (const auto & node: nodes_) {
//    if (dynamic_cast<KeyFrameNode *>(node.second))
//      node.second->Draw();
//  }
//  ShaderRepository::UseColor(ColorRepository::Blue());
//  for (const auto & node: nodes_) {
//    if (nullptr == dynamic_cast<KeyFrameNode *>(node.second))
//      node.second->Draw();
//  }
}

}
}