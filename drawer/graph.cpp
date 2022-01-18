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
  glBufferData(GL_ARRAY_BUFFER, 3 * sizeof(MapPointNode::Scalar) * size_, nullptr, GL_STATIC_DRAW);
  map_points_ordered_.resize(size_, nullptr);
}

void Graph::UpdateMapPoint(messages::MapPointGeometryUpdated * update_message) {
  auto map_point_node = GetMapPoint(update_message->id);
  if(nullptr == map_point_node)
    return;
  glBindBuffer(GL_ARRAY_BUFFER, buffer_);
  map_point_node->coordinates[0] = update_message->position.x();
  map_point_node->coordinates[1] = update_message->position.y();
  map_point_node->coordinates[2] = update_message->position.z();
  glBufferSubData(GL_ARRAY_BUFFER, map_point_node->offset * 3 * sizeof(MapPointNode::Scalar),
                  sizeof(map_point_node->coordinates), map_point_node->coordinates);
}

void Graph::ReinstantiateBuffer() {
  size_ *= 2;
  glBindBuffer(GL_ARRAY_BUFFER, buffer_);
  glBufferData(GL_ARRAY_BUFFER, 3 * sizeof(MapPointNode::Scalar) * size_, nullptr, GL_STATIC_DRAW);
  for(auto map_point_node: map_points_ordered_){
    glBufferSubData(GL_ARRAY_BUFFER, map_point_node->offset * 3 * sizeof(MapPointNode::Scalar),
                    sizeof(map_point_node->coordinates), map_point_node->coordinates);
  }
  map_points_ordered_.resize(size_, nullptr);
}


void Graph::AddMapPoint(MapPointNode * map_point_node) {
  if (current_carrette_ == size_)
    ReinstantiateBuffer();
  map_points_[map_point_node->Id()] = map_point_node;
  map_point_node->offset = current_carrette_ ;
  map_points_ordered_[current_carrette_] = map_point_node;
  glBindBuffer(GL_ARRAY_BUFFER, buffer_);
  glBufferSubData(GL_ARRAY_BUFFER, map_point_node->offset * 3 * sizeof(MapPointNode::Scalar),
                  sizeof(map_point_node->coordinates), map_point_node->coordinates);
  ++current_carrette_;

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
    MapPointNode * mp_to_move = map_points_ordered_[current_carrette_];
    if(mp_to_move != mp_to_delete){

    }

    glCopyBufferSubData(GL_COPY_READ_BUFFER,
                        GL_COPY_WRITE_BUFFER,
                        mp_to_move->offset * 3 * sizeof(MapPointNode::Scalar),
                        mp_to_delete->offset * 3 * sizeof(MapPointNode::Scalar),
                        3 * sizeof(MapPointNode::Scalar));
    std::swap(map_points_ordered_[mp_to_move->offset], map_points_ordered_[mp_to_delete->offset]);
    mp_to_move->offset = mp_to_delete->offset;
  }
  delete mp_to_delete;
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