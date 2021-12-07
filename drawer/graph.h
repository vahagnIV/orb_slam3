//
// Created by vahagn on 16/08/2021.
//

#ifndef ORB_SLAM3_DRAWER_GRAPH_H_
#define ORB_SLAM3_DRAWER_GRAPH_H_

#include <unordered_map>

#include "node.h"
#include "edge.h"
#include "map_point_node.h"
#include "key_frame_node.h"

namespace orb_slam3 {
namespace drawer {

class Graph {
 public:
  Graph(size_t size);
  void AddNode( Node * node);
  void AddMapPoint(MapPointNode * map_point_node);
  void AddKeyFrame(KeyFrameNode * key_frame_node);
  void DeleteMapPoint(size_t id);
  void DeleteKeyFrame(size_t id);
  MapPointNode * GetMapPoint(size_t id);
  KeyFrameNode * GetKeyFrame(size_t);
  void DeleteNode(size_t node_id);
  void AddEdge(Edge * edge);
  void DeleteEdge(size_t node_id1, size_t node_id2);
  Node * GetNode(size_t node_id) const;
  bool NodeExists(size_t node_id);
  void Draw();
 private:
  size_t size_;
  GLuint buffer_;
  GLsizeiptr current_carrette_;
  std::unordered_map<size_t, MapPointNode *> map_points_;
  std::unordered_map<size_t , GLsizeiptr> position_in_buffer_;

  std::unordered_map<size_t, KeyFrameNode *> keyframes_;


};

}
}

#endif //ORB_SLAM3_DRAWER_GRAPH_H_
