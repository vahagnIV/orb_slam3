//
// Created by vahagn on 16.08.21.
//

#ifndef ORB_SLAM3_DRAWER_MAP_POINT_NODE_H_
#define ORB_SLAM3_DRAWER_MAP_POINT_NODE_H_

#include <GL/glew.h>
#include <GL/gl.h>

#include "node.h"
#include <typedefs.h>
#include <messages/map_point_created.h>

namespace orb_slam3 {
namespace drawer {

class MapPointNode : public Node {
 public:
  typedef float Scalar;
  MapPointNode(messages::MapPointCreated * message);
  void Draw() const override;
  GLuint point_buffer_id;
  GLuint normal_buffer_id;
  GLsizeiptr offset;
  float coordinates[3];
  float normal[3];


};

}
}

#endif //ORB_SLAM3_DRAWER_MAP_POINT_NODE_H_
