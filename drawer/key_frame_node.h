//
// Created by vahagn on 16.08.21.
//

#ifndef ORB_SLAM3_DRAWER_KEY_FRAME_NODE_H_
#define ORB_SLAM3_DRAWER_KEY_FRAME_NODE_H_

#include <GL/glew.h>
#include <GL/gl.h>

#include "node.h"
#include <typedefs.h>

namespace orb_slam3 {
namespace drawer {

class KeyFrameNode : public Node {
 public:
  KeyFrameNode(size_t id);
  void Draw() const override;
  size_t map_id;
  /*TVector3D::Scalar*/float vertices[18];
  GLuint vertex_buffer_id;
};

}
}
#endif //ORB_SLAM3_DRAWER_KEY_FRAME_NODE_H_
