//
// Created by vahagn on 16.08.21.
//

#ifndef ORB_SLAM3_DRAWER_MAP_POINT_NODE_H_
#define ORB_SLAM3_DRAWER_MAP_POINT_NODE_H_

#include <GL/glew.h>
#include <GL/gl.h>

#include "node.h"
#include <typedefs.h>

namespace orb_slam3 {
namespace drawer {

class MapPointNode : public Node {
 public:
  MapPointNode(size_t id);
  void Draw() const override;
  GLuint buffer_id;

};

}
}

#endif //ORB_SLAM3_DRAWER_MAP_POINT_NODE_H_
