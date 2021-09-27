//
// Created by vahagn on 16.08.21.
//

#include "map_point_node.h"


namespace orb_slam3 {
namespace drawer {

MapPointNode::MapPointNode(size_t id) : Node(id) {

}

void MapPointNode::Draw() const {
  // 1rst attribute buffer : vertices
  glEnableVertexAttribArray(0);
  glBindBuffer(GL_ARRAY_BUFFER, buffer_id);
  glVertexAttribPointer(
      0,                  // attribute. No particular reason for 0, but must match the layout in the shader.
      3,                  // size
      GL_FLOAT,           // type
      GL_FALSE,           // normalized?
      0,                  // stride
      (void *) 0            // array buffer offset
      );
  glDrawArrays(GL_POINTS, 0, 1); // 3 indices starting at 0 -> 1 triangle

  glDisableVertexAttribArray(0);
}

}
}