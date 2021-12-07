//
// Created by vahagn on 16.08.21.
//

#include "map_point_node.h"
#include <shaders/shader_repository.h>
#include <shaders/color_repository.h>

namespace orb_slam3 {
namespace drawer {

MapPointNode::MapPointNode(messages::MapPointCreated * message) : Node(message->id) {
  coordinates[0] = message->position.x();
  coordinates[1] = message->position.y();
  coordinates[2] = message->position.z();

  normal[0] = 0;
  normal[1] = 0;
  normal[2] = 0;
}

void MapPointNode::Draw() const {
  // 1rst attribute buffer : vertices
  glEnableVertexAttribArray(0);
  glBindBuffer(GL_ARRAY_BUFFER, point_buffer_id);
  ShaderRepository::UseColor(ColorRepository::Blue());
  glVertexAttribPointer(
      0,                  // attribute. No particular reason for 0, but must match the layout in the shader.
      3,                  // size
      GL_FLOAT,           // type
      GL_FALSE,           // normalized?
      0,                  // stride
      (void *) 0            // array buffer offset
  );
  glDrawArrays(GL_POINTS, 0, 1); // 3 indices starting at 0 -> 1 triangle

  ShaderRepository::UseColor(ColorRepository::Pink());
  glBindBuffer(GL_ARRAY_BUFFER, normal_buffer_id);
  glVertexAttribPointer(
      0,                  // attribute. No particular reason for 0, but must match the layout in the shader.
      3,                  // size
      GL_FLOAT,           // type
      GL_FALSE,           // normalized?
      0,                  // stride
      (void *) 0            // array buffer offset
  );
  glDrawArrays(GL_LINES, 0, 2);
  glDisableVertexAttribArray(0);
}

}
}