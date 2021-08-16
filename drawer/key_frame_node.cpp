//
// Created by vahagn on 16.08.21.
//

#include "key_frame_node.h"
#include <GL/glew.h>
#include <GLFW/glfw3.h>

#include "shaders/shader_repository.h"

namespace orb_slam3 {
namespace drawer {
KeyFrameNode::KeyFrameNode(size_t id) : Node(id) {

}

void KeyFrameNode::Draw() const {
  glUseProgram(ShaderRepository::GetKeyFrameProgramId());

  // 1rst attribute buffer : vertices
  glEnableVertexAttribArray(0);
  glBindBuffer(GL_ARRAY_BUFFER, vertex_buffer_id);
  glVertexAttribPointer(
      0,                  // attribute. No particular reason for 0, but must match the layout in the shader.
      3,                  // size
      GL_FLOAT,           // type
      GL_FALSE,           // normalized?
      0,                  // stride
      (void *) 0            // array buffer offset
  );
  glDrawArrays(GL_TRIANGLES, 0, 6); // 3 indices starting at 0 -> 1 triangle

  glDisableVertexAttribArray(0);
}

}
}