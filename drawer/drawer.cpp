//
// Created by vahagn on 16/08/2021.
//

#include "drawer.h"
#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <iostream>

#include "shaders/shader_repository.h"

namespace orb_slam3 {
namespace drawer {

void GLAPIENTRY
MessageCallback(GLenum source,
                GLenum type,
                GLuint id,
                GLenum severity,
                GLsizei length,
                const GLchar * message,
                const void * userParam) {
  fprintf(stderr, "GL CALLBACK: %s type = 0x%x, severity = 0x%x, message = %s\n",
          (type == GL_DEBUG_TYPE_ERROR ? "** GL ERROR **" : ""),
          type, severity, message);
}

// During init, enable debug output


bool Initialize() {
  if (!glfwInit()) {
    fprintf(stderr, "Failed to initialize GLFW\n");
    getchar();
    return false;
  }

  return true;
}

void Terminate() {
  glfwTerminate();
}

}
}