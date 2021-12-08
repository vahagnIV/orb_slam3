//
// Created by vahagn on 16/08/2021.
//

#include "drawer.h"
#include <GL/glew.h>
#include <GL/glut.h>
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

void error_callback(int error, const char * msg) {
  std::string s;
  s = " [" + std::to_string(error) + "] " + msg + '\n';
  std::cerr << s << std::endl;
}

bool Initialize() {
  if (!glfwInit()) {
    fprintf(stderr, "Failed to initialize GLFW\n");
    getchar();
    return false;
  }
  glFrontFace(GL_CCW);
  glfwSetErrorCallback(error_callback);
  glfwWindowHint(GLFW_SAMPLES, 4);
  glfwWindowHint(GLFW_DOUBLEBUFFER, GLFW_TRUE);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
  glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE); // To make MacOS happy; should not be needed
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
//  glutInitDisplayMode(GLUT_RGB);


  return true;
}

void Terminate() {
  glfwTerminate();
}

}
}