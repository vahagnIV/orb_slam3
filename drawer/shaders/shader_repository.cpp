//
// Created by vahagn on 16/08/2021.
//

#include <vector>
#include <cstdio>
#include <cstdlib>

#include "shader_repository.h"
#include "vertex_shader.h"
#include "key_frame_fragment_shader.h"

namespace orb_slam3 {
namespace drawer {
bool ShaderRepository::is_initialized_ = false;
GLuint ShaderRepository::vertex_shader_id_ = 0;
GLuint ShaderRepository::keyframe_fragment_shader_id_ = 0;
GLuint ShaderRepository::keyframe_program_id_ = 0;

void ShaderRepository::InitializeVertexShader() {
  ShaderRepository & repo = Instance();
  vertex_shader_id_ = glCreateShader(GL_VERTEX_SHADER);
  glShaderSource(vertex_shader_id_, 1, &VERTEX_SHADER_SOURCE, NULL);
  glCompileShader(vertex_shader_id_);

  GLint result = GL_FALSE;
  int info_log_length;
  glGetShaderiv(vertex_shader_id_, GL_COMPILE_STATUS, &result);
  glGetShaderiv(vertex_shader_id_, GL_INFO_LOG_LENGTH, &info_log_length);
  if (info_log_length > 0) {
    std::vector<char> VertexShaderErrorMessage(info_log_length + 1);
    glGetShaderInfoLog(vertex_shader_id_, info_log_length, NULL, &VertexShaderErrorMessage[0]);
    printf("%s\n", &VertexShaderErrorMessage[0]);
    exit(1);
  }

}
void ShaderRepository::InitializeKeyFrameFragmentShader() {
  keyframe_fragment_shader_id_ = glCreateShader(GL_FRAGMENT_SHADER);
  glShaderSource(keyframe_fragment_shader_id_, 1, &KEYFRAME_FRAGMENT_SHADER_SOURCE, NULL);
  glCompileShader(keyframe_fragment_shader_id_);
  // Check Fragment Shader
  GLint result = GL_FALSE;
  int info_log_length;
  glGetShaderiv(keyframe_fragment_shader_id_, GL_COMPILE_STATUS, &result);
  glGetShaderiv(keyframe_fragment_shader_id_, GL_INFO_LOG_LENGTH, &info_log_length);
  if (info_log_length > 0) {
    std::vector<char> FragmentShaderErrorMessage(info_log_length + 1);
    glGetShaderInfoLog(keyframe_fragment_shader_id_, info_log_length, NULL, FragmentShaderErrorMessage.data());
    printf("%s\n", &FragmentShaderErrorMessage[0]);
    exit(1);
  }
}

void ShaderRepository::Initialize() {
  if (is_initialized_)
    return;
  is_initialized_ = true;

  InitializeVertexShader();
  InitializeKeyFrameFragmentShader();

  keyframe_program_id_ = glCreateProgram();
  glAttachShader(keyframe_program_id_, vertex_shader_id_);
  glAttachShader(keyframe_program_id_, keyframe_fragment_shader_id_);
  glLinkProgram(keyframe_program_id_);

  GLint result = GL_FALSE;
  int info_log_length;
  // Check the program
  glGetProgramiv(keyframe_program_id_, GL_LINK_STATUS, &result);
  glGetProgramiv(keyframe_program_id_, GL_INFO_LOG_LENGTH, &info_log_length);
  if (info_log_length > 0) {
    std::vector<char> ProgramErrorMessage(info_log_length + 1);
    glGetProgramInfoLog(keyframe_program_id_, info_log_length, NULL, &ProgramErrorMessage[0]);
    printf("%s\n", &ProgramErrorMessage[0]);
    exit(1);
  }
}

ShaderRepository::~ShaderRepository() {
  glDetachShader(keyframe_program_id_, vertex_shader_id_);
  glDetachShader(keyframe_program_id_, keyframe_fragment_shader_id_);

  glDeleteShader(vertex_shader_id_);
  glDeleteShader(keyframe_fragment_shader_id_);
}

GLuint ShaderRepository::GetKeyFrameProgramId() {
  return keyframe_program_id_;
}

}
}