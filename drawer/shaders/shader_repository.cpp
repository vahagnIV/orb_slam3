//
// Created by vahagn on 16/08/2021.
//

#include <vector>
#include <cstdio>
#include <cstdlib>
#include <cassert>

#include "shader_repository.h"
#include "vertex_shader.h"
#include "key_frame_fragment_shader.h"
#include "frame_fragment_shader.h"

namespace orb_slam3 {
namespace drawer {
bool ShaderRepository::is_initialized_ = false;
GLuint ShaderRepository::vertex_shader_id_ = 0;
GLuint ShaderRepository::keyframe_fragment_shader_id_ = 0;
GLuint ShaderRepository::frame_fragment_shader_id_ = 0;
GLuint ShaderRepository::keyframe_program_id_ = 0;
GLuint ShaderRepository::position_program_id_ = 0;
GLuint ShaderRepository::color_id_ = 0;

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
    assert(false);
    exit(1);
  }

}

GLuint ShaderRepository::InitializeFragmentShader(const char *source) {
  GLuint shader_id = glCreateShader(GL_FRAGMENT_SHADER);
  glShaderSource(shader_id, 1, &source, NULL);
  glCompileShader(shader_id);
  // Check Fragment Shader
  GLint result = GL_FALSE;
  int info_log_length;
  glGetShaderiv(shader_id, GL_COMPILE_STATUS, &result);
  glGetShaderiv(shader_id, GL_INFO_LOG_LENGTH, &info_log_length);
  if (info_log_length > 0) {
    std::vector<char> FragmentShaderErrorMessage(info_log_length + 1);
    glGetShaderInfoLog(shader_id, info_log_length, NULL, FragmentShaderErrorMessage.data());
    printf("%s\n", &FragmentShaderErrorMessage[0]);
    assert(false);
    exit(1);
  }
  return shader_id;
}

void ShaderRepository::Initialize() {
  if (is_initialized_)
    return;
  is_initialized_ = true;

  GLuint VertexArrayID;
  glGenVertexArrays(1, &VertexArrayID);
  glBindVertexArray(VertexArrayID);

  InitializeVertexShader();
  keyframe_fragment_shader_id_ = InitializeFragmentShader(KEYFRAME_FRAGMENT_SHADER_SOURCE);
  frame_fragment_shader_id_ = InitializeFragmentShader(FRAME_FRAGMENT_SHADER_SOURCE);
  keyframe_program_id_ = CreateProgram(vertex_shader_id_, keyframe_fragment_shader_id_);
  position_program_id_ = CreateProgram(vertex_shader_id_, frame_fragment_shader_id_);
  color_id_ = glGetUniformLocation(ShaderRepository::GetKeyFrameProgramId(), "col");
}

void ShaderRepository::UseColor(const float *color) {
  ShaderRepository & repo = Instance();
  glUniform3fv(repo.color_id_, 1, color);
}

GLuint ShaderRepository::GetPositionProgramId() {
  return position_program_id_;
}

GLuint ShaderRepository::CreateProgram(GLuint vertex_shader_id, GLuint fragment_shader_id) {
  GLuint program_id = glCreateProgram();
  glAttachShader(program_id, vertex_shader_id);
  glAttachShader(program_id, fragment_shader_id);
  glLinkProgram(program_id);

  GLint result = GL_FALSE;
  int info_log_length;
  // Check the program
  glGetProgramiv(program_id, GL_LINK_STATUS, &result);
  glGetProgramiv(program_id, GL_INFO_LOG_LENGTH, &info_log_length);
  if (info_log_length > 0) {
    std::vector<char> ProgramErrorMessage(info_log_length + 1);
    glGetProgramInfoLog(program_id, info_log_length, NULL, &ProgramErrorMessage[0]);
    printf("%s\n", &ProgramErrorMessage[0]);
    exit(1);
  }
  return program_id;
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