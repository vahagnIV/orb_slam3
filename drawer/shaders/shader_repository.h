//
// Created by vahagn on 16/08/2021.
//

#ifndef ORB_SLAM3_DRAWER_SHADERS_SHADER_REPOSITORY_H_
#define ORB_SLAM3_DRAWER_SHADERS_SHADER_REPOSITORY_H_

#include <GL/glew.h>

namespace orb_slam3 {
namespace drawer {

class ShaderRepository {
 public:
  static void Initialize();
  static GLuint GetKeyFrameProgramId();
 private:
  static ShaderRepository & Instance() {
    static ShaderRepository repository;
    return repository;
  }
  ShaderRepository() = default;
  static void InitializeVertexShader();
  static void InitializeKeyFrameFragmentShader();

 private:
  static bool is_initialized_;
  static GLuint vertex_shader_id_;
  static GLuint keyframe_fragment_shader_id_;
  static GLuint keyframe_program_id_;
  ~ShaderRepository();

};

}
}

#endif //ORB_SLAM3_DRAWER_SHADERS_SHADER_REPOSITORY_H_
