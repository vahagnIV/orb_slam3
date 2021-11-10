//
// Created by vahagn on 13.06.21.
//
// ====== stl ===========
#include <thread>

// ===== open-gl ========
#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

// ===== orb-slam3 ======
#include <geometry/pose.h>
#include <messages/messages.h>

// ===== drawer ======
#include "graph.h"

#ifndef ORB_SLAM3_DRAWER_IMPL_DRAWER_H_
#define ORB_SLAM3_DRAWER_IMPL_DRAWER_H_

namespace orb_slam3 {
namespace drawer {

class DrawerImpl {
 public:
  DrawerImpl(size_t width, size_t height, std::string window_name = "Orb slam 3");

  ~DrawerImpl();

 public:
  void Start();
  void Stop();
  bool Started() const;
  const std::string & GetError() const;

 public:
  void KeyFrameCreated(messages::KeyFrameCreated *message);
  void KeyFrameDeleted(messages::KeyFrameDeleted *message);
  void MapPointCreated(messages::MapPointCreated *message);
  void MapPointDeleted(messages::MapPointDeleted *message);
  void KeyFramePositionUpdated(messages::KeyFramePositionUpdated *message);
  void MapPointGeometryUpdated(messages::MapPointGeometryUpdated *message);
  void TrackingInfo(messages::TrackingInfo *message);

 private:
  void WorkThread();

  template<typename T>
  T * Extract(messages::BaseMessage * message) {
    auto converted_message = dynamic_cast<T *>(message);
    assert(nullptr != converted_message);
    return converted_message;
  }

  static void Convert(const geometry::Pose & pose, glm::mat4 & out_mat);
  void CreatePositionRectangle(const geometry::Pose &pose, float result[]) const;

 private:
  size_t windo_width_;
  size_t windo_height_;
  std::string window_name_;
  GLFWwindow * window_;
  std::thread * thread_;
  std::string error_;
  bool cancellation_token_;
  Graph graph_;
  glm::mat4 transformation_matrix_;
  GLuint position_vertex_buffer_id_;
  precision_t scale_;

};

}
}

#endif //ORB_SLAM3_DRAWER_DRAWER_H_
