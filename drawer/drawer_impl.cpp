//
// Created by vahagn on 13.06.21.
//

#include <messages/messages.h>
#include <shaders/shader_repository.h>
#include "drawer_impl.h"

namespace orb_slam3 {
namespace drawer {

DrawerImpl::DrawerImpl(size_t width, size_t height, std::string window_name) :
    windo_width_(width),
    windo_height_(height),
    window_name_(std::move(window_name)),
    window_(nullptr),
    thread_(nullptr),
    error_(),
    cancellation_token_(false) {

}

DrawerImpl::~DrawerImpl() {
  Stop();
}

void DrawerImpl::Start() {
  if (thread_ != nullptr)
    return;

  cancellation_token_ = true;
  thread_ = new std::thread(&DrawerImpl::WorkThread, this);
}

bool DrawerImpl::Started() const {
  return nullptr != thread_;
}

const std::string DrawerImpl::GetError() const {
  return error_;
}

void DrawerImpl::WorkThread() {
  glfwWindowHint(GLFW_SAMPLES, 4);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
  glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE); // To make MacOS happy; should not be needed
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

  window_ = glfwCreateWindow(windo_width_, windo_height_, window_name_.c_str(), NULL, NULL);
  if (nullptr == window_) {
    error_ = "Failed to open GLFW window. If you have an Intel GPU, they are not 3.3 compatible.";
    return;
  }
  glfwMakeContextCurrent(window_);

  GLenum glew_initialization_result = glewInit();
  if (glew_initialization_result != GLEW_OK) {
    std::cerr << glewGetErrorString(glew_initialization_result) << std::endl;
    assert(!"Glew was not initialized");
  }
  ShaderRepository::Initialize();

  glClearColor(1.0f, 1.0f, 1.4f, 0.0f);

  while (cancellation_token_) {
    glClear(GL_COLOR_BUFFER_BIT);
    glfwSwapBuffers(window_);
    glfwPollEvents();
    orb_slam3::messages::BaseMessage * message;
    if (!orb_slam3::messages::MessageProcessor::Instance().Dequeue(message)) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      continue;
    }

    switch (message->Type()) {
      case messages::MessageType::KEYFRAME_CREATED:KeyFrameCreated(Extract<messages::KeyFrameCreated>(message));
        break;
    }
  }
}

void DrawerImpl::Stop() {
  cancellation_token_ = false;
  thread_->join();
  delete thread_;
  thread_ = nullptr;
}

void DrawerImpl::TrackingInfo(messages::TrackingInfo * message) {
  // Projection matrix : 45 Field of View, 4:3 ratio, display range : 0.1 unit <-> 100 units
  glm::mat4 Projection = glm::perspective(glm::radians(45.0f), 4.0f / 3.0f, 0.1f, 100.0f);


  // Camera matrix
  glm::mat4 View = glm::lookAt(
      glm::vec3(4, 3, -3), // Camera is at (4,3,-3), in World Space
      glm::vec3(0, 0, 0), // and looks at the origin
      glm::vec3(0, 1, 0)  // Head is up (set to 0,-1,0 to look upside-down)
  );
  // Model matrix : an identity matrix (model will be at the origin)
  glm::mat4 Model;
  Convert(message->position, Model);
  // Our ModelViewProjection : multiplication of our 3 matrices
  transformation_matrix_ = Projection * View * Model; // Remember, matrix multiplication is the other way around
}

void DrawerImpl::Convert(const geometry::Pose & pose, glm::mat4 & out_mat) {
  for (int i = 0; i < pose.R.rows(); ++i) {
    for (int j = 0; j < pose.R.cols(); ++j) {
      out_mat[i][j] = pose.R(i, j);
    }
  }
  for (int i = 0; i < pose.T.size(); ++i) {
    out_mat[i][3] = pose.T[i];
  }

  for (int i = 0; i < pose.T.size(); ++i) {
    out_mat[3][i] = 0;
  }
  out_mat[3][3] = 1;

}

void DrawerImpl::KeyFrameCreated(messages::KeyFrameCreated * message) {

  static const TVector3D bottom_left_init{-0.5, -0.5, 0};
  static const TVector3D top_left_init{-0.5, 0.5, 0};
  static const TVector3D top_right_init{0.5, 0.5, 0};
  static const TVector3D bottom_right_init{0.5, -0.5, 0};

  geometry::Pose inverse = message->position.GetInversePose();

  TVector3D bottom_left = inverse.Transform(bottom_left_init);
  TVector3D top_left = inverse.Transform(top_left_init);
  TVector3D top_right = inverse.Transform(top_right_init);
  TVector3D bottom_right = inverse.Transform(bottom_right_init);
  KeyFrameDetails details;
  details.id = message->id;
  details.map_id = message->map_id;
  memcpy(details.vertices, bottom_left.data(), 3 * sizeof(TVector3D::Scalar));
  memcpy(details.vertices + 3, top_left.data(), 3 * sizeof(TVector3D::Scalar));
  memcpy(details.vertices + 6, bottom_right.data(), 3 * sizeof(TVector3D::Scalar));
  memcpy(details.vertices + 9, top_left.data(), 3 * sizeof(TVector3D::Scalar));
  memcpy(details.vertices + 12, top_right.data(), 3 * sizeof(TVector3D::Scalar));
  memcpy(details.vertices + 15, bottom_right.data(), 3 * sizeof(TVector3D::Scalar));

  glGenBuffers(1, &details.vertex_buffer_id);
  glBindBuffer(GL_ARRAY_BUFFER, details.vertex_buffer_id);
  glBufferData(GL_ARRAY_BUFFER, sizeof(details.vertices), details.vertices, GL_STATIC_DRAW);
  key_frames_[message->id] = details;
}

void DrawerImpl::KeyFrameUpdated() {

}

void DrawerImpl::Draw() {
  // Get a handle for our "MVP" uniform
  GLuint MatrixID = glGetUniformLocation(ShaderRepository::GetKeyFrameProgramId(), "MVP");

  glUseProgram(ShaderRepository::GetKeyFrameProgramId());

  glUniformMatrix4fv(MatrixID, 1, GL_FALSE, &transformation_matrix_[0][0]);
}

}
}