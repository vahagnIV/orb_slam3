//
// Created by vahagn on 13.06.21.
//

#include <messages/messages.h>
#include <shaders/shader_repository.h>
#include <shaders/color_repository.h>
#include "drawer_impl.h"
#include "key_frame_node.h"
#include "map_point_node.h"

namespace orb_slam3 {
namespace drawer {

DrawerImpl::DrawerImpl(size_t width, size_t height, std::string window_name) :
    windo_width_(width),
    windo_height_(height),
    window_name_(std::move(window_name)),
    window_(nullptr),
    thread_(nullptr),
    error_(),
    cancellation_token_(false),
    scale_(4.) {

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

const std::string & DrawerImpl::GetError() const {
  return error_;
}

void error_callback(int error, const char * msg) {
  std::string s;
  s = " [" + std::to_string(error) + "] " + msg + '\n';
  std::cerr << s << std::endl;
}

void DrawerImpl::WorkThread() {
  glfwSetErrorCallback(error_callback);
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

  glGenBuffers(1, &position_vertex_buffer_id_);

  glClearColor(1.0f, 1.0f, 1.0f, 0.0f);

  while (cancellation_token_) {
    orb_slam3::messages::BaseMessage * message;
    if (!orb_slam3::messages::MessageProcessor::Instance().Dequeue(message)) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      continue;
    }

    switch (message->Type()) {
      case messages::MessageType::KEYFRAME_CREATED:
        KeyFrameCreated(Extract<messages::KeyFrameCreated>(message));
        break;
      case messages::MessageType::KEYFRAME_DELETED:
        KeyFrameDeleted(Extract<messages::KeyFrameDeleted>(message));
        break;
      case messages::MessageType::TRACKING_INFO:
        TrackingInfo(Extract<messages::TrackingInfo>(message));
        break;
      case messages::MessageType::MAP_POINT_CREATED:
        MapPointCreated(Extract<messages::MapPointCreated>(message));
        break;
      case messages::MessageType::MAP_POINT_DELETED:
        MapPointDeleted(Extract<messages::MapPointDeleted>(message));
        break;
      case messages::MessageType::KEYFRAME_POSITION_UPDATED:
        KeyFramePositionUpdated(Extract<messages::KeyFramePositionUpdated>(message));
        break;
      case messages::MessageType::MAP_POINT_GEOMETRY_UPDATED:
        MapPointGeometryUpdated(Extract<messages::MapPointGeometryUpdated>(message));
        break;
    }
    delete message;
  }
}

void DrawerImpl::Stop() {
  cancellation_token_ = false;
  thread_->join();
  delete thread_;
  thread_ = nullptr;
}

void DrawerImpl::TrackingInfo(messages::TrackingInfo * message) {
  glClear(GL_COLOR_BUFFER_BIT);
  // Projection matrix : 45 Field of View, 4:3 ratio, display range : 0.1 unit <-> 100 units
  glm::mat4 Projection = glm::perspective(glm::radians(45.0f), 4.0f / 3.0f, 0.1f, 100.0f);


  // Camera matrix
  glm::mat4 View = glm::lookAt(
      glm::vec3(4, 3, -3), // Camera is at (4,3,-3), in World Space
      glm::vec3(2, 0, 0), // and looks at the origin
      glm::vec3(0, 1, 0)  // Head is up (set to 0,-1,0 to look upside-down)
  );
  // Model matrix : an identity matrix (model will be at the origin)
  glm::mat4 Model = glm::translate(glm::mat4(1.0), glm::vec3(-2, 0, 0));
//  Convert(message->position.GetInversePose(), Model);
  // Our ModelViewProjection : multiplication of our 3 matrices
  transformation_matrix_ = Projection * View * Model; // Remember, matrix multiplication is the other way around
  GLuint MatrixID = glGetUniformLocation(ShaderRepository::GetKeyFrameProgramId(), "MVP");
  glUniformMatrix4fv(MatrixID, 1, GL_FALSE, &transformation_matrix_[0][0]);

  ShaderRepository::UseColor(ColorRepository::Green());

  graph_.Draw();
  ShaderRepository::UseColor(ColorRepository::Red());

//  glUseProgram(ShaderRepository::GetPositionProgramId());
  float buffer[18];
  CreatePositionRectangle(message->position, buffer);
  // 1rst attribute buffer : vertices
  glEnableVertexAttribArray(0);
  glBindBuffer(GL_ARRAY_BUFFER, position_vertex_buffer_id_);
  glBufferData(GL_ARRAY_BUFFER, sizeof(buffer), buffer, GL_STATIC_DRAW);
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

  glfwSwapBuffers(window_);
  glfwPollEvents();
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

void DrawerImpl::CreatePositionRectangle(const geometry::Pose & pose, float result[]) const {
  static const float rectangle_size = 0.15;
  static const TVector3D bottom_left_init{-rectangle_size, -rectangle_size, 0};
  static const TVector3D top_left_init{-rectangle_size, rectangle_size, 0};
  static const TVector3D top_right_init{rectangle_size, rectangle_size, 0};
  static const TVector3D bottom_right_init{rectangle_size, -rectangle_size, 0};

  geometry::Pose inverse = pose.GetInversePose();

  TVector3D bottom_left = inverse.Transform(bottom_left_init);
  TVector3D top_left = inverse.Transform(top_left_init);
  TVector3D top_right = inverse.Transform(top_right_init);
  TVector3D bottom_right = inverse.Transform(bottom_right_init);

#define COPY(dest, source) for(int i =0; i < 3; ++i) *((dest)+i)=(source)[i] / scale_;

  COPY(result, bottom_left.data());
  COPY(result + 3, top_left.data());
  COPY(result + 6, bottom_right.data());
  COPY(result + 9, bottom_right.data());
  COPY(result + 12, top_left.data());
  COPY(result + 15, top_right.data());
}

void DrawerImpl::KeyFrameCreated(messages::KeyFrameCreated * message) {

  KeyFrameNode * kf_node = new KeyFrameNode(message->id);;
  kf_node->map_id = message->map_id;
  CreatePositionRectangle(message->position, kf_node->vertices);

  // TODO: Use memory manager to reuse this memory
  if (vertex_buffers_.empty()) {
    glGenBuffers(1, &kf_node->vertex_buffer_id);
  } else {
    kf_node->vertex_buffer_id = vertex_buffers_.top();
    vertex_buffers_.pop();
  }
  glBindBuffer(GL_ARRAY_BUFFER, kf_node->vertex_buffer_id);
  glBufferData(GL_ARRAY_BUFFER, sizeof(kf_node->vertices), kf_node->vertices, GL_STATIC_DRAW);
  graph_.AddNode(kf_node);
}

void DrawerImpl::MapPointCreated(messages::MapPointCreated * message) {
  auto mp_node = new MapPointNode(message->id);
  if (point_buffers_.empty()) {
    glGenBuffers(1, &mp_node->point_buffer_id);
  } else {
    mp_node->point_buffer_id = point_buffers_.top();
    point_buffers_.pop();
  }
  float buffer[] = {static_cast<float>(message->position.x() / scale_),
                    static_cast<float>(message->position.y() / scale_),
                    static_cast<float>(message->position.z() / scale_)};

  glBindBuffer(GL_ARRAY_BUFFER, mp_node->point_buffer_id);
  glBufferData(GL_ARRAY_BUFFER, sizeof(buffer), buffer, GL_STATIC_DRAW);

  if (line_buffers_.empty()) {
    glGenBuffers(1, &mp_node->normal_buffer_id);
  } else {
    mp_node->normal_buffer_id = line_buffers_.top();
    line_buffers_.pop();
  }

  float normal_buffer[] = {static_cast<float>(message->position.x() / scale_),
                           static_cast<float>(message->position.y() / scale_),
                           static_cast<float>(message->position.z() / scale_),
                           static_cast<float>(message->position.x() / scale_),
                           static_cast<float>(message->position.y() / scale_),
                           static_cast<float>(message->position.z() / scale_)};

  glBindBuffer(GL_ARRAY_BUFFER, mp_node->normal_buffer_id);
  glBufferData(GL_ARRAY_BUFFER, sizeof(normal_buffer), normal_buffer, GL_STATIC_DRAW);

  graph_.AddNode(mp_node);

}

void DrawerImpl::KeyFrameDeleted(messages::KeyFrameDeleted * message) {
  auto kf_node = dynamic_cast<KeyFrameNode *>( graph_.GetNode(message->id));
  if (kf_node)
    vertex_buffers_.push(kf_node->vertex_buffer_id);
  graph_.DeleteNode(message->id);
}

void DrawerImpl::MapPointDeleted(messages::MapPointDeleted * message) {
  auto mp_node = dynamic_cast<MapPointNode *>( graph_.GetNode(message->id));
  if (mp_node)
    point_buffers_.push(mp_node->point_buffer_id);
  graph_.DeleteNode(message->id);
}

void DrawerImpl::KeyFramePositionUpdated(messages::KeyFramePositionUpdated * message) {
  auto node = dynamic_cast<KeyFrameNode *>(graph_.GetNode(message->id));
  // TODO: This issue arises because monocular keyframe's constructor raises this event
  // TODO: before raising kf creted event.
  if (nullptr == node)
    return;
  CreatePositionRectangle(message->position, node->vertices);
  glBindBuffer(GL_ARRAY_BUFFER, node->vertex_buffer_id);
  glBufferData(GL_ARRAY_BUFFER, sizeof(node->vertices), node->vertices, GL_STATIC_DRAW);
}

void DrawerImpl::MapPointGeometryUpdated(messages::MapPointGeometryUpdated * message) {
  auto node = dynamic_cast<MapPointNode *>(graph_.GetNode(message->id));
  if (nullptr == node)
    return;
  float buffer[] = {static_cast<float>(message->position.x() / scale_),
                    static_cast<float>(message->position.y() / scale_),
                    static_cast<float>(message->position.z() / scale_)};

  glBindBuffer(GL_ARRAY_BUFFER, node->point_buffer_id);
  glBufferData(GL_ARRAY_BUFFER, sizeof(buffer), buffer, GL_STATIC_DRAW);

  float normal_buffer[6] = {static_cast<float>(message->position.x() / scale_),
                           static_cast<float>(message->position.y() / scale_),
                           static_cast<float>(message->position.z() / scale_),
                           static_cast<float>(message->normal.x() / scale_),
                           static_cast<float>(message->normal.y() / scale_),
                           static_cast<float>(message->normal.z() / scale_)};

  glBindBuffer(GL_ARRAY_BUFFER, node->normal_buffer_id);
//  glVertexPointer(2, GL_FLOAT, 0, normal_buffer);
  glBufferData(GL_ARRAY_BUFFER, sizeof(normal_buffer), normal_buffer, GL_STATIC_DRAW);
}

}
}