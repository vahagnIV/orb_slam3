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
    is_initialized_(false),
    windo_width_(width),
    windo_height_(height),
    window_name_(std::move(window_name)),
    window_(nullptr),
    thread_(nullptr),
    error_(),
    graph_(nullptr),
    position_is_predicted_(false),
    cancellation_token_(false),
    scale_(.005) {

}

void DrawerImpl::Initialize() {
  if (is_initialized_)
    return;
  is_initialized_ = true;

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
  graph_ = new Graph(10000);

}

DrawerImpl::~DrawerImpl() {
  Stop();
  delete graph_;
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

void DrawerImpl::WorkThread() {
  Initialize();
  ShaderRepository::Initialize();

  glGenBuffers(1, &position_vertex_buffer_id_);
  glGenBuffers(1, &predicted_position_vertex_buffer_id_);

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
      case messages::MessageType::POSITION_PREDICTED:
        PositionPredicted(Extract<messages::PositionPredicted>(message));
        break;
    }
    delete message;
  }
}

void DrawerImpl::PositionPredicted(messages::PositionPredicted * message) {
  position_is_predicted_ = true;
  glUseProgram(ShaderRepository::GetKeyFrameProgramId());
  glClear(GL_COLOR_BUFFER_BIT);

  auto invpose = message->pose.GetInversePose();
  TVector3D center = invpose.R * TVector3D{0, 0, 1} + invpose.T;
  TVector3D up = invpose.R * TVector3D{0, 1, 0} + invpose.T;


  glUseProgram(ShaderRepository::GetPositionProgramId());
  float buffer[18];
  CreatePositionRectangle(message->pose, buffer);
  // 1rst attribute buffer : vertices
  glEnableVertexAttribArray(0);
  glBindBuffer(GL_ARRAY_BUFFER, predicted_position_vertex_buffer_id_);
  glVertexAttribPointer(
      0,                  // attribute. No particular reason for 0, but must match the layout in the shader.
      3,                  // size
      GL_FLOAT,           // type
      GL_FALSE,           // normalized?
      0,                  // stride
      (void *) 0            // array buffer offset
  );
  glBufferData(GL_ARRAY_BUFFER, sizeof(buffer), buffer, GL_STATIC_DRAW);
//  glPointSize(2);

  glDisableVertexAttribArray(0);
}

void DrawerImpl::Stop() {
  cancellation_token_ = false;
  thread_->join();
  delete thread_;
  thread_ = nullptr;
}

void DrawerImpl::TrackingInfo(messages::TrackingInfo * message) {
  glUseProgram(ShaderRepository::GetKeyFrameProgramId());
  glClear(GL_COLOR_BUFFER_BIT);
  // Projection matrix : 45 Field of View, 4:3 ratio, display range : 0.1 unit <-> 100 units
  glm::mat4 Projection = glm::perspective(glm::radians(90.0f), 4.0f / 3.0f, 0.0001f, 100.0f);

  auto invpose = message->position.GetInversePose();
  TVector3D center = invpose.R * TVector3D{0, 0, 1} + invpose.T;
  TVector3D up = invpose.R * TVector3D{0, 1, 0} + invpose.T;
  glm::mat4 View = glm::lookAt(
      glm::vec3(invpose.T.x(), invpose.T.y(), invpose.T.z()), // Camera is at (4,3,-3), in World Space
      glm::vec3(center.x(), center.y(), center.z()), // and looks at the origin
      glm::vec3(0, 0, 1)  // Head is up (set to 0,-1,0 to look upside-down)
  );
  transformation_matrix_ = /*Projection **/ View;//* Model; // Remember, matrix multiplication is the other way around
  GLuint MatrixID = glGetUniformLocation(ShaderRepository::GetKeyFrameProgramId(), "MVP");
  GLuint ProjectionID = glGetUniformLocation(ShaderRepository::GetKeyFrameProgramId(), "Projection");

  const static glm::mat4 aMat4 = glm::mat4(1.0, 0.0, 0.0, 0.0,  // 1. column\n"
                                           0.0, -1.0, 0.0, 0.0,  // 2. column\n"
                                           0.0, 0.0, -1.0, 0.0,  // 3. column\n"
                                           0.0, 0.0, 0.0, 1.0); // 4. column"

  glm::mat4 mmm(message->position.R(0, 0), message->position.R(1, 0), message->position.R(2, 0), 0,
                message->position.R(0, 1), message->position.R(1, 1), message->position.R(2, 1), 0,
                message->position.R(0, 2), message->position.R(1, 2), message->position.R(2, 2), 0,
                message->position.T.x(), message->position.T.y(), message->position.T.z() + .5, 1);

  glUniformMatrix4fv(MatrixID, 1, GL_FALSE, &mmm[0][0]);
  glUniformMatrix4fv(ProjectionID, 1, GL_FALSE, &Projection[0][0]);

  GLuint ScaleID = glGetUniformLocation(ShaderRepository::GetKeyFrameProgramId(), "scale");
  glUniform1f(ScaleID, scale_);

  ShaderRepository::UseColor(ColorRepository::Green());

  glBindBuffer(GL_ARRAY_BUFFER, graph_->Buffer());
  glEnableVertexAttribArray(0);
  ShaderRepository::UseColor(ColorRepository::Blue());
  glVertexAttribPointer(
      0,                  // attribute. No particular reason for 0, but must match the layout in the shader.
      3,                  // size
      GL_FLOAT,           // type
      GL_FALSE,           // normalized?
      0,                  // stride
      (void *) 0            // array buffer offset
  );
  glDrawArrays(GL_POINTS, 0, graph_->Size()); //
  glDisableVertexAttribArray(0);

  if(position_is_predicted_) {
    glBindBuffer(GL_ARRAY_BUFFER, predicted_position_vertex_buffer_id_);
    ShaderRepository::UseColor(ColorRepository::Yellow());
    glEnableVertexAttribArray(0);
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
    position_is_predicted_ = false;
  }

  ShaderRepository::UseColor(ColorRepository::Red());

//  glUseProgram(ShaderRepository::GetPositionProgramId());
  float buffer[18];
  CreatePositionRectangle(message->position, buffer);
  // 1rst attribute buffer : vertices
  glEnableVertexAttribArray(0);
  glBindBuffer(GL_ARRAY_BUFFER, position_vertex_buffer_id_);
  glBufferData(GL_ARRAY_BUFFER, sizeof(buffer), buffer, GL_STATIC_DRAW);
  glPointSize(2);
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
  static const float rectangle_size = 0.025;
  static const TVector3D bottom_left_init{-rectangle_size, -rectangle_size, 0.};
  static const TVector3D top_left_init{-rectangle_size, rectangle_size, 0.};
  static const TVector3D top_right_init{rectangle_size, rectangle_size, 0.};
  static const TVector3D bottom_right_init{rectangle_size, -rectangle_size, 0.};

  geometry::Pose inverse = pose.GetInversePose();

  TVector3D bottom_left = inverse.Transform(bottom_left_init);
  TVector3D top_left = inverse.Transform(top_left_init);
  TVector3D top_right = inverse.Transform(top_right_init);
  TVector3D bottom_right = inverse.Transform(bottom_right_init);

#define COPY(dest, source) for(int i =0; i < 3; ++i) *((dest)+i)=(source)[i];

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
//  graph_.AddNode(kf_node);
}

void DrawerImpl::MapPointCreated(messages::MapPointCreated * message) {
  auto mp_node = new MapPointNode(message);
  graph_->AddMapPoint(mp_node);
}

void DrawerImpl::KeyFrameDeleted(messages::KeyFrameDeleted * message) {
  auto kf_node = graph_->GetKeyFrame(message->id);
  if (kf_node)
    vertex_buffers_.push(kf_node->vertex_buffer_id);
//  graph_.DeleteNode(message->id);
}

void DrawerImpl::MapPointDeleted(messages::MapPointDeleted * message) {
  graph_->DeleteMapPoint(message->id);
}

void DrawerImpl::KeyFramePositionUpdated(messages::KeyFramePositionUpdated * message) {
  auto node = graph_->GetKeyFrame(message->id);
  // TODO: This issue arises because monocular keyframe's constructor raises this event
  // TODO: before raising kf creted event.
  if (nullptr == node)
    return;
  CreatePositionRectangle(message->position, node->vertices);
  glBindBuffer(GL_ARRAY_BUFFER, node->vertex_buffer_id);
  glBufferData(GL_ARRAY_BUFFER, sizeof(node->vertices), node->vertices, GL_STATIC_DRAW);
}

void DrawerImpl::MapPointGeometryUpdated(messages::MapPointGeometryUpdated * message) {
  graph_->UpdateMapPoint(message);
}

}
}