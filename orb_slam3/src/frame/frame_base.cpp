//
// Created by vahagn on 11/29/20.
//

// == orb-slam3 ===
#include "frame/frame_base.h"

namespace orb_slam3 {
namespace frame {

void FrameBase::SetPosition(const geometry::Pose & pose) noexcept {
  pose_ = pose;
}

void FrameBase::SetPosition(const geometry::Quaternion & pose) noexcept {
  pose_.R = pose.rotation().toRotationMatrix();
  pose_.T = pose.translation();
}

void FrameBase::InitializeIdentity() noexcept {
  pose_.R.setIdentity();
  pose_.T.setZero();
}

g2o::VertexSE3Expmap *FrameBase::CreatePoseVertex() const {
  auto pose_vertex = new g2o::VertexSE3Expmap();
  pose_vertex->setEstimate(geometry::Quaternion(pose_.R, pose_.T));
  pose_vertex->setId(Id());
  pose_vertex->setFixed(false);
  return pose_vertex;
}

const map::MapPoint *FrameBase::MapPoint(size_t id) const {
  auto map_point = map_points_.find(id);
  return map_point == map_points_.end() ? nullptr : map_point->second;
}

FrameBase::~FrameBase() {
  for (auto & mp_id: map_points_) {
    mp_id.second->EraseObservation(this);
    if (mp_id.second->Observations().empty())
      delete mp_id.second;
  }
}

}
}  // namespace orb_slam3