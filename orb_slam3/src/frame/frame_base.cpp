//
// Created by vahagn on 11/29/20.
//

// == orb-slam3 ===
#include "frame/frame_base.h"

namespace orb_slam3 {
namespace frame {

void FrameBase::SetPosition(const geometry::Pose &pose) noexcept {
  pose_.setEstimate(pose.estimate());
}

void FrameBase::InitializeIdentity() noexcept {
  pose_.setToOriginImpl();
}

g2o::VertexSE3Expmap *FrameBase::CreatePoseVertex() const {
  auto pose_vertex = new g2o::VertexSE3Expmap();
  pose_vertex->setEstimate(pose_.estimate());
  pose_vertex->setId(Id());
  pose_vertex->setFixed(false);
  return pose_vertex;
}

const map::MapPoint *FrameBase::MapPoint(size_t id) const {
  auto map_point = map_points_.find(id);
  return map_point == map_points_.end() ? nullptr : map_point->second;
}

FrameBase::~FrameBase() {
  for(auto & mp_id: map_points_){
    mp_id.second->EraseObservation(this);
    if(mp_id.second->Observations().empty())
      delete mp_id.second;
  }
}

}
}  // namespace orb_slam3