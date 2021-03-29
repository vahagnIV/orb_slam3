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

void FrameBase::ListMapPoints(const unordered_set<FrameBase *> & frames, set<map::MapPoint *> & out_map_points) {
  out_map_points.clear();
  for (auto frame: frames) {
    for (auto mp: frame->MapPoints()) {
      if (mp.second->IsValid())
        out_map_points.insert(mp.second);
    }
  }
}

void FrameBase::FixedFrames(const set<map::MapPoint *> & map_points,
                            const std::unordered_set<FrameBase *> & frames,
                            unordered_set<FrameBase *> & out_fixed_frames) {
  for (auto mp: map_points) {
    for (auto obs: mp->Observations()) {
      if (frames.find(obs.first) == frames.end())
        out_fixed_frames.insert(obs.first);
    }
  }

  if (out_fixed_frames.size() < 2) {
    FrameBase *earliest = nullptr, *second_earilest = nullptr;
    for (auto frame: frames) {
      if (nullptr == earliest || earliest->Id() < frame->Id()) {
        earliest = frame;
      } else if (nullptr == second_earilest || second_earilest->Id() < frame->Id()) {
        second_earilest = frame;
      }
    }

    out_fixed_frames.insert(earliest);
    if (out_fixed_frames.size() < 2)
      out_fixed_frames.insert(second_earilest);
  }
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