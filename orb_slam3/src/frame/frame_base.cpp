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

void FrameBase::FixedFrames(const std::unordered_set<map::MapPoint *> & map_points,
                            const std::unordered_set<FrameBase *> & frames,
                            std::unordered_set<FrameBase *> & out_fixed_frames) {
  for (auto mp: map_points) {
    for (auto obs: mp->Observations()) {
      if (frames.find(obs.first) == frames.end() || obs.first->IsInitial())
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
  /*for (auto & mp_id: map_points_) {
    mp_id.second->EraseObservation(this);
    if (mp_id.second->Observations().empty())
      delete mp_id.second;
  }*/
}
FrameBase *FrameBase::ListLocalKeyFrames(std::unordered_set<frame::FrameBase *> & out_frames) const {
  std::unordered_map<frame::FrameBase *, unsigned> key_frame_counter;
//  for (auto & map_point: MapPoints()) {
//    for (auto observation: map_point.second->Observations()) {
//      ++key_frame_counter[observation.first];
//    }
//  }

  FrameBase *max_covisible_key_frame = nullptr;
  unsigned max_count = 0;
  for (auto & kf_count: key_frame_counter) {
    out_frames.insert(kf_count.first);
    if (max_count < kf_count.second) {
      max_count = kf_count.second;
      max_covisible_key_frame = kf_count.first;
    }
  }
  return max_covisible_key_frame;

  // TODO: list from children, list from parent
}

void FrameBase::ListAllMapPoints(const unordered_set<FrameBase *> & frames,
                                 unordered_set<map::MapPoint *> & out_map_points) {
  for (auto & frame: frames) {
    frame->ListMapPoints(out_map_points);
  }
}

}
}  // namespace orb_slam3