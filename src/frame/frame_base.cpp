//
// Created by vahagn on 11/29/20.
//

// == orb-slam3 ===
#include "frame_base.h"

namespace orb_slam3 {
namespace frame {

std::atomic_uint64_t FrameBase::frame_counter_(0);

void FrameBase::SetPosition(const geometry::Pose & pose) noexcept {
  pose_ = pose;
  inverse_pose_.R = pose_.R.transpose();
  inverse_pose_.T = -inverse_pose_.R * pose_.T;
}

void FrameBase::SetPosition(const geometry::Quaternion & pose) noexcept {
  pose_.R = pose.rotation().toRotationMatrix();
  pose_.T = pose.translation();
  inverse_pose_.R = pose_.R.transpose();
  inverse_pose_.T = -inverse_pose_.R * pose_.T;
}

void FrameBase::InitializeIdentity() noexcept {
  pose_.R.setIdentity();
  pose_.T.setZero();
  inverse_pose_.R.setIdentity();
  inverse_pose_.T.setZero();
}

void FrameBase::FixedFrames(const std::unordered_set<map::MapPoint *> & map_points,
                            const std::unordered_set<FrameBase *> & frames,
                            std::unordered_set<FrameBase *> & out_fixed_frames) {
  for (auto mp: map_points) {
    for (auto obs: mp->Observations()) {
      if (obs.first->IsInitial() || frames.find(obs.first) == frames.end())
        out_fixed_frames.insert(obs.first);
    }
  }

  if (out_fixed_frames.size() < 2) {
    FrameBase * earliest = nullptr, * second_earilest = nullptr;
    for (auto frame: frames) {
      if (nullptr == earliest || earliest->Id() < frame->Id()) {
        earliest = frame;
      } else if (nullptr == second_earilest || second_earilest->Id() < frame->Id()) {
        second_earilest = frame;
      }
    }

    if (nullptr == earliest)
      return; // TODO: log
    out_fixed_frames.insert(earliest);
    if (out_fixed_frames.size() < 2)
      out_fixed_frames.insert(second_earilest);
  }
}

FrameBase::~FrameBase() {
  for(auto ob: recent_observations_){
    delete ob;
  }
}

}
}  // namespace orb_slam3
