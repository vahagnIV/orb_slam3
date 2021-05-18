//
// Created by vahagn on 16/03/2021.
//

#include "local_mapper.h"
#include "frame/visible_map_point.h"
#include "frame/key_frame.h"
#include "logging.h"
#include "optimization/bundle_adjustment.h"

namespace orb_slam3 {

LocalMapper::LocalMapper(map::Atlas * atlas) : PositionObserver(), atlas_(atlas), thread_(nullptr) {}

LocalMapper::~LocalMapper() {
  Stop();
}

void LocalMapper::Run() {
  while (!cancelled_) {
    UpdateMessage message;
    GetUpdateQueue().wait_dequeue(message);
    switch (message.type) {
      case PositionMessageType::Final:continue;
      case PositionMessageType::Initial:continue;
      case PositionMessageType::Update: {
        std::cout << message.frame->Id() << std::endl;
//        CreateNewMapPoints(message.frame);
      }
    }
  }
}

void LocalMapper::Start() {
  if (thread_ != nullptr)
    return;
  cancelled_ = false;
  thread_ = new std::thread(&LocalMapper::Run, this);
}

void LocalMapper::Stop() {
  if (nullptr == thread_)
    return;
  cancelled_ = true;
  thread_->join();
  delete thread_;
  thread_ = nullptr;
}

void LocalMapper::MapPointCulling(frame::KeyFrame * keyframe) {
  unsigned erased = recently_added_map_points_.size();
  for (auto mp_it = recently_added_map_points_.begin(); mp_it != recently_added_map_points_.end(); ) {
    map::MapPoint * mp = *mp_it;
    if (mp->IsBad()) {
      mp_it = recently_added_map_points_.erase(mp_it);
    } else if (static_cast<precision_t>(mp->GetFound()) / mp->GetVisible() < 0.25) {
      mp->SetBad();
      mp_it = recently_added_map_points_.erase(mp_it);
    } else if (keyframe->Id() > mp->GetFirstObservedFrameId() && keyframe->Id() - mp->GetFirstObservedFrameId() >= 2
        && mp->GetObservationCount() < keyframe->GetSensorConstants()->max_mp_disappearance_count) {
      mp->SetBad();
      mp_it = recently_added_map_points_.erase(mp_it);
    } else if (keyframe->Id() > mp->GetFirstObservedFrameId() && keyframe->Id() - mp->GetFirstObservedFrameId() >= 2) {
      mp_it = recently_added_map_points_.erase(mp_it);
    } else {
      --erased;
      ++mp_it;
    }
  }

  logging::RetrieveLogger()->debug("LM: erased {} mps", erased);
}

void LocalMapper::ProcessNewKeyFrame(frame::KeyFrame * keyframe) {
  frame::KeyFrame::MapPointSet map_points;
  keyframe->ComputeBow();
  keyframe->ListMapPoints(map_points);
  for (auto mp: map_points) {
    if (mp->IsBad())
      continue;

    if (!mp->IsInKeyFrame(keyframe)) {
      // TODO: find cases when this could happen
      assert(false);
    } else
      recently_added_map_points_.insert(mp);
  }
  keyframe->GetCovisibilityGraph().Update();
  // TODO: Add to Atlas

}

void LocalMapper::CreateNewMapPoints(frame::KeyFrame * key_frame) {
  auto covisible_frames =
      key_frame->GetCovisibilityGraph().GetCovisibleKeyFrames(key_frame->GetSensorConstants()->number_of_keyframe_to_search_lm);

  for (auto neighbour_keyframe: covisible_frames) {
    key_frame->CreateNewMapPoints(neighbour_keyframe);
  }
}

void LocalMapper::Optimize(frame::KeyFrame * frame) {
  std::unordered_set<frame::KeyFrame *> local_keyframes = frame->GetCovisibilityGraph().GetCovisibleKeyFrames(),
      fixed_keyframes;
  frame::KeyFrame::MapPointSet local_map_points;
  for (auto keyframe: local_keyframes) keyframe->ListMapPoints(local_map_points);
  local_keyframes.insert(frame);
  FilterFixedKeyFames(local_keyframes, local_map_points, fixed_keyframes);
  if (fixed_keyframes.empty())
    return;
  optimization::LocalBundleAdjustment(local_keyframes, fixed_keyframes, local_map_points);

}

void LocalMapper::FilterFixedKeyFames(unordered_set<frame::KeyFrame *> & local_keyframes,
                                      frame::KeyFrame::MapPointSet & local_map_points,
                                      unordered_set<frame::KeyFrame *> & out_fixed) const {
  unsigned number_of_fixed = 0;
  for (auto keyframe: local_keyframes) {
    if (keyframe->IsInitial())
      ++number_of_fixed;
  }

  for (auto map_point: local_map_points) {
    for (auto observation: map_point->Observations()) {
      if (local_keyframes.find(observation.first) == local_keyframes.end())
        out_fixed.insert(observation.first);
    }
  }
  number_of_fixed += out_fixed.size();

  if (number_of_fixed < 2) {
    frame::KeyFrame * earliest_keyframe = nullptr, * second_eraliset_keyframe = nullptr;
    for (auto frame: local_keyframes) {
      if (nullptr == earliest_keyframe || frame->Id() < earliest_keyframe->Id())
        earliest_keyframe = frame;
      else if (nullptr == second_eraliset_keyframe || frame->Id() < second_eraliset_keyframe->Id())
        second_eraliset_keyframe = frame;
    }
    if (nullptr != earliest_keyframe) {
      ++number_of_fixed;
      out_fixed.insert(earliest_keyframe);
      local_keyframes.erase(earliest_keyframe);
    }
    if (number_of_fixed < 2 && nullptr != second_eraliset_keyframe) {
      ++number_of_fixed;
      out_fixed.insert(second_eraliset_keyframe);
      local_keyframes.erase(second_eraliset_keyframe);
    }
  }
}

bool LocalMapper::CheckNewKeyFrames() const {
  bool new_keyframes =  GetUpdateQueue().size_approx() > 0;
  return new_keyframes;
}

void LocalMapper::RunIteration() {
  UpdateMessage message;
  while (GetUpdateQueue().try_dequeue(message)) {
    message.frame->GetCovisibilityGraph().Update();
    ProcessNewKeyFrame(message.frame);
    MapPointCulling(message.frame);
    CreateNewMapPoints(message.frame);
    if (!CheckNewKeyFrames()) {
      Optimize(message.frame);
    }
  }

}

}