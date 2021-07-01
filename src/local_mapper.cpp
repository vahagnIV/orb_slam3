//
// Created by vahagn on 16/03/2021.
//

#include "local_mapper.h"
#include "frame/map_point_visibility_params.h"
#include "frame/key_frame.h"
#include "logging.h"
#include "optimization/bundle_adjustment.h"
#include <map/atlas.h>

namespace orb_slam3 {

LocalMapper::LocalMapper(map::Atlas * atlas) : PositionObserver(), atlas_(atlas), thread_(nullptr) {}

LocalMapper::~LocalMapper() {
  Stop();
}

void LocalMapper::Run() {
  while (!cancelled_) {
  accept_key_frames_ = false;
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
  accept_key_frames_ = true;
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
//  recently_added_map_points_.clear();
  size_t erased = recently_added_map_points_.size();
  for (auto mp_it = recently_added_map_points_.begin(); mp_it != recently_added_map_points_.end();) {
    map::MapPoint * mp = *mp_it;
    if (mp->IsBad()) {
      mp_it = recently_added_map_points_.erase(mp_it);
    } else if (static_cast<precision_t>(mp->GetFound()) / mp->GetVisible() < 0.25) {
      mp->SetBad();
      mp_it = recently_added_map_points_.erase(mp_it);
    } else if (keyframe->Id() > mp->GetFirstObservedFrameId() && keyframe->Id() - mp->GetFirstObservedFrameId() >= 2
        && mp->GetObservationCount() < keyframe->GetSensorConstants()->min_mp_disappearance_count) {
      mp->SetBad();
      mp_it = recently_added_map_points_.erase(mp_it);
    } else if (keyframe->Id() > mp->GetFirstObservedFrameId() && keyframe->Id() - mp->GetFirstObservedFrameId() >= 3) {
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

  logging::RetrieveLogger()->debug("LM: covisible frame count: {}", covisible_frames.size());

  for (auto neighbour_keyframe: covisible_frames) {
    frame::KeyFrame::MapPointSet new_map_points;
    key_frame->CreateNewMapPoints(neighbour_keyframe, new_map_points);
    recently_added_map_points_.insert(new_map_points.begin(), new_map_points.end());
  }
  frame::KeyFrame::MapPointSet map_points;
  key_frame->ListMapPoints(map_points);
  for (auto mp: map_points)
    atlas_->GetCurrentMap()->AddMapPoint(mp);
}

void LocalMapper::Optimize(frame::KeyFrame * frame) {
  std::unordered_set<frame::KeyFrame *> local_keyframes = frame->GetCovisibilityGraph().GetCovisibleKeyFrames(),
      fixed_keyframes;
  frame::KeyFrame::MapPointSet local_map_points;
  for (auto keyframe: local_keyframes) keyframe->ListMapPoints(local_map_points);
  local_keyframes.insert(frame);
  FilterFixedKeyFames(local_keyframes, local_map_points, fixed_keyframes);
  if (fixed_keyframes.size() < 2 && fixed_keyframes.size() == local_keyframes.size())
    return;
  optimization::LocalBundleAdjustment(local_keyframes, fixed_keyframes, local_map_points);
  for (auto & kf: local_keyframes)
    kf->GetCovisibilityGraph().Update();

}

void LocalMapper::FilterFixedKeyFames(unordered_set<frame::KeyFrame *> & local_keyframes,
                                      frame::KeyFrame::MapPointSet & local_map_points,
                                      unordered_set<frame::KeyFrame *> & out_fixed) {
  size_t number_of_fixed = 0;
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
      if (frame->IsInitial()) continue;
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
  bool new_keyframes = GetUpdateQueue().size_approx() > 0;
  return new_keyframes;
}

void LocalMapper::RunIteration() {
  UpdateMessage message;
  while (GetUpdateQueue().try_dequeue(message)) {
    ProcessNewKeyFrame(message.frame);
    MapPointCulling(message.frame);
    CreateNewMapPoints(message.frame);
    if (!CheckNewKeyFrames()) {
      FuseMapPoints(message.frame);
      Optimize(message.frame);
      KeyFrameCulling(message.frame);
    }
//    NotifyObservers(message.frame);
  }
}

void LocalMapper::ListCovisiblesOfCovisibles(frame::KeyFrame * frame, std::unordered_set<frame::KeyFrame *> & out) {
  std::unordered_set<frame::KeyFrame *> covisibles =
      frame->GetCovisibilityGraph().GetCovisibleKeyFrames(frame->GetSensorConstants()->number_of_keyframe_to_search_lm);
  for (auto kf: covisibles) {
    std::unordered_set<frame::KeyFrame *> second_covisibles = kf->GetCovisibilityGraph().GetCovisibleKeyFrames(20);
    for (auto second_frame: second_covisibles) {
      if (frame != second_frame && covisibles.find(kf) != covisibles.end())
        out.insert(second_frame);
    }
  }
}

void LocalMapper::FuseMapPoints(frame::KeyFrame * frame) {
  std::unordered_set<frame::KeyFrame *> second_neighbours;
  ListCovisiblesOfCovisibles(frame, second_neighbours);
  std::unordered_set<map::MapPoint *> map_points;
  frame->ListMapPoints(map_points);
  for (auto kf: second_neighbours) {
    kf->FuseMapPoints(map_points);
  }
  frame::KeyFrame::MapPointSet mps;
  frame->ListMapPoints(mps);
  for (auto mp:mps) {
    if (!mp->IsBad())
      mp->Refresh(frame->GetFeatureExtractor());
  }
  frame->GetCovisibilityGraph().Update();
}

void LocalMapper::KeyFrameCulling(frame::KeyFrame * keyframe) {
  auto local_keyframes = keyframe->GetCovisibilityGraph().GetCovisibleKeyFrames();
  for (auto kf: local_keyframes) {
    if (kf->IsInitial())
      continue;
    frame::KeyFrame::MapPointSet map_points;
    kf->ListMapPoints(map_points);
    int mobs = 0;
    for (auto mp: map_points)
      if (mp->GetObservationCount() >= 3)
        ++mobs;
    if (mobs > map_points.size() * 0.9) {
      kf->SetBad();
    }
    for (auto mp : map_points) {
      if (!mp->IsBad())
        mp->Refresh(kf->GetFeatureExtractor());
    }
  }

}

}
