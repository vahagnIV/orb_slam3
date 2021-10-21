//
// Created by vahagn on 16/03/2021.
//

#include "local_mapper.h"
#include "frame/map_point_visibility_params.h"
#include "frame/key_frame.h"
#include <frame/observation.h>
#include "logging.h"
#include "optimization/bundle_adjustment.h"
#include <map/atlas.h>
#include <map/map_point.h>
#include <settings.h>
#include <messages/messages.h>

// TODO: remove this in multithreading
#include <loop_merge_detector.h>

namespace orb_slam3 {

LocalMapper::LocalMapper(map::Atlas * atlas, LoopMergeDetector * loop_mege_detector)
    : atlas_(atlas),
      thread_(nullptr),
      accept_key_frames_(true),
      loop_merge_detector_(loop_mege_detector) {
  loop_merge_detector_->SetLocalMapper(this);
}

LocalMapper::~LocalMapper() {
  Stop();
}

void LocalMapper::Run() {
  while (!cancelled_) {
    RunIteration();
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
    if (mp->IsBad()) { mp_it = recently_added_map_points_.erase(mp_it); }
    else if (static_cast<precision_t>(mp->GetFound()) / mp->GetVisible() < 0.25) {
      SetBad(mp);
      mp_it = recently_added_map_points_.erase(mp_it);
    } else if (keyframe->Id() > mp->GetFirstObservedFrameId() && keyframe->Id() - mp->GetFirstObservedFrameId() >= 2
        && mp->GetObservationCount() < keyframe->GetSensorConstants()->min_mp_disappearance_count) {
      SetBad(mp);
      mp_it = recently_added_map_points_.erase(mp_it);
    } else if (keyframe->Id() > mp->GetFirstObservedFrameId() && keyframe->Id() - mp->GetFirstObservedFrameId() >= 3) {
      mp_it = recently_added_map_points_.erase(mp_it);
      --erased;
    } else {
      --erased;
      ++mp_it;
    }
  }

  logging::RetrieveLogger()->debug("LM: erased {} mps", erased);
}

void LocalMapper::ProcessNewKeyFrame(frame::KeyFrame * keyframe) {
  keyframe->Initialize();


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
  if (key_frame->IsInitial())
    return;
  assert(!key_frame->IsBad());

  auto covisible_frames =
      key_frame->GetCovisibilityGraph().GetCovisibleKeyFrames(key_frame->GetSensorConstants()->number_of_keyframe_to_search_lm);

  assert(!covisible_frames.empty());

  logging::RetrieveLogger()->debug("LM: covisible frame count: {}", covisible_frames.size());

  for (auto neighbour_keyframe: covisible_frames) {
    frame::KeyFrame::NewMapPoints new_map_points;

    key_frame->CreateNewMapPoints(neighbour_keyframe, new_map_points);
    key_frame->LockMapPointContainer();
    neighbour_keyframe->LockMapPointContainer();
    for (auto & new_mp: new_map_points) {
      frame::Observation & key_frame_obs = std::get<0>(new_mp);
      frame::Observation & neighbour_obs = std::get<1>(new_mp);
      assert(key_frame_obs.GetKeyFrame() == key_frame);
      assert(neighbour_obs.GetKeyFrame() == neighbour_keyframe);
      assert(key_frame_obs.GetMapPoint() == neighbour_obs.GetMapPoint());
      map::MapPoint * map_point = key_frame_obs.GetMapPoint();
      map_point->AddObservation(key_frame_obs);
      map_point->AddObservation(neighbour_obs);

      map_point->ComputeDistinctiveDescriptor();
      map_point->CalculateNormalStaging();
      map_point->ApplyStaging();
      key_frame->AddMapPoint(key_frame_obs);
      neighbour_keyframe->AddMapPoint(neighbour_obs);
      recently_added_map_points_.insert(map_point);
    }
    key_frame->UnlockMapPointContainer();
    neighbour_keyframe->UnlockMapPointContainer();
  }
}

void LocalMapper::Optimize(frame::KeyFrame * frame) {
  std::unordered_set<frame::KeyFrame *> local_keyframes = frame->GetCovisibilityGraph().GetCovisibleKeyFrames(),
      fixed_keyframes;
  frame::KeyFrame::MapPointSet local_map_points;
  for (auto keyframe: local_keyframes) keyframe->ListMapPoints(local_map_points);
  local_keyframes.insert(frame);
  FilterFixedKeyFames(local_keyframes, local_map_points, fixed_keyframes);

  for (auto & kf: fixed_keyframes)
    local_keyframes.erase(kf);

  if (fixed_keyframes.size() < 2 && fixed_keyframes.size() == local_keyframes.size())
    return;

  std::vector<std::pair<map::MapPoint *, frame::KeyFrame *>> observations_to_delete;
  optimization::LocalBundleAdjustment(local_keyframes,
                                      fixed_keyframes,
                                      local_map_points,
                                      observations_to_delete,
                                      nullptr);

  for (auto & obs_to_delete: observations_to_delete) {
    obs_to_delete.second->LockMapPointContainer();
    obs_to_delete.first->LockObservationsContainer();
    obs_to_delete.second->EraseMapPoint(obs_to_delete.first);
    if (obs_to_delete.first->GetObservationCount() == 1)
      SetBad(obs_to_delete.first);
    obs_to_delete.first->UnlockObservationsContainer();
    obs_to_delete.second->UnlockMapPointContainer();
  }
  for (auto & kf: local_keyframes)
    kf->GetCovisibilityGraph().Update();

}

void LocalMapper::FilterFixedKeyFames(const std::unordered_set<frame::KeyFrame *> & local_keyframes,
                                      const frame::BaseFrame::MapPointSet & local_map_points,
                                      std::unordered_set<frame::KeyFrame *> & out_fixed) {
  size_t number_of_fixed = 0;
  for (auto keyframe: local_keyframes) {
    if (keyframe->IsInitial())
      ++number_of_fixed;
  }

  for (auto map_point: local_map_points) {
    for (auto observation: map_point->Observations()) {
      if (local_keyframes.find(observation.second.GetKeyFrame()) == local_keyframes.end())
        out_fixed.insert(observation.second.GetKeyFrame());
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
    }
    if (number_of_fixed < 2 && nullptr != second_eraliset_keyframe) {
      ++number_of_fixed;
      out_fixed.insert(second_eraliset_keyframe);
    }
  }
}


void LocalMapper::RunIteration() {

  while (!cancelled_) {
    if (!loop_merge_detection_queue_.Empty()) {
      loop_merge_detection_queue_.Clear();
//      accept_key_frames_ = false;
//      new_key_frames_.Clear();
    }
    else if (!new_key_frames_.Empty()) {
      frame::KeyFrame *key_frame;
      key_frame = new_key_frames_.Front();
      new_key_frames_.Pop();
      accept_key_frames_ = false;
      ProcessNewKeyFrame(key_frame);

      MapPointCulling(key_frame);
      CreateNewMapPoints(key_frame);

      if (new_key_frames_.Empty()) {
        FuseMapPoints(key_frame);
      }
      if (new_key_frames_.Empty()) {
        Optimize(key_frame);
        KeyFrameCulling(key_frame);
      }
      if (loop_merge_detector_)
        loop_merge_detector_->Process(key_frame);

      accept_key_frames_ = true;
    }
    std::this_thread::sleep_for(std::chrono::nanoseconds(1));
  }
}

void LocalMapper::AddToQueue(frame::KeyFrame * key_frame) {
  new_key_frames_.Push(key_frame);
}

void LocalMapper::ListCovisiblesOfCovisibles(const frame::KeyFrame * frame,
                                             std::unordered_set<frame::KeyFrame *> & out) {
  std::unordered_set<frame::KeyFrame *> covisibles =
      frame->GetCovisibilityGraph().GetCovisibleKeyFrames(frame->GetSensorConstants()->number_of_keyframe_to_search_lm);
  for (auto kf: covisibles) {
    out.insert(kf);
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
  frame->LockMapPointContainer();
  for (auto kf: second_neighbours) {
    std::unordered_set<map::MapPoint *> map_points;
    kf->ListMapPoints(map_points);

    std::list<frame::MapPointVisibilityParams> visibles;
    frame->FilterVisibleMapPoints(map_points, visibles, false);
    std::list<std::pair<map::MapPoint *, map::MapPoint *>> matched_mps;
    std::list<frame::Observation> local_mps;
    frame->MatchVisibleMapPoints(visibles, matched_mps, local_mps);

    for (auto & obs: local_mps)
      frame->AddMapPoint(obs);

    for (auto match: matched_mps) {
      if (match.first->GetObservationCount() > match.second->GetObservationCount())
        ReplaceMapPoint(match.second, match.first);
      else
        ReplaceMapPoint(match.first, match.second);
    }

  }

  frame::KeyFrame::MapPointSet mps;
  frame->ListMapPoints(mps);
  for (auto mp: mps) {
    if (!mp->IsBad()) {
      mp->ComputeDistinctiveDescriptor();
      mp->CalculateNormalStaging();
      mp->ApplyStaging();
    }
  }
  frame->GetCovisibilityGraph().Update();
  frame->UnlockMapPointContainer();
}

void LocalMapper::ReplaceMapPoint(map::MapPoint * old_mp, map::MapPoint * new_mp) {
  map::MapPoint::MapType old_observations = old_mp->Observations();
  for (auto old_obs: old_observations) {
    old_obs.second.GetKeyFrame()->LockMapPointContainer();
  }
  old_mp->LockObservationsContainer();
  new_mp->LockObservationsContainer();

  for (auto old_obs: old_observations) {
    frame::KeyFrame * key_frame = old_obs.second.GetKeyFrame();
    if (new_mp->IsInKeyFrame(key_frame)) {
      key_frame->EraseMapPoint(new_mp);
    }
    key_frame->EraseMapPoint(old_mp);
    old_obs.second.SetMapPoint(new_mp);
    key_frame->AddMapPoint(old_obs.second);
  }
  old_mp->SetReplaced(new_mp);
  old_mp->GetMap()->EraseMapPoint(old_mp);
  old_mp->UnlockObservationsContainer();
  new_mp->UnlockObservationsContainer();
  for (auto old_obs: old_observations) {
    old_obs.second.GetKeyFrame()->UnlockMapPointContainer();
  }
}

void LocalMapper::KeyFrameCulling(frame::KeyFrame * keyframe) {
  auto local_keyframes = keyframe->GetCovisibilityGraph().GetCovisibleKeyFrames();
  for (auto kf: local_keyframes) {
    if (kf->IsInitial())
      continue;
    frame::KeyFrame::MapPointSet map_points;
    kf->ListMapPoints(map_points);
    int redundan_observations = 0;
    for (auto mp: map_points) {
      if (mp->GetObservationCount() >= 3) {
        frame::Observation observation;
        if (!mp->GetObservation(kf, observation))
          throw std::runtime_error("KeyframeCulling Observation not in map point");
        int scale_level = kf->GetScaleLevel(observation);
        map::MapPoint::MapType observations = mp->Observations();
        long no_obs =
            std::count_if(observations.begin(),
                          observations.end(),
                          [&scale_level](const map::MapPoint::MapType::value_type & obs) {
                            return obs.second.GetKeyFrame()->GetScaleLevel(obs.second) < scale_level + 1;
                          }) - 1;
        if (no_obs > 3l)
          ++redundan_observations;
      }
    }

    if (redundan_observations > map_points.size() * 0.9) {
      atlas_->GetKeyframeDatabase()->Erase(kf);
      kf->LockMapPointContainer();
      for (auto mp: map_points) {
        if (mp->IsBad())
          continue;

        mp->LockObservationsContainer();
        kf->EraseMapPoint(mp);
        if (mp->GetObservationCount() == 1) {
          SetBad(mp);
        }
        mp->UnlockObservationsContainer();
      }
      kf->SetBad();
      kf->UnlockMapPointContainer();
    }
    for (auto mp: map_points) {
      if (!mp->IsBad()) {
        mp->CalculateNormalStaging();
        mp->ApplyStaging();
        mp->ComputeDistinctiveDescriptor();
      }
    }
  }

}

void LocalMapper::SetBad(map::MapPoint * map_point) {
  map::MapPoint::MapType observations = map_point->Observations();
  for (auto &obs: observations) {
    frame::KeyFrame *last_key_frame = obs.second.GetKeyFrame();
    last_key_frame->LockMapPointContainer();
    last_key_frame->EraseMapPoint(map_point);
    last_key_frame->UnlockMapPointContainer();
  }
  map_point->SetBad();
  map_point->GetMap()->EraseMapPoint(map_point);

}

void LocalMapper::AddToLMDetectionQueue(DetectionResult &detection_result) {
  loop_merge_detection_queue_.Push(detection_result);
}

}
