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
#include <fstream>
#include "profiler.h"

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
#ifndef MULTITHREADED
  cancelled_ = false;
  return;
#endif
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

      map_point->ApplyStaging();
      key_frame->AddMapPoint(key_frame_obs);
      neighbour_keyframe->AddMapPoint(neighbour_obs);
      recently_added_map_points_.insert(map_point);
    }
    key_frame->UnlockMapPointContainer();
    neighbour_keyframe->UnlockMapPointContainer();
  }
  for(auto kf: covisible_frames)
    kf->GetCovisibilityGraph().Update();
  key_frame->GetCovisibilityGraph().Update();
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

    obs_to_delete.second->EraseMapPoint(obs_to_delete.first);
    if (obs_to_delete.first->GetObservationCount() == 1)
      SetBad(obs_to_delete.first);
    obs_to_delete.second->ApplyStaging();
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

size_t iteration_cycle = 0;
void LocalMapper::RunIteration() {


    if (!loop_merge_detection_queue_.Empty()) {
      switch (loop_merge_detection_queue_.Front().type) {
        case DetectionType::LoopDetected:
          CorrectLoop(loop_merge_detection_queue_.Front());
          break;
        case DetectionType::MergeDetected:
          MergeMaps(loop_merge_detection_queue_.Front());
          break;
        default:
          loop_merge_detection_queue_.Pop();
          break;
      }
      loop_merge_detection_queue_.Clear();
//      accept_key_frames_ = false;
//      new_key_frames_.Clear();
    } else if (!new_key_frames_.Empty()) {

      frame::KeyFrame * key_frame;
      key_frame = new_key_frames_.Front();
      new_key_frames_.Pop();
      accept_key_frames_ = false;

//      Profiler::Start("ProcessNewKeyFrame");
      ProcessNewKeyFrame(key_frame);
//      Profiler::End("ProcessNewKeyFrame");

//      Profiler::Start("MapPointCulling");
      MapPointCulling(key_frame);
//      Profiler::End("MapPointCulling");

      if (key_frame->GetCovisibilityGraph().GetCovisibleKeyFrames().empty())
        return;

//      Profiler::Start("CreateNewMapPoints");
      CreateNewMapPoints(key_frame);
//      Profiler::End("CreateNewMapPoints");

      if (new_key_frames_.Empty()) {
//        Profiler::Start("FuseMapPoints");
        FuseMapPoints(key_frame, false);
//        Profiler::End("FuseMapPoints");
      }
      if (new_key_frames_.Empty()) {
//        Profiler::Start("Optimize");
        Optimize(key_frame);
//        Profiler::End("Optimize");
//        Profiler::Start("KeyFrameCulling");
        KeyFrameCulling(key_frame);
//        Profiler::End("KeyFrameCulling");
      }

      if (++iteration_cycle % 20 == 0) {
        Profiler::PrintProfiles();
        std::cout << "Total Map points: "<< map::MapPoint::GetTotalMapPointCount() << std::endl;
      }
      if (loop_merge_detector_)
        loop_merge_detector_->Process(key_frame);
#ifndef MULTITHREADED
      loop_merge_detector_->RunIteration();
#endif

      accept_key_frames_ = true;
    }
    std::this_thread::sleep_for(std::chrono::nanoseconds(1));

}

void LocalMapper::ListSurroundingWindow(const frame::KeyFrame * key_frame,
                                        frame::IKeyFrameDatabase::KeyFrameSet & out_window) {
  static const size_t TEMPORAL_KFS_COUNT = 15;
  out_window = key_frame->GetCovisibilityGraph().GetCovisibleKeyFrames(TEMPORAL_KFS_COUNT);

  static const size_t MAX_TRIES = 3;
  size_t nNumTries = 0;
  while (out_window.size() < TEMPORAL_KFS_COUNT && nNumTries < MAX_TRIES) {
    for (frame::KeyFrame * pKFi: out_window) {
      if (!pKFi->IsBad()) {
        auto neighbours_of_neighbour = pKFi->GetCovisibilityGraph().GetCovisibleKeyFrames(TEMPORAL_KFS_COUNT / 2);
        out_window.insert(neighbours_of_neighbour.begin(), neighbours_of_neighbour.end());
      }
    }
    nNumTries++;
  }
}

void LocalMapper::MergeMaps(DetectionResult & detection_result) {
  std::cout << "========== Merging maps ==================" << std::endl;

  map::Map * merge_map = detection_result.keyframe->GetMap();
  map::Map * target_map = detection_result.candidate->GetMap();
  // TODO: Stop Local Mapper
  frame::IKeyFrameDatabase::KeyFrameSet current_kf_window;
  std::unordered_set<map::MapPoint *> current_window_map_points;
  ListSurroundingWindow(detection_result.keyframe, current_kf_window);
  for (auto kf: current_kf_window)
    kf->ListMapPoints(current_window_map_points);

  frame::IKeyFrameDatabase::KeyFrameSet candidate_kf_window;
  std::unordered_set<map::MapPoint *> candidate_window_map_points;
  ListSurroundingWindow(detection_result.candidate, candidate_kf_window);
  for (auto kf: current_kf_window)
    kf->ListMapPoints(candidate_window_map_points);

  geometry::Sim3Transformation G21 =
      detection_result.candidate->GetInversePosition() *
          detection_result.transformation.GetInverse() *
          detection_result.keyframe->GetPosition();

  atlas_->SetCurrentMap(target_map);
  geometry::Sim3Transformation G12 = G21.GetInverse();
  for (auto keyframe: merge_map->GetAllKeyFrames()) {
    if (keyframe->IsBad())
      continue;
    geometry::Sim3Transformation
        transform = keyframe->GetPosition() * G12;
    detection_result.candidate->GetMap()->AddKeyFrame(keyframe);
    keyframe->SetStagingPosition(transform.R, transform.T * G21.s);
    keyframe->ApplyStaging();
  }

  for (const auto & mp: merge_map->GetAllMapPoints()) {
    if (mp->IsBad())
      continue;
    mp->SetStagingPosition(G21.Transform(mp->GetPosition()));
    mp->SetStagingMinInvarianceDistance(
        mp->GetMinInvarianceDistance() / 1.2 * G21.s);
    mp->SetStagingMinInvarianceDistance(
        mp->GetMinInvarianceDistance() / 0.8 * G21.s);
    detection_result.candidate->GetMap()->AddMapPoint(mp);
    mp->ApplyStaging();
  }

  for (auto keyframe: current_kf_window) {
    if (keyframe->IsBad())
      continue;

    std::list<frame::MapPointVisibilityParams> visibles;
    keyframe->FilterVisibleMapPoints(candidate_window_map_points, visibles, true);
    std::list<std::pair<map::MapPoint *, map::MapPoint *>> matched_map_points;
    std::list<frame::Observation> local_matches;
    keyframe->MatchVisibleMapPoints(visibles, matched_map_points, local_matches);
    std::cout << "Found " << matched_map_points.size() << " Matches" << std::endl;
    for (auto match: matched_map_points) {
      if (match.first->IsBad() || match.second->IsBad())
        continue;
      if (match.first->GetObservationCount() > match.second->GetObservationCount())
        ReplaceMapPoint(match.first, match.second);
      else
        ReplaceMapPoint(match.second, match.first);
    }
    for (auto local_match: local_matches) {
      keyframe->AddMapPoint(local_match);
      local_match.GetMapPoint()->ApplyStaging();
      local_match.GetMapPoint()->ApplyStaging();
    }
    keyframe->GetCovisibilityGraph().Update();
  }

  for(auto mp: merge_map->GetAllMapPoints()){
    mp->SetMap(target_map);
  }
  for(auto keyframe: merge_map->GetAllKeyFrames()) {
    merge_map->EraseKeyFrame(keyframe);
    target_map->AddKeyFrame(keyframe);
    keyframe->SetMap(target_map);
  }
  atlas_->EraseMap(merge_map);
#warning map should be deleted
//  delete merge_map;

  // TODO: release local mapper
  std::vector<std::pair<map::MapPoint *, frame::KeyFrame *>> observations_to_delete;
  optimization::LocalBundleAdjustment(current_kf_window,
                                      candidate_kf_window,
                                      current_window_map_points,
                                      observations_to_delete,
                                      nullptr);


  // TODO: Stop everything
  for (auto mp: merge_map->GetAllMapPoints()) {
    if (mp->IsBad())
      continue;
    mp->ApplyStaging();
  }

  for (auto kf: merge_map->GetAllKeyFrames()) {
    if (kf->IsBad())
      continue;
    kf->SetMap(detection_result.candidate->GetMap());
    detection_result.candidate->GetMap()->AddKeyFrame(kf);
    kf->ApplyStaging();
  }



  return;

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

void LocalMapper::FuseMapPoints(frame::KeyFrame * frame, bool use_staging) {
  std::unordered_set<frame::KeyFrame *> second_neighbours;
  ListCovisiblesOfCovisibles(frame, second_neighbours);
  frame->LockMapPointContainer();
  for (auto kf: second_neighbours) {
    std::unordered_set<map::MapPoint *> map_points;
    kf->ListMapPoints(map_points);

    std::list<frame::MapPointVisibilityParams> visibles;
    frame->FilterVisibleMapPoints(map_points, visibles, use_staging);
    std::list<std::pair<map::MapPoint *, map::MapPoint *>> matched_mps;
    std::list<frame::Observation> local_mps;
    frame->MatchVisibleMapPoints(visibles, matched_mps, local_mps);

    for (auto & obs: local_mps) {
      frame->AddMapPoint(obs);
      obs.GetMapPoint()->ApplyStaging();
    }

    for (auto match: matched_mps) {
      if (match.first->GetObservationCount() > match.second->GetObservationCount())
        ReplaceMapPoint(match.second, match.first);
      else
        ReplaceMapPoint(match.first, match.second);
    }

  }

//  frame::KeyFrame::MapPointSet mps;
//  frame->ListMapPoints(mps);
//  for (auto mp: mps) {
//    if (!mp->IsBad()) {
//      mp->ApplyStaging();
//    }
//  }
  frame->GetCovisibilityGraph().Update();
  frame->UnlockMapPointContainer();
}

void LocalMapper::ReplaceMapPoint(map::MapPoint * old_mp, map::MapPoint * new_mp) {

  map::MapPoint::MapType old_observations = old_mp->Observations();
  for (auto old_obs: old_observations) {
    old_obs.second.GetKeyFrame()->LockMapPointContainer();
  }

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
  new_mp->ApplyStaging();
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

        kf->EraseMapPoint(mp);
        if (mp->GetStagingObservationCount() == 1) {
          SetBad(mp);
        } else
          mp->ApplyStaging();
      }
      kf->SetBad();
      kf->UnlockMapPointContainer();
    }
    for (auto mp: map_points) {
      if (!mp->IsBad()) {
        mp->ApplyStaging();
      }
    }
  }

}

void LocalMapper::SetBad(map::MapPoint * map_point) {

  for (auto & obs: map_point->StagingObservations()) {
    frame::KeyFrame * last_key_frame = obs.second.GetKeyFrame();
    last_key_frame->LockMapPointContainer();
    last_key_frame->EraseMapPoint(map_point);
    last_key_frame->UnlockMapPointContainer();
  }
  map_point->SetBad();
  map_point->GetMap()->EraseMapPoint(map_point);

}

void LocalMapper::AddToLMDetectionQueue(DetectionResult & detection_result) {
  loop_merge_detection_queue_.Push(detection_result);
}

void LocalMapper::CorrectLoop(DetectionResult & detection_result) {
  std::cout << "Loop closing " << std::endl;

  auto current_covisible_keyframes = detection_result.keyframe->GetCovisibilityGraph().GetCovisibleKeyFrames();
  geometry::Sim3Transformation SG1 = detection_result.transformation * detection_result.candidate->GetPosition();

  geometry::Pose keyframe_pose_inverse = detection_result.keyframe->GetInversePosition();

  geometry::Pose corrected_keyframe_pose(SG1.R, SG1.T / SG1.s);
  std::unordered_set<map::MapPoint *> map_points;
  for (auto covisible_kf: current_covisible_keyframes) {
    covisible_kf->ListMapPoints(map_points);
    covisible_kf->SetStagingPosition(covisible_kf->GetPosition() * keyframe_pose_inverse * corrected_keyframe_pose);
    covisible_kf->ApplyStaging();
  }

  geometry::Sim3Transformation global_sim3 = keyframe_pose_inverse * detection_result.transformation
      * detection_result.candidate->GetPosition();
  for (auto mp: map_points) {
    mp->SetStagingPosition(global_sim3.Transform(mp->GetPosition()));
    mp->ApplyStaging();
  }
  for (auto covisible_kf: current_covisible_keyframes) {
    covisible_kf->ApplyStaging();
  }
  for (auto covisible_kf: current_covisible_keyframes) {
    covisible_kf->ApplyStaging();
  }
  for (auto mp: map_points) {
    mp->ApplyStaging();
  }

  frame::BaseFrame::MapPointSet candidate_map_points;
  detection_result.candidate->ListMapPoints(candidate_map_points);
  for (auto candidate_neighbour: detection_result.candidate->GetCovisibilityGraph().GetCovisibleKeyFrames()) {
    candidate_neighbour->ListMapPoints(candidate_map_points);
  }

  for (auto covisible_kf: current_covisible_keyframes) {
    std::list<frame::MapPointVisibilityParams> visibles;
    covisible_kf->FilterVisibleMapPoints(candidate_map_points, visibles, true);
    std::list<std::pair<map::MapPoint *, map::MapPoint *>> matched_map_points;
    std::list<frame::Observation> local_matches;
    covisible_kf->MatchVisibleMapPoints(visibles, matched_map_points, local_matches);
    std::cout << "Found " << matched_map_points.size() << " Matches" << std::endl;
    for (auto match: matched_map_points) {
      if (match.first->IsBad() || match.second->IsBad())
        continue;
      if (match.first->GetObservationCount() > match.second->GetObservationCount())
        ReplaceMapPoint(match.first, match.second);
      else
        ReplaceMapPoint(match.second, match.first);
    }
    for (auto local_match: local_matches) {
      covisible_kf->AddMapPoint(local_match);
      local_match.GetMapPoint()->ApplyStaging();
    }
  }

  for (auto mp: map_points) {
    if (mp->IsBad())
      continue;
    mp->ApplyStaging();
  }
  for (auto covisible_kf: current_covisible_keyframes) {
    covisible_kf->GetCovisibilityGraph().Update();
  }

  /*loop_merge_detection_queue_.Clear();

  std::cout << "Runnning GBA " << std::endl;
  std::unordered_set<frame::KeyFrame *> all_keyframes =
      detection_result.candidate->GetMap()->GetAllKeyFrames();
  std::unordered_set<map::MapPoint *> all_map_point = detection_result.candidate->GetMap()->GetAllMapPoints();
  std::unordered_set<frame::KeyFrame *> fixed_key_frames;
  this->FilterFixedKeyFames(all_keyframes, all_map_point, fixed_key_frames);

  for(auto fkf: fixed_key_frames)
    all_keyframes.erase(fkf);

  std::vector<std::pair<map::MapPoint *, frame::KeyFrame *>> observations_to_delete;
  optimization::LocalBundleAdjustment(all_keyframes, fixed_key_frames, all_map_point, observations_to_delete, nullptr);
  for (auto & obs_to_delete: observations_to_delete) {
    obs_to_delete.second->EraseMapPoint(obs_to_delete.first);
    if (obs_to_delete.first->GetObservationCount() == 1) {
      SetBad(obs_to_delete.first);
    }
//    detection_result.candidate->GetMap()->EraseMapPoint(obs_to_delete.first);

  }*/

}

}
