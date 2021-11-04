//
// Created by vahagn on 30/06/2021.
//

#include "loop_merge_detector.h"
#include <map/map.h>
#include <map/map_point.h>
#include <geometry/ransac_sim3_solver.h>
#include <optimization/bundle_adjustment.h>

//TODO: remove in production
#include <debug/debug_utils.h>
#include <local_mapper.h>
namespace orb_slam3 {

LoopMergeDetector::LoopMergeDetector(map::Atlas * atlas) :
    atlas_(atlas), thread_(nullptr) {
}

LoopMergeDetector::~LoopMergeDetector() {
  Stop();
}

void LoopMergeDetector::Start() {
  if (thread_)
    return;
  canceled_ = false;
  thread_ = new std::thread(&LoopMergeDetector::Run, this);

}

void LoopMergeDetector::Stop() {
  if (nullptr == thread_)
    return;
  canceled_ = true;
  thread_->join();
  delete thread_;
  thread_ = nullptr;
}

void LoopMergeDetector::Run() {
  while (!canceled_)
    RunIteration();
}

void LoopMergeDetector::Process(frame::KeyFrame * key_frame) {
  queue_.enqueue(key_frame);
}

void LoopMergeDetector::RunIteration() {
  frame::KeyFrame * key_frame;
  while (queue_.try_dequeue(key_frame)) {
    auto map = key_frame->GetMap();
    if (map->GetAllKeyFrames().size() < 7) {
      map->GetAtlas()->GetKeyframeDatabase()->Append(key_frame);
      return;
    }

    frame::IKeyFrameDatabase::KeyFrameSet merge_candidates, loop_candidates;
    map->GetAtlas()->GetKeyframeDatabase()->DetectNBestCandidates(key_frame,
                                                                  loop_candidates,
                                                                  merge_candidates,
                                                                  constants::MAX_NUMBER_OF_MATCH_CANDIDATES);
    auto key_frame_neighbours = key_frame->GetCovisibilityGraph().GetCovisibleKeyFrames();

    // TODO: remove the following line
    if (!merge_candidates.empty()) {

      for (const auto & merge_candidate: merge_candidates) {
        if (merge_candidate->IsBad()) continue;
        geometry::Sim3Transformation L12;
        if (DetectLoopOrMerge(key_frame, key_frame_neighbours, merge_candidate, L12)) {
          std::cout << "==================================" << std::endl;
          L12.print();
          std::cout << "==================================" << std::endl;

          // TODO: Stop Local Mapper
          frame::IKeyFrameDatabase::KeyFrameSet current_kf_window;
          std::unordered_set<map::MapPoint *> current_window_map_points;
          ListSurroundingWindow(key_frame, current_kf_window);
          ListAllMapPoints(current_kf_window, current_window_map_points);

          frame::IKeyFrameDatabase::KeyFrameSet candidate_kf_window;
          std::unordered_set<map::MapPoint *> candidate_window_map_points;
          ListSurroundingWindow(merge_candidate, candidate_kf_window);
          ListAllMapPoints(candidate_kf_window, candidate_window_map_points);

          geometry::Sim3Transformation G21 =
              merge_candidate->GetInversePosition() *
                  L12.GetInverse() *
                  key_frame->GetPosition();

          geometry::Sim3Transformation G12 = G21.GetInverse();
          for (auto keyframe: key_frame->GetMap()->GetAllKeyFrames()) {
            if (keyframe->IsBad())
              continue;
            geometry::Sim3Transformation
                transform = keyframe->GetPosition() * G12;

            keyframe->SetStagingPosition(transform.R, transform.T * G21.s);
          }

          for (const auto & mp: key_frame->GetMap()->GetAllMapPoints()) {
            if (mp->IsBad())
              continue;
            mp->SetStagingPosition(G21.Transform(mp->GetPosition()));
            mp->SetStagingMinInvarianceDistance(
                mp->GetMinInvarianceDistance() / 1.2 * G21.s);
            mp->SetStagingMinInvarianceDistance(
                mp->GetMinInvarianceDistance() / 0.8 * G21.s);
            mp->CalculateNormalStaging();
          }

          for (auto keyframe: key_frame->GetMap()->GetAllKeyFrames()) {
            if (keyframe->IsBad())
              continue;
            //TODO: Implement this function
//            keyframe->FuseMapPoints(current_window_map_points, true);
          }
          // TODO: release local mapper
          std::vector<std::pair<map::MapPoint *, frame::KeyFrame *>> observations_to_delete;
          optimization::LocalBundleAdjustment(current_kf_window,
                                              candidate_kf_window,
                                              current_window_map_points,
                                              observations_to_delete,
                                              nullptr);

          // TODO: Stop everything
          for (auto mp: key_frame->GetMap()->GetAllMapPoints()) {
            if (mp->IsBad())
              continue;
            mp->SetMap(merge_candidate->GetMap());
            mp->ApplyStaging();
          }

          for (auto kf: key_frame->GetMap()->GetAllKeyFrames()) {
            if (kf->IsBad())
              continue;
            kf->SetMap(merge_candidate->GetMap());
            kf->ApplyStaging();
          }
          atlas_->SetCurrentMap(merge_candidate->GetMap());

          return;
        }
      }
    }

    if (!loop_candidates.empty()) {
      for (const auto & loop_candidate: loop_candidates) {
        if (loop_candidate->IsBad()) continue;
        geometry::Sim3Transformation transformation;
        if (DetectLoopOrMerge(key_frame, key_frame_neighbours, loop_candidate, transformation)) {
          DetectionResult result
              {.type = DetectionType::LoopDetected,
                  .keyframe = key_frame,
                  .candidate = loop_candidate,
                  .transformation = transformation};
          local_mapper_->AddToLMDetectionQueue(result);
          // TODO: merge
          return;
        }
      }
    }

    atlas_->GetKeyframeDatabase()->Append(key_frame);
  }
}

void LoopMergeDetector::SetLocalMapper(LocalMapper *local_mapper) {
  local_mapper_ = local_mapper;
}

bool LoopMergeDetector::Intersect(const KeyFrameSet &bow_candidate_neighbours,
                                  const KeyFrameSet &key_frame_neighbours) {
  return std::any_of(bow_candidate_neighbours.begin(),
                     bow_candidate_neighbours.end(),
                     [&key_frame_neighbours](frame::KeyFrame *key_frame) {
                       return key_frame_neighbours.find(key_frame) != key_frame_neighbours.end();
                     });
}

void LoopMergeDetector::FindMapPointMatches(const frame::KeyFrame * current_key_frame,
                                            const LoopMergeDetector::KeyFrameSet & loop_neighbours,
                                            MapPointMatches & out_matches) {
  std::unordered_set<map::MapPoint *> loop_map_points;
  std::unordered_set<map::MapPoint *> kf_map_points;

  for (const auto & loop_candidate: loop_neighbours) {
    MapPointMatches matches;
    current_key_frame->FindMatchingMapPoints(loop_candidate, matches);
    for (const auto & match: matches) {
      if (loop_map_points.find(match.second) == loop_map_points.end()
          && kf_map_points.find(match.first) == kf_map_points.end()) {
        loop_map_points.insert(match.second);
        kf_map_points.insert(match.first);
        out_matches.emplace_back(match);
      }
    }
  }
}

bool LoopMergeDetector::DetectLoopOrMerge(const frame::KeyFrame * key_frame,
                                          frame::IKeyFrameDatabase::KeyFrameSet & current_neighbourhood,
                                          frame::KeyFrame * candidate_keyframe,
                                          geometry::Sim3Transformation & out_sim3_transformation) {

  if (candidate_keyframe->IsBad()) return false;

  auto candidate_neighbours =
      candidate_keyframe->GetCovisibilityGraph().GetCovisibleKeyFrames(constants::LM_COVISIBLE_COUNT);
  candidate_neighbours.insert(candidate_keyframe);

  // Proximity check
  if (Intersect(candidate_neighbours, current_neighbourhood)) return false;
  MapPointMatches matches;
  FindMapPointMatches(key_frame, candidate_neighbours, matches);

  if (matches.size() < constants::LM_MIN_NUMBER_OF_MP_MATCHES)
    return false;

  if (!key_frame->FindSim3Transformation(matches, candidate_keyframe, out_sim3_transformation))
    return false;

  frame::KeyFrame::MapPointSet map_points;
  for (const auto & candidate_neighbour: candidate_neighbours) {
    candidate_neighbour->ListMapPoints(map_points);
  }
  std::list<frame::MapPointVisibilityParams> visible_map_points;
  const auto & pose = candidate_keyframe->GetPosition();
  key_frame->FilterVisibleMapPoints(map_points,
                                    out_sim3_transformation,
                                    pose,
                                    8,
                                    visible_map_points);
  if (visible_map_points.size() < constants::LM_MIN_NUMBER_OF_VISIBLES)
    return false;

  // TODO: move the constant to constants
  return key_frame->AdjustSim3Transformation(visible_map_points, candidate_keyframe, out_sim3_transformation) > 15;

}

void LoopMergeDetector::ListSurroundingWindow(const frame::KeyFrame * key_frame,
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

void LoopMergeDetector::ListAllMapPoints(const frame::IKeyFrameDatabase::KeyFrameSet & key_frames,
                                         std::unordered_set<map::MapPoint *> & out_mps) {
  for (auto kf: key_frames)
    kf->ListMapPoints(out_mps);
}

}