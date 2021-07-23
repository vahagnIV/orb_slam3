//
// Created by vahagn on 30/06/2021.
//

#include "loop_merge_detector.h"
#include <map/map.h>
#include <geometry/ransac_sim3_solver.h>

//TODO: remove in production
#include <debug/debug_utils.h>
namespace orb_slam3 {

LoopMergeDetector::LoopMergeDetector(frame::IKeyFrameDatabase * key_frame_database) :
    key_frame_database_(key_frame_database) {
}

void LoopMergeDetector::RunIteration() {
  UpdateMessage message{};
  while (GetUpdateQueue().try_dequeue(message)) {
    auto map = message.frame->GetMap();
    if (map->GetAllKeyFrames().size() < 7) {
      key_frame_database_->Append(message.frame);
      return;
    }
    auto detection_result = DetectLoopOrMerge(message.frame);

  }
}

bool LoopMergeDetector::Intersect(const KeyFrameSet & bow_candidate_neighbours,
                                  const KeyFrameSet & key_frame_neighbours) {
  return std::any_of(bow_candidate_neighbours.begin(),
                     bow_candidate_neighbours.end(),
                     [&key_frame_neighbours]( frame::KeyFrame * key_frame){
    return key_frame_neighbours.find(key_frame)!=key_frame_neighbours.end();
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

LoopMergeDetector::DetectionResult LoopMergeDetector::DetectLoopOrMerge(frame::KeyFrame * key_frame) const {

  frame::IKeyFrameDatabase::KeyFrameSet merge_candidates, loop_candidates;
  key_frame_database_->DetectNBestCandidates(key_frame,
                                             loop_candidates,
                                             merge_candidates,
                                             constants::MAX_NUMBER_OF_MATCH_CANDIDATES);
  auto key_frame_neighbours = key_frame->GetCovisibilityGraph().GetCovisibleKeyFrames();

  // TODO: remove the following line
  if (!merge_candidates.empty()) {

    for (const auto & merge_candidate: merge_candidates) {
      if (merge_candidate->IsBad()) continue;

      auto loop_candidate_neighbours = merge_candidate->GetCovisibilityGraph().GetCovisibleKeyFrames(constants::LM_COVISIBLE_COUNT);
      loop_candidate_neighbours.insert(merge_candidate);

      // Proximity check
      if (Intersect(loop_candidate_neighbours, key_frame_neighbours)) continue;
      MapPointMatches matches;
      FindMapPointMatches(key_frame, loop_candidate_neighbours, matches);

      if (matches.size() < constants::LM_MIN_NUMBER_OF_MP_MATCHES)
        continue;

      geometry::Sim3Transformation transformation;
      if (key_frame->FindSim3Transformation(matches, merge_candidate, transformation)) {
        frame::KeyFrame::MapPointSet map_points;
        for (const auto & loop_neighbour: loop_candidate_neighbours) {
          loop_neighbour->ListMapPoints(map_points);
        }
        std::list<frame::MapPointVisibilityParams> visible_map_points;
        const auto & pose = merge_candidate->GetPosition();
        key_frame->FilterVisibleMapPoints(map_points,
                                          transformation,
                                          pose,
                                          visible_map_points,
                                          8);
        if (visible_map_points.size() < constants::LM_MIN_NUMBER_OF_VISIBLES)
          continue;

        key_frame->AdjustSim3Transformation(visible_map_points, merge_candidate, transformation);
      }

    }
  }


  return LoopMergeDetector::Empty;
}

}