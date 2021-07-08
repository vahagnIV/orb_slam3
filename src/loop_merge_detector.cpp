//
// Created by vahagn on 30/06/2021.
//

#include "loop_merge_detector.h"
#include <map/map.h>
namespace orb_slam3 {

LoopMergeDetector::LoopMergeDetector(frame::IKeyFrameDatabase * key_frame_database) :
    key_frame_database_(key_frame_database) {
}

void LoopMergeDetector::RunIteration() {
  UpdateMessage message;
  while (GetUpdateQueue().try_dequeue(message)) {
    auto map = message.frame->GetMap();
    if (map->GetAllKeyFrames().size() < 12) {
      key_frame_database_->Append(message.frame);
      return;
    }

  }
}

bool LoopMergeDetector::Intersect(const KeyFrameSet & bow_candidate_neighbours,
                                  const KeyFrameSet & key_frame_neighbours) {
  for (auto bow_candidate_neighbour: bow_candidate_neighbours) {
    if (key_frame_neighbours.find(bow_candidate_neighbour) != key_frame_neighbours.end()) {
      return true;
    }
  }
}

LoopMergeDetector::DetectionResult LoopMergeDetector::DetectLoopOrMerge(frame::KeyFrame * key_frame) const {
  frame::IKeyFrameDatabase::KeyFrameSet merge_candidates, loop_candidates;
  key_frame_database_->DetectNBestCandidates(key_frame, loop_candidates, merge_candidates, 3);
  const auto & kf_handler = key_frame->GetFeatureHandler();
  auto key_frame_neighbours = key_frame->GetCovisibilityGraph().GetCovisibleKeyFrames();
  if (!loop_candidates.empty()) {

    for (const auto & bow_candidate: loop_candidates) {
      if (bow_candidate->IsBad()) continue;


      auto bow_candidate_neighbours = key_frame->GetCovisibilityGraph().GetCovisibleKeyFrames(5);
      bow_candidate_neighbours.insert(key_frame);

      // Proximity check
      if (Intersect(bow_candidate_neighbours, key_frame_neighbours)) continue;

      std::unordered_map<frame::KeyFrame *, features::FastMatches> neighbour_matches;

      for (auto bow_candidate_neighbour: bow_candidate_neighbours) {
        const auto & neighbour_handler = bow_candidate_neighbour->GetFeatureHandler();
        kf_handler->FastMatch(neighbour_handler,
                              neighbour_matches[bow_candidate_neighbour],
                              features::MatchingSeverity::WEAK,
                              true);
      }

      std::unordered_set<map::MapPoint *> matched_map_points;

    }
  }

  if (!merge_candidates.empty()) {

  }

  return LoopMergeDetector::MergeDetected;
}

}