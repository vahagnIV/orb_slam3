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

LoopMergeDetector::DetectionResult LoopMergeDetector::DetectLoopOrMerge(frame::KeyFrame * key_frame) const {
  frame::IKeyFrameDatabase::KeyFrameSet merge_candidates, loop_candidates;
  key_frame_database_->DetectNBestCandidates(key_frame, loop_candidates, merge_candidates, 3);

  frame::KeyFrame * best_local_kf = nullptr;
  size_t best_match_count = 0;
  if (!loop_candidates.empty()) {
    auto covisible_kfs = key_frame->GetCovisibilityGraph().GetCovisibleKeyFrames(5);
    covisible_kfs.insert(key_frame);
    for (const auto & loop_candidate: loop_candidates) {
      if (loop_candidate->IsBad()) {
        continue;
      }
      const auto & candidate_handler = loop_candidate->GetFeatureHandler();
      std::unordered_map<frame::KeyFrame *, features::FastMatches> neighbour_matches;
      for (auto neighbour: covisible_kfs) {

        const auto & neighbour_handler = neighbour->GetFeatureHandler();

        candidate_handler->FastMatch(neighbour_handler,
                                     neighbour_matches[neighbour],
                                     features::MatchingSeverity::WEAK,
                                     true);
      }
    }
  }

  if (!merge_candidates.empty()) {

  }

  return LoopMergeDetector::MergeDetected;
}

}