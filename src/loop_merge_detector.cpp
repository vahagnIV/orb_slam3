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
  for (auto bow_candidate_neighbour: bow_candidate_neighbours) {
    if (key_frame_neighbours.find(bow_candidate_neighbour) != key_frame_neighbours.end()) {
      return true;
    }
  }
  return false;
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
  key_frame_database_->DetectNBestCandidates(key_frame, loop_candidates, merge_candidates, 3);
  auto key_frame_neighbours = key_frame->GetCovisibilityGraph().GetCovisibleKeyFrames();

  // TODO: remove the following line
  loop_candidates = merge_candidates;
  if (!loop_candidates.empty()) {

    for (const auto & loop_candidate: loop_candidates) {
      if (loop_candidate->IsBad()) continue;

      auto loop_candidate_neighbours = loop_candidate->GetCovisibilityGraph().GetCovisibleKeyFrames(5);
      loop_candidate_neighbours.insert(loop_candidate);

      // Proximity check
      if (Intersect(loop_candidate_neighbours, key_frame_neighbours)) continue;
      MapPointMatches matches;
      FindMapPointMatches(key_frame, loop_candidate_neighbours, matches);
      /*cv::Mat result = debug::DrawMapPointMatches(dynamic_cast<frame::monocular::MonocularKeyFrame *>(key_frame),
                                                  dynamic_cast<frame::monocular::MonocularKeyFrame *>(loop_candidate),
                                                  matches);
      cv::imshow("Matched mps", result);
      cv::waitKey();*/

      if (matches.size() < 20)
        continue;

      geometry::Sim3Transformation transformation;
      if (key_frame->FindSim3Transformation(matches, loop_candidate, transformation)) {
        frame::KeyFrame::MapPointSet map_points;
        for (const auto & loop_neighbour: loop_candidate_neighbours) {
          loop_neighbour->ListMapPoints(map_points);
        }
        std::list<frame::MapPointVisibilityParams> visible_map_points;
        const auto & pose = loop_candidate->GetPosition();
        key_frame->FilterVisibleMapPoints(map_points, transformation, pose, visible_map_points, 8);
        if (visible_map_points.size() < 50)
          continue;

        key_frame->AdjustSim3Transformation(visible_map_points, loop_candidate, transformation);

        std::cout << "asd" << std::endl;

      }

    }
  }

  return LoopMergeDetector::MergeDetected;
}

}