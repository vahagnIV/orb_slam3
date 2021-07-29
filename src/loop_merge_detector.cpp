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

    frame::IKeyFrameDatabase::KeyFrameSet merge_candidates, loop_candidates;
    key_frame_database_->DetectNBestCandidates(message.frame,
                                               loop_candidates,
                                               merge_candidates,
                                               constants::MAX_NUMBER_OF_MATCH_CANDIDATES);
    auto key_frame_neighbours = message.frame->GetCovisibilityGraph().GetCovisibleKeyFrames();

    // TODO: remove the following line
    if (!merge_candidates.empty()) {

      for (const auto & merge_candidate: merge_candidates) {
        if (merge_candidate->IsBad()) continue;
        geometry::Sim3Transformation transformation;
        if (DetectLoopOrMerge(message.frame, key_frame_neighbours, merge_candidate, transformation)) {
          std::cout << "==================================" << std::endl;
          transformation.print();
          std::cout << "==================================" << std::endl;

          frame::IKeyFrameDatabase::KeyFrameSet current_kf_window;
          std::unordered_set<map::MapPoint *> current_window_map_points;
          ListSurroundingWindow(message.frame, current_kf_window);
          ListAllMapPoints(current_kf_window, current_window_map_points);

          frame::IKeyFrameDatabase::KeyFrameSet candidate_kf_window;
          std::unordered_set<map::MapPoint *> candidate_window_map_points;
          ListSurroundingWindow(merge_candidate, candidate_kf_window);
          ListAllMapPoints(candidate_kf_window, candidate_window_map_points);

          geometry::Sim3Transformation from_current_to_merge_candidate =
              merge_candidate->GetInversePosition() *
                  transformation.GetInverse() *
                  message.frame->GetPosition();

          for (const auto & mp: current_window_map_points) {
            mp->SetStagingPosition(from_current_to_merge_candidate.Transform(mp->GetPosition()));
          }

          for (auto keyframe: current_kf_window) {
            geometry::Sim3Transformation
                transform = keyframe->GetPosition() * from_current_to_merge_candidate.GetInverse();
            keyframe->SetStagingPosition(transform.R, transform.T);
            keyframe->FuseMapPoints(current_window_map_points, true);
          }

          optimization::LocalBundleAdjustment(current_kf_window, candidate_kf_window, current_window_map_points);
          cv::Mat mm = cv::imread("/home/vahagn/Pictures/731c5e7b912764af7be25763ac7e098c.jpg");
          cv::imshow("bad", mm);
          cv::waitKey();

          // TODO: merge
          return;
        }
      }
    }

    if (loop_candidates.empty()) {
      for (const auto & loop_candidate: loop_candidates) {
        if (loop_candidate->IsBad()) continue;
        geometry::Sim3Transformation transformation;
        if (DetectLoopOrMerge(message.frame, key_frame_neighbours, loop_candidate, transformation)) {
          // TODO: merge
          return;
        }
      }
    }
  }
}

bool LoopMergeDetector::Intersect(const KeyFrameSet & bow_candidate_neighbours,
                                  const KeyFrameSet & key_frame_neighbours) {
  return std::any_of(bow_candidate_neighbours.begin(),
                     bow_candidate_neighbours.end(),
                     [&key_frame_neighbours](frame::KeyFrame * key_frame) {
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
                                    visible_map_points,
                                    8);
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
    for (frame::KeyFrame * pKFi : out_window) {
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