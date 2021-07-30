//
// Created by vahagn on 30/06/2021.
//

#ifndef ORB_SLAM3_SRC_LOOP_MERGE_DETECTOR_H_
#define ORB_SLAM3_SRC_LOOP_MERGE_DETECTOR_H_

#include "position_observer.h"
#include <frame/database/ikey_frame_database.h>
#include <map/atlas.h>
namespace orb_slam3 {

class LoopMergeDetector : public PositionObserver {

 public:
  LoopMergeDetector(frame::IKeyFrameDatabase * key_frame_database, map::Atlas * atlas);
  void RunIteration();
 private:
  enum DetectionResult {
    LoopDetected = 1,
    MergeDetected = 2,
    Empty = 0
  };
 private:
  typedef std::unordered_set<frame::KeyFrame *> KeyFrameSet;
  typedef std::vector<std::pair<map::MapPoint *, map::MapPoint *>> MapPointMatches;
  static bool DetectLoopOrMerge(const frame::KeyFrame * key_frame,
                                frame::IKeyFrameDatabase::KeyFrameSet & current_neighbourhood,
                                frame::KeyFrame * candidate_keyframe,
                                geometry::Sim3Transformation & out_sim3_transformation);
 private:
  static bool Intersect(const KeyFrameSet & bow_candidate_neighbours, const KeyFrameSet & key_frame_neighbours);
  static void FindMapPointMatches(const frame::KeyFrame * current_key_frame,
                                  const LoopMergeDetector::KeyFrameSet & loop_neighbours,
                                  MapPointMatches & out_matches);
  static void ListSurroundingWindow(const frame::KeyFrame * key_frame,
                                    frame::IKeyFrameDatabase::KeyFrameSet & out_window);
  static void ListAllMapPoints(const frame::IKeyFrameDatabase::KeyFrameSet & key_frames,
                               std::unordered_set<map::MapPoint *> & out_mps);
  static void SearchAndFuse();
 private:
  frame::IKeyFrameDatabase * key_frame_database_;
  map::Atlas * atlas_;

};

}

#endif //ORB_SLAM3_SRC_LOOP_MERGE_DETECTOR_H_
