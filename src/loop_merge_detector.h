//
// Created by vahagn on 30/06/2021.
//

#ifndef ORB_SLAM3_SRC_LOOP_MERGE_DETECTOR_H_
#define ORB_SLAM3_SRC_LOOP_MERGE_DETECTOR_H_

#include "position_observer.h"
#include <frame/database/ikey_frame_database.h>
namespace orb_slam3 {

class LoopMergeDetector : public PositionObserver {

 public:
  LoopMergeDetector(frame::IKeyFrameDatabase * key_frame_database);
  LoopMergeDetector() {}
  void RunIteration();
 private:
  enum DetectionResult {
    LoopDetected = 1,
    MergeDetected = 2,
    Empty = 0
  };
 private:
  DetectionResult DetectLoopOrMerge(frame::KeyFrame * key_frame) const;
  frame::IKeyFrameDatabase * key_frame_database_;

};

}

#endif //ORB_SLAM3_SRC_LOOP_MERGE_DETECTOR_H_
