//
// Created by vahagn on 30/06/2021.
//

#ifndef ORB_SLAM3_SRC_LOOP_MERGE_DETECTOR_H_
#define ORB_SLAM3_SRC_LOOP_MERGE_DETECTOR_H_

#include "position_observer.h"

namespace orb_slam3 {

class LoopMergeDetector: public PositionObserver{
 public:
  LoopMergeDetector(){}
  void RunIteration();
 private:


};

}

#endif //ORB_SLAM3_SRC_LOOP_MERGE_DETECTOR_H_
