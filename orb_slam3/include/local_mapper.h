//
// Created by vahagn on 16/03/2021.
//

#ifndef ORB_SLAM3_ORB_SLAM3_INCLUDE_LOCAL_MAPPER_H_
#define ORB_SLAM3_ORB_SLAM3_INCLUDE_LOCAL_MAPPER_H_
// === orb-slam3 ===
#include <position_observer.h>

namespace orb_slam3 {

class LocalMapper : public PositionObserver {
 public:
  LocalMapper() : PositionObserver() {}
  void Run();
};

}

#endif //ORB_SLAM3_ORB_SLAM3_INCLUDE_LOCAL_MAPPER_H_
