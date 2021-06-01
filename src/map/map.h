//
// Created by vahagn on 12/8/20.
//

#ifndef ORB_SLAM3_INCLUDE_MAP_H_
#define ORB_SLAM3_INCLUDE_MAP_H_

// == stl =====
#include <unordered_set>

// == orb-slam3 ===
#include "../frame/frame_base.h"

namespace orb_slam3 {
namespace map {
class Map {
 public:
  void AddKeyFrame(frame::KeyFrame * frame);
  void SetInitialKeyFrame(frame::KeyFrame * frame);
  void AddMapPoint(MapPoint * map_point);
  std::unordered_set<MapPoint *> GetAllMapPoints();
 private:
  std::unordered_set<frame::KeyFrame *> key_frames_;
  frame::KeyFrame * initial_keyframe_;
  std::unordered_set<MapPoint *> map_points_;

};
}
}
#endif //ORB_SLAM3_INCLUDE_MAP_H_
