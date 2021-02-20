//
// Created by vahagn on 12/8/20.
//

#ifndef ORB_SLAM3_INCLUDE_ATLAS_H_
#define ORB_SLAM3_INCLUDE_ATLAS_H_
// === stl ===
#include <memory>

// === orb-slam3 ===
#include <map/map.h>

namespace orb_slam3 {
namespace map {
class Atlas {
 public:
  Atlas();
  Map * GetCurrentMap();
  void CreateNewMap();
  ~Atlas();
 private:
  Map * current_map_;
};

}
}
#endif //ORB_SLAM3_INCLUDE_ATLAS_H_
