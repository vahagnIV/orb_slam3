//
// Created by vahagn on 12/8/20.
//

#ifndef ORB_SLAM3_INCLUDE_ATLAS_H_
#define ORB_SLAM3_INCLUDE_ATLAS_H_

#include <map.h>
#include <memory>
namespace nvision {

class Atlas {
 public:
  Atlas();
  Map *GetCurrentMap();
  void CreateNewMap();
  ~Atlas();
 private:
  Map *current_map_;
};

}
#endif //ORB_SLAM3_INCLUDE_ATLAS_H_
