//
// Created by vahagn on 12/8/20.
//

#ifndef ORB_SLAM3_INCLUDE_ATLAS_H_
#define ORB_SLAM3_INCLUDE_ATLAS_H_
// === stl ===
#include <memory>

// === orb-slam3 ===

#include "map.h"

namespace orb_slam3 {
namespace map {

class Atlas {
 public:
  Atlas();
  ~Atlas();

  Map *GetCurrentMap();
  void CreateNewMap();
  size_t GetMapCount() const;
  const std::unordered_set<map::Map *> &GetMaps() const;

  void SetCurrentMap(map::Map *map);
  void Serialize(std::ostream &ostream) const;
  void Deserialize(std::istream &istream);
 private:
  Map *current_map_;
  std::unordered_set<map::Map *> maps_;
};

}
}
#endif //ORB_SLAM3_INCLUDE_ATLAS_H_
