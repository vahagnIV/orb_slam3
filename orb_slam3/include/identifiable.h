//
// Created by vahagn on 09.03.21.
//

#ifndef ORB_SLAM3_ORB_SLAM3_INCLUDE_IDENTIFIABLE_H_
#define ORB_SLAM3_ORB_SLAM3_INCLUDE_IDENTIFIABLE_H_
// === orb-slam3 ===
#include <cstddef>

namespace orb_slam3 {

class Identifiable {
 public:
  Identifiable();
  static size_t GetNextId() { return next_id_; }
 protected:
  const size_t id_;
 private:
  static size_t next_id_;
};

}
#endif //ORB_SLAM3_ORB_SLAM3_INCLUDE_IDENTIFIABLE_H_
