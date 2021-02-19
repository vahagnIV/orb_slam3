//
// Created by vahagn on 2/19/21.
//

#ifndef ORB_SLAM3_ORB_SLAM3_INCLUDE_FEATURES_MATCH_H_
#define ORB_SLAM3_ORB_SLAM3_INCLUDE_FEATURES_MATCH_H_

#include <cstddef>

namespace orb_slam3 {
namespace features {

struct Match {
  Match(size_t to_idx, size_t from_idx) : to_idx(to_idx), from_idx(from_idx) {}
  size_t to_idx;
  size_t from_idx;
};

}
}

#endif //ORB_SLAM3_ORB_SLAM3_INCLUDE_FEATURES_MATCH_H_
