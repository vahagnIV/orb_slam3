//
// Created by vahagn on 01/02/21.
//

#ifndef ORB_SLAM3_IMATCHER_H
#define ORB_SLAM3_IMATCHER_H

// == orb-slam3 ===
#include <features/features.h>
#include <features/match.h>

namespace orb_slam3 {
namespace features {
namespace matching {

class IMatcher {
 public:
  virtual void Match(const features::Features &features1,
                     const features::Features &features2,
                     std::vector<Match> &out_matches,
                     size_t window_size) const = 0;

  virtual ~IMatcher() = default;

};

}
}
}
#endif //ORB_SLAM3_IMATCHER_H
