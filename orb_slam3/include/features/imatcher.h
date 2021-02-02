//
// Created by vahagn on 01/02/21.
//

#ifndef ORB_SLAM3_IMATCHER_H
#define ORB_SLAM3_IMATCHER_H
#include <features/features.h>

namespace orb_slam3 {
namespace features {

class IMatcher {
 public:
  virtual int Match(const features::Features & features1,
                    const features::Features & features2,
                    std::vector<int> & out_matches_12) = 0;

  virtual ~IMatcher() = default;

};

}
}
#endif //ORB_SLAM3_IMATCHER_H
