//
// Created by vahagn on 02/03/21.
//

#ifndef ORB_SLAM3_I_FEATURE_DATABASE_H
#define ORB_SLAM3_I_FEATURE_DATABASE_H
#include <frame/frame_base.h>

namespace orb_slam3 {
namespace frame {

class IFrameDatabase {
 public:
  virtual int Append(const std::shared_ptr<FrameBase> & frame) = 0;
  virtual int Erase(const std::shared_ptr<FrameBase> & frame) = 0;
  virtual int Query(const std::shared_ptr<FrameBase> & frame,
                    std::vector<std::shared_ptr<FrameBase>> & out_candidates, size_t max_count) = 0;
};

}
}
#endif //ORB_SLAM3_I_FEATURE_DATABASE_H
