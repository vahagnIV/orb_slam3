//
// Created by vahagn on 01/07/2021.
//

#ifndef ORB_SLAM3_SRC_FRAME_I_KEY_FRAME_DATABASE_H_
#define ORB_SLAM3_SRC_FRAME_I_KEY_FRAME_DATABASE_H_

#include <frame/key_frame.h>

namespace orb_slam3 {
namespace frame {

class IKeyFrameDatabase {
 public:
  virtual void Append(KeyFrame * keyframe) = 0;
  virtual void DetectNBestCandidates(const frame::KeyFrame * frame,
                                     std::unordered_set<KeyFrame *> & out_loop_candidates,
                                     std::unordered_set<KeyFrame *> & out_merge_candidates,
                                     int count) const = 0;
  virtual ~IKeyFrameDatabase() = default;

};

}
}

#endif //ORB_SLAM3_SRC_FRAME_I_KEY_FRAME_DATABASE_H_
