//
// Created by vahagn on 01/07/2021.
//

#ifndef ORB_SLAM3_SRC_FRAME_I_KEY_FRAME_DATABASE_H_
#define ORB_SLAM3_SRC_FRAME_I_KEY_FRAME_DATABASE_H_

#include <frame/key_frame.h>
#include "keyframe_database_type.h"

namespace orb_slam3 {
namespace frame {

class IKeyFrameDatabase {
 public:
  virtual ~IKeyFrameDatabase() = default;
 public:
  typedef std::unordered_set<KeyFrame *> KeyFrameSet;
  virtual KeyframeDatabaseType Type() const = 0;
  virtual void Serialize(std::ostream &ostream) const = 0;
  virtual void Append(KeyFrame *keyframe) = 0;
  virtual void Erase(KeyFrame *key_frame) = 0;
  virtual void DetectNBestCandidates(const BaseFrame *frame,
                                     KeyFrameSet &out_loop_candidates,
                                     KeyFrameSet &out_merge_candidates,
                                     size_t count) const = 0;
  virtual void DetectRelocCandidates(const BaseFrame *keyframe,
                                     KeyFrameSet &out_reloc_candidates) const = 0;
};

}
}

#endif //ORB_SLAM3_SRC_FRAME_I_KEY_FRAME_DATABASE_H_
