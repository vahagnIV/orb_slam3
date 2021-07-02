//
// Created by vahagn on 02.07.21.
//

#ifndef ORB_SLAM3_SRC_FRAME_DATABASE_DBOW2_D_BO_W_2_DATABASE_H_
#define ORB_SLAM3_SRC_FRAME_DATABASE_DBOW2_D_BO_W_2_DATABASE_H_

#include <frame/database/ikey_frame_database.h>

namespace orb_slam3 {
namespace frame {

class DBoW2Database : public IKeyFrameDatabase {
 public:
  void Append(KeyFrame * keyframe) override;
  void DetectNBestCandidates(const frame::KeyFrame * frame,
                             std::unordered_set<KeyFrame *> & out_loop_candidates,
                             std::unordered_set<KeyFrame *> & out_merge_candidates,
                             int count) const override;
 public:

 private:
  std::vector<std::list<KeyFrame *> > inverted_file_;

};

}
}

#endif //ORB_SLAM3_SRC_FRAME_DATABASE_DBOW2_D_BO_W_2_DATABASE_H_
