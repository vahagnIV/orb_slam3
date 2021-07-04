//
// Created by vahagn on 02.07.21.
//

#ifndef ORB_SLAM3_SRC_FRAME_DATABASE_DBOW2_D_BO_W_2_DATABASE_H_
#define ORB_SLAM3_SRC_FRAME_DATABASE_DBOW2_D_BO_W_2_DATABASE_H_

#include <frame/database/ikey_frame_database.h>

namespace orb_slam3 {

namespace features {
namespace handlers {
class DBoW2Handler;
}
}

namespace frame {

class DBoW2Database : public IKeyFrameDatabase {
 public:
  void Append(KeyFrame * keyframe) override;
  void DetectNBestCandidates(const frame::KeyFrame * frame,
                             std::unordered_set<KeyFrame *> & out_loop_candidates,
                             std::unordered_set<KeyFrame *> & out_merge_candidates,
                             int count) const override;
 private:
  typedef std::unordered_map<KeyFrame *, size_t> WordSharingKeyFrameMap;

  void SearchWordSharingKeyFrames(const KeyFrame * keyframe,
                                  const features::handlers::DBoW2Handler * handler,
                                  DBoW2Database::WordSharingKeyFrameMap & out_word_sharing_key_frames) const;
  size_t FindMaxSharingWord(const WordSharingKeyFrameMap & word_sharing_key_frames) const;
 private:
  std::vector<std::list<KeyFrame *> > inverted_file_;

};

}
}

#endif //ORB_SLAM3_SRC_FRAME_DATABASE_DBOW2_D_BO_W_2_DATABASE_H_
