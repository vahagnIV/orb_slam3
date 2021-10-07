//
// Created by vahagn on 02.07.21.
//

#ifndef ORB_SLAM3_SRC_FRAME_DATABASE_DBOW2_D_BO_W_2_DATABASE_H_
#define ORB_SLAM3_SRC_FRAME_DATABASE_DBOW2_D_BO_W_2_DATABASE_H_

#include <frame/database/ikey_frame_database.h>
#include <features/bow_vocabulary.h>
namespace orb_slam3 {
namespace serialization{
class SerializationContext;
}

namespace features {
namespace handlers {
class DBoW2Handler;
}
}

namespace frame {

class DBoW2Database : public IKeyFrameDatabase {
 public:
  DBoW2Database(const features::BowVocabulary * vocabulary);
  DBoW2Database(std::istream & istream, serialization::SerializationContext & context);
 public:
  void Append(KeyFrame * keyframe) override;
  void DetectNBestCandidates(const BaseFrame * keyframe,
                             KeyFrameSet & out_loop_candidates,
                             KeyFrameSet & out_merge_candidates,
                             size_t count) const override;
  void DetectRelocCandidates(const BaseFrame * keyframe, KeyFrameSet & out_reloc_candidates) const override;
  void Erase(KeyFrame * key_frame) override;
  KeyframeDatabaseType Type() const override;
  void Serialize(ostream &ostream) const override;
 private:
  typedef std::unordered_map<KeyFrame *, size_t> WordSharingKeyFrameMap;
  typedef std::unordered_map<KeyFrame *, precision_t> ScoreKeyFrameMap;

  void SearchWordSharingKeyFrames(const BaseFrame * keyframe,
                                  const features::handlers::DBoW2Handler * handler,
                                  DBoW2Database::WordSharingKeyFrameMap & out_word_sharing_key_frames) const;
  static size_t FindMaxSharingWord(const WordSharingKeyFrameMap & word_sharing_key_frames);
  static void FilterCandidatesByWordSharingAcceptableScore(const features::handlers::DBoW2Handler * handler,
                                                           const DBoW2Database::WordSharingKeyFrameMap & word_sharing_key_frames,
                                                           DBoW2Database::ScoreKeyFrameMap & out_key_frame_reloc_scores);
  static void FilterCandidatesByCovisibility(const map::Map * current_map,
                                             const ScoreKeyFrameMap & key_frame_reloc_scores,
                                             const WordSharingKeyFrameMap & word_sharing_key_frames,
                                             KeyFrameSet & out_loop_candidates,
                                             KeyFrameSet & out_merge_candidates,
                                             size_t count);
 private:
  std::vector<std::map<KeyFrame *, std::size_t> > inverted_file_;

};

}
}

#endif //ORB_SLAM3_SRC_FRAME_DATABASE_DBOW2_D_BO_W_2_DATABASE_H_
