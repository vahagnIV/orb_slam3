//
// Created by vahagn on 02/03/21.
//

#ifndef ORB_SLAM3_I_FEATURE_DATABASE_H
#define ORB_SLAM3_I_FEATURE_DATABASE_H
#include <orb_vocabulary.h>
#include "frame_base.h"
#include "map/map.h"
#include "frame.h"

namespace orb_slam3 {
namespace frame {

class KeyFrameDatabase {
 public:
  KeyFrameDatabase(const ORBVocabulary & voc);

 public:
  // Loop and Merge Detection
  void DetectCandidates(const FrameBase * frame,
                        float minScore,
                        std::unordered_set<FrameBase *> & out_loop_candidates,
                        std::unordered_set<FrameBase *> & out_merge_candidates);
  void DetectBestCandidates(const FrameBase * frame,
                            std::unordered_set<FrameBase *> & out_loop_candidates,
                            std::unordered_set<FrameBase *> & out_merge_candidates,
                            int nMinWords);
  void DetectNBestCandidates(const FrameBase * frame,
                             std::unordered_set<FrameBase *> & out_loop_candidates,
                             std::unordered_set<FrameBase *> & out_merge_candidates,
                             int count);

  // Relocalization
  std::unordered_set<KeyFrame *> DetectRelocalizationCandidates(Frame * F, map::Map * map);

  //Helper member functions//
 private:
  static void FilterRelocalizationCandidatesByWordSharingAcceptableScore(const Frame * frame,
                                                                         const std::unordered_map<KeyFrame *,
                                                                                                  std::size_t> & word_sharing_key_frames,
                                                                         std::unordered_map<KeyFrame *,
                                                                                            precision_t> & out_key_frame_reloc_scores);
  static void FilterRelocalizationCandidatesByCovisibility(const std::unordered_map<KeyFrame *,
                                                                                    precision_t> & key_frame_reloc_scores,
                                                           const std::unordered_map<KeyFrame *,
                                                                                    std::size_t> & word_sharing_key_frames,
                                                           std::unordered_set<KeyFrame *> & out_relocalization_candidates);

 private:
  std::vector<std::unordered_set<KeyFrame *> > inverted_file_;
  const ORBVocabulary * vocabulary_;
};

}
}
#endif //ORB_SLAM3_I_FEATURE_DATABASE_H
