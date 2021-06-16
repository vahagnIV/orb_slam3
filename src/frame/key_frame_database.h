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
  void DetectCandidates(FrameBase * frame,
                        float minScore,
                        std::unordered_set<FrameBase *> & out_loop_candidates,
                        std::unordered_set<FrameBase *> & out_merge_candidates);
  void DetectBestCandidates(FrameBase * frame,
                            std::unordered_set<FrameBase *> & out_loop_candidates,
                            std::unordered_set<FrameBase *> & out_merge_candidates,
                            int nMinWords);
  void DetectNBestCandidates(FrameBase * frame,
                             std::unordered_set<FrameBase *> & out_loop_candidates,
                             std::unordered_set<FrameBase *> & out_merge_candidates,
                             int count);

  // Relocalization
  std::vector<KeyFrame *> DetectRelocalizationCandidates(Frame * F, map::Map * map);
 private:
  std::vector<std::unordered_set<KeyFrame *> > inverted_file;
  const ORBVocabulary * vocabulary;
};

}
}
#endif //ORB_SLAM3_I_FEATURE_DATABASE_H