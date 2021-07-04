//
// Created by vahagn on 02.07.21.
//

#include "dbo_w2_database.h"
#include <features/handlers/DBoW2/dbo_w2_handler.h>

namespace orb_slam3 {
namespace frame {

void DBoW2Database::Append(KeyFrame * keyframe) {
  auto feature_handler = dynamic_cast<const features::handlers::DBoW2Handler *>(keyframe->GetFeatureHandler().get());
  assert(nullptr != feature_handler);
  for (auto wid: feature_handler->GetFeatureVector()) {
    inverted_file_[wid.first].push_back(keyframe);
  }
}

void DBoW2Database::DetectNBestCandidates(const frame::KeyFrame * keyframe,
                                          unordered_set<KeyFrame *> & out_loop_candidates,
                                          unordered_set<KeyFrame *> & out_merge_candidates,
                                          int count) const {

  auto feature_handler = dynamic_cast<const features::handlers::DBoW2Handler *>(keyframe->GetFeatureHandler().get());
  assert(nullptr != feature_handler);
  WordSharingKeyFrameMap word_sharing_key_frames;
  SearchWordSharingKeyFrames(keyframe, feature_handler, word_sharing_key_frames);
  if (word_sharing_key_frames.empty()) return;
  size_t max_sharing_words = FindMaxSharingWord(word_sharing_key_frames);
  size_t min_common_words = 0.8 * max_sharing_words;

}

void DBoW2Database::SearchWordSharingKeyFrames(const KeyFrame * keyframe,
                                               const features::handlers::DBoW2Handler * handler,
                                               DBoW2Database::WordSharingKeyFrameMap & out_word_sharing_key_frames) const {
  out_word_sharing_key_frames.clear();
  auto neighbours = keyframe->GetCovisibilityGraph().GetCovisibleKeyFrames();
  for (auto wit: handler->GetFeatureVector()) {
    assert(wit.first < inverted_file_.size());
    for (auto kf: inverted_file_[wit.first]) {
      if (neighbours.find(kf) != neighbours.end())
        continue;
      ++out_word_sharing_key_frames[kf];
    }
  }
}

size_t DBoW2Database::FindMaxSharingWord(const DBoW2Database::WordSharingKeyFrameMap & word_sharing_key_frames) const {
  size_t max = 0;
  for (const auto & wit: word_sharing_key_frames) {
    if (max < wit.second)
      max = wit.second;
  }
  return max;
}

}
}