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
  auto neighbours = keyframe->GetCovisibilityGraph().GetCovisibleKeyFrames();
  for (auto wit: feature_handler->GetFeatureVector()) {
    assert(wit.first < inverted_file_.size());
    for (auto kf: inverted_file_[wit.first]) {
      if (neighbours.find(kf) != neighbours.end())
        continue;
    }
  }

}

void DBoW2Database::SearchWordSharingKeyFrames(const features::handlers::DBoW2Handler * handler,
                                               DBoW2Database::WordSharingKeyFrameMap & out_word_sharing_key_frames) {
  for (auto word : handler->GetBowVector()) {
    std::list<KeyFrame *> key_frames = inverted_file_[word.first];
    for (auto key_frame : key_frames) {
      out_word_sharing_key_frames[key_frame]++;
    }
  }
}

}
}