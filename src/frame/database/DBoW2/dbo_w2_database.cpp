//
// Created by vahagn on 02.07.21.
//

#include "dbo_w2_database.h"
#include <features/handlers/DBoW2/dbo_w2_handler.h>
#include <serialization/serialization_context.h>

namespace orb_slam3 {
namespace frame {

DBoW2Database::DBoW2Database(const features::BowVocabulary *vocabulary) : inverted_file_(vocabulary->size()) {

}

DBoW2Database::DBoW2Database(istream &istream, serialization::SerializationContext &context) {
  size_t inverted_file_size;
  READ_FROM_STREAM(inverted_file_size, istream);
  inverted_file_.resize(inverted_file_size);
  for (size_t i = 0; i < inverted_file_size; ++i) {
    size_t inv_map_size;
    READ_FROM_STREAM(inv_map_size, istream);
    while (inv_map_size--) {
      size_t kf_id;
      decltype(inverted_file_)::value_type::mapped_type index;
      READ_FROM_STREAM(kf_id, istream);
      READ_FROM_STREAM(index, istream);
      inverted_file_[i][context.kf_id[kf_id]] = index;
    }
  }
  std::cout << "Loaded" << std::endl;
}


KeyframeDatabaseType DBoW2Database::Type() const {
  return DBoW2DB;
}

void DBoW2Database::Serialize(ostream &ostream) const {
  size_t inverted_file_size = inverted_file_.size();
  WRITE_TO_STREAM(inverted_file_size, ostream);
  for (const auto &inv_map: inverted_file_) {
    size_t inv_map_size = inv_map.size();
    WRITE_TO_STREAM(inv_map_size, ostream);
    for (const auto &kv: inv_map) {
      size_t kf_id = kv.first->Id();
      WRITE_TO_STREAM(kf_id, ostream);
      WRITE_TO_STREAM(kv.second, ostream);
    }
  }
}

void DBoW2Database::Append(KeyFrame *keyframe) {
  auto feature_handler = dynamic_cast<const features::handlers::DBoW2Handler *>(keyframe->GetFeatureHandler().get());
  assert(nullptr != feature_handler);
  std::unique_lock<std::mutex> lock(mutex_);
  for (auto wf: feature_handler->GetWordFrequencies()) {
    assert(wf.first < inverted_file_.size());
    inverted_file_[wf.first][keyframe] = wf.second;
  }
}

void DBoW2Database::DetectNBestCandidates(const BaseFrame * keyframe,
                                          KeyFrameSet & out_loop_candidates,
                                          KeyFrameSet & out_merge_candidates,
                                          size_t count) const {

  auto feature_handler = dynamic_cast<const features::handlers::DBoW2Handler *>(keyframe->GetFeatureHandler().get());
  assert(nullptr != feature_handler);
  WordSharingKeyFrameMap word_sharing_key_frames;
  SearchWordSharingKeyFrames(keyframe, feature_handler, word_sharing_key_frames);
  if (word_sharing_key_frames.empty()) return;
  ScoreKeyFrameMap score_key_frame_map;
  FilterCandidatesByWordSharingAcceptableScore(feature_handler, word_sharing_key_frames, score_key_frame_map);
  FilterCandidatesByCovisibility(keyframe->GetMap(),
                                 score_key_frame_map,
                                 word_sharing_key_frames,
                                 out_loop_candidates,
                                 out_merge_candidates, count);

}

void DBoW2Database::SearchWordSharingKeyFrames(const BaseFrame * keyframe,
                                               const features::handlers::DBoW2Handler * handler,
                                               WordSharingKeyFrameMap & out_word_sharing_key_frames) const {
  out_word_sharing_key_frames.clear();
  KeyFrameSet neighbours;

  {
    // TODO: find a better way
    auto kf = dynamic_cast<const KeyFrame *> (keyframe);
    if (nullptr != kf)
      neighbours = kf->GetCovisibilityGraph().GetCovisibleKeyFrames();
  }

  for (const auto & b: handler->GetWordFrequencies()) {
    assert(b.first < inverted_file_.size());
    for (auto kf_count: inverted_file_[b.first]) {
      auto kf = kf_count.first;
      out_word_sharing_key_frames[kf] += kf_count.second * b.second;
    }
  }
}

size_t DBoW2Database::FindMaxSharingWord(const WordSharingKeyFrameMap & word_sharing_key_frames) {
  size_t max = 0;
  for (const auto & wit: word_sharing_key_frames) {
    if (max < wit.second)
      max = wit.second;
  }
  return max;
}

void DBoW2Database::FilterCandidatesByWordSharingAcceptableScore(const features::handlers::DBoW2Handler * handler,
                                                                 const WordSharingKeyFrameMap & word_sharing_key_frames,
                                                                 ScoreKeyFrameMap & out_key_frame_reloc_scores) {
  size_t max_sharing_words = FindMaxSharingWord(word_sharing_key_frames);
  size_t min_common_words = 0.8 * max_sharing_words;
  for (auto word_sharing_key_frame_pair: word_sharing_key_frames) {
    KeyFrame * word_sharing_key_frame = word_sharing_key_frame_pair.first;
    if (word_sharing_key_frame_pair.second > min_common_words) {
      const auto
          kf_handler =
          dynamic_cast<const features::handlers::DBoW2Handler * >(word_sharing_key_frame_pair.first->GetFeatureHandler().get());
      assert(nullptr != kf_handler);
      assert(handler->GetVocabulary() == kf_handler->GetVocabulary());
      precision_t word_sharing_key_frame_voc_score =
          handler->GetVocabulary()->score(handler->GetBowVector(), kf_handler->GetBowVector());
      out_key_frame_reloc_scores[word_sharing_key_frame] = word_sharing_key_frame_voc_score;
    }
  }
}

void DBoW2Database::FilterCandidatesByCovisibility(const map::Map * current_map,
                                                   const ScoreKeyFrameMap & key_frame_reloc_scores,
                                                   const WordSharingKeyFrameMap & word_sharing_key_frames,
                                                   KeyFrameSet & out_loop_candidates,
                                                   KeyFrameSet & out_merge_candidates,
                                                   size_t count) {
  std::unordered_map<KeyFrame *, precision_t> acc_score_and_matches;
  precision_t best_acc_score = 0;
  // Lets now accumulate score by covisibility
  for (auto key_frame_similarity_score : key_frame_reloc_scores) {
    KeyFrame * match_key_frame = key_frame_similarity_score.first;
    std::unordered_set<KeyFrame *>
        top_covisible_neighbour_key_frames = match_key_frame->GetCovisibilityGraph().GetCovisibleKeyFrames(10);
    precision_t best_score = key_frame_similarity_score.second;
    precision_t acc_score = best_score;
    KeyFrame * best_key_frame = match_key_frame;
    for (auto covisible_neighbour_key_frame : top_covisible_neighbour_key_frames) {
      if (word_sharing_key_frames.find(covisible_neighbour_key_frame) == word_sharing_key_frames.end()) {
        continue;
      }
      auto key_frame_reloc_score = key_frame_reloc_scores.find(covisible_neighbour_key_frame);
      if (key_frame_reloc_score != key_frame_reloc_scores.end()) {
        acc_score += key_frame_reloc_score->second;
        if (key_frame_reloc_score->second > best_score) {
          best_key_frame = covisible_neighbour_key_frame;
          best_score = key_frame_reloc_score->second;
        }
      }
    }
    acc_score_and_matches[best_key_frame] = acc_score;
    if (acc_score > best_acc_score)
      best_acc_score = acc_score;
  }
  // Return all those keyframes with a score higher than 0.75*bestScore
  precision_t min_acceptable_score = 0.75 * best_acc_score;
  for (auto acc_score_and_match_pair : acc_score_and_matches) {
    const precision_t & score = acc_score_and_match_pair.second;
    if (score > min_acceptable_score) {
      KeyFrame * match_key_frame = acc_score_and_match_pair.first;
      if (match_key_frame->GetMap() == current_map && out_loop_candidates.size() < count)
        out_loop_candidates.insert(match_key_frame);
      else if (out_merge_candidates.size() < count)
        out_merge_candidates.insert(match_key_frame);
    }
  }
}

void DBoW2Database::DetectRelocCandidates(const BaseFrame * keyframe,
                                          IKeyFrameDatabase::KeyFrameSet & out_reloc_candidates) const {
  KeyFrameSet tmp;
  DetectNBestCandidates(keyframe, out_reloc_candidates, tmp, std::numeric_limits<std::size_t>::max());
}

void DBoW2Database::Erase(KeyFrame * key_frame) {
  auto feature_handler = dynamic_cast<const features::handlers::DBoW2Handler *>(key_frame->GetFeatureHandler().get());
  assert(nullptr != feature_handler);
  std::unique_lock<std::mutex> lock(mutex_);
  for (auto wid: feature_handler->GetBowVector()) {
    assert(wid.first < inverted_file_.size());
    inverted_file_[wid.first].erase(key_frame);
  }
}

}
}