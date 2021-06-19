//
// Created by suren on 18.05.21.
//

#include <orb_vocabulary.h>
#include "key_frame_database.h"
#include "frame.h"
#include "key_frame.h"

namespace orb_slam3 {
namespace frame {

KeyFrameDatabase::KeyFrameDatabase(const ORBVocabulary & voc) : vocabulary_(&voc) {
  inverted_file_.resize(voc.size());
}

std::unordered_set<KeyFrame *> KeyFrameDatabase::DetectRelocalizationCandidates(Frame * frame, map::Map * map) {
  Frame::WordSharingKeyFrameMap word_sharing_key_frames;
  frame->SearchWordSharingKeyFrames(inverted_file_, word_sharing_key_frames);
  if (word_sharing_key_frames.empty())
    return std::unordered_set<KeyFrame *>();

  // Only compare against those keyframes that share enough words
  std::unordered_map<KeyFrame *, precision_t> key_frame_reloc_scores;
  FilterRelocalizationCandidatesByWordSharingAcceptableScore(frame, word_sharing_key_frames, key_frame_reloc_scores);

  if (key_frame_reloc_scores.empty()) {
    return std::unordered_set<KeyFrame *>();
  }
  std::unordered_set<KeyFrame *> relocalization_candidates;
  FilterRelocalizationCandidatesByCovisibility(key_frame_reloc_scores,
                                               word_sharing_key_frames,
                                               relocalization_candidates);
  return relocalization_candidates;
}

void KeyFrameDatabase::FilterRelocalizationCandidatesByWordSharingAcceptableScore(const Frame * frame,
                                                                                  const std::unordered_map<
                                                                                      KeyFrame *,
                                                                                      std::size_t> & word_sharing_key_frames,
                                                                                  std::unordered_map<KeyFrame *,
                                                                                                     precision_t> & out_key_frame_reloc_scores) {
  size_t max_common_word_count = 0;
  for (auto word_sharing_key_frame_pair : word_sharing_key_frames) {
    if (word_sharing_key_frame_pair.second > max_common_word_count) {
      max_common_word_count = word_sharing_key_frame_pair.second;
    }
  }

  size_t min_acceptable_common_words = max_common_word_count * 4 / 5;
  // Compute similarity score.
  for (auto word_sharing_key_frame_pair: word_sharing_key_frames) {
    KeyFrame * word_sharing_key_frame = word_sharing_key_frame_pair.first;
    if (word_sharing_key_frame_pair.second > min_acceptable_common_words) {
      precision_t word_sharing_key_frame_voc_score = frame->GetSimilarityScore(word_sharing_key_frame);
      out_key_frame_reloc_scores[word_sharing_key_frame] = word_sharing_key_frame_voc_score;
    }
  }
}

void KeyFrameDatabase::FilterRelocalizationCandidatesByCovisibility(const std::unordered_map<KeyFrame *,
                                                                                             precision_t> & key_frame_reloc_scores,
                                                                    const std::unordered_map<KeyFrame *,
                                                                                             std::size_t> & word_sharing_key_frames,
                                                                    std::unordered_set<KeyFrame *> & out_relocalization_candidates) {
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
  out_relocalization_candidates.reserve(acc_score_and_matches.size());
  for (auto acc_score_and_match_pair : acc_score_and_matches) {
    const precision_t & score = acc_score_and_match_pair.second;
    if (score > min_acceptable_score) {
      KeyFrame * match_key_frame = acc_score_and_match_pair.first;
      //TODO:Implement check when GetMap() is ready
/*      if (match_key_frame->GetMap() != map)
        continue;*/
      out_relocalization_candidates.insert(match_key_frame);
    }
  }
}

}
}