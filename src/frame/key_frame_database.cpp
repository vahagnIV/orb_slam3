//
// Created by suren on 18.05.21.
//

#include <orb_vocabulary.h>
#include "key_frame_database.h"
#include "frame.h"
#include "key_frame.h"

namespace orb_slam3 {
namespace frame {

KeyFrameDatabase::KeyFrameDatabase(const ORBVocabulary & voc) : vocabulary(&voc) {
  inverted_file.resize(voc.size());
}

vector<KeyFrame *> KeyFrameDatabase::DetectRelocalizationCandidates(Frame * frame, map::Map * map) {
  list<KeyFrame *> word_sharing_key_frames;
  frame->SearchWordSharingKeyFrames(inverted_file, word_sharing_key_frames);
  if (word_sharing_key_frames.empty())
    return vector<KeyFrame *>();

  // Only compare against those keyframes that share enough words
  size_t max_common_words = 0;
  for (list<KeyFrame *>::iterator lit = word_sharing_key_frames.begin(), lend = word_sharing_key_frames.end();
       lit != lend; lit++) {
    KeyFrame * word_sharing_key_frame = * lit;
    if (word_sharing_key_frame->common_words_count > max_common_words) {
      max_common_words = word_sharing_key_frame->common_words_count;
    }
  }

  int min_common_words = max_common_words * 0.8f;

  list<pair<float, KeyFrame *> > score_and_match_pairs;

  size_t number_of_scores = 0;

  // Compute similarity score.
  for (list<KeyFrame *>::iterator lit = word_sharing_key_frames.begin(), lend = word_sharing_key_frames.end();
       lit != lend; lit++) {
    KeyFrame * word_sharing_key_frame = *lit;

    if (word_sharing_key_frame->common_words_count > min_common_words) {
      number_of_scores++;
      float word_sharing_key_frame_voc_score = frame->GetSimilarityScore(word_sharing_key_frame);
      word_sharing_key_frame->reloc_score = word_sharing_key_frame_voc_score;
      score_and_match_pairs.push_back(make_pair(word_sharing_key_frame_voc_score, word_sharing_key_frame));
    }
  }

  if (score_and_match_pairs.empty())
    return vector<KeyFrame *>();

  list<pair<float, KeyFrame *> > acc_score_and_match_pairs;
  float best_acc_score = 0;

  // Lets now accumulate score by covisibility
  for (list<pair<float, KeyFrame *> >::iterator it = score_and_match_pairs.begin(), itend = score_and_match_pairs.end(); it != itend;
       it++) {
    KeyFrame * match_key_frame = it->second;
    std::unordered_set<KeyFrame *> top_covisible_neighbour_key_frames = match_key_frame->GetCovisibilityGraph().GetCovisibleKeyFrames(10);

    float best_score = it->first;
    float acc_score = best_score;
    KeyFrame * best_key_frame = match_key_frame;
    for (std::unordered_set<KeyFrame *>::iterator vit = top_covisible_neighbour_key_frames.begin(), vend = top_covisible_neighbour_key_frames.end(); vit != vend; vit++) {
      KeyFrame * covisible_neighbour_key_frame = *vit;
      if (covisible_neighbour_key_frame->reloc_query_frame_id != frame->Id())
        continue;

      acc_score += covisible_neighbour_key_frame->reloc_score;
      if (covisible_neighbour_key_frame->reloc_score > best_score) {
        best_key_frame = covisible_neighbour_key_frame;
        best_score = covisible_neighbour_key_frame->reloc_score;
      }

    }
    acc_score_and_match_pairs.push_back(make_pair(acc_score, best_key_frame));
    if (acc_score > best_acc_score)
      best_acc_score = acc_score;
  }

  // Return all those keyframes with a score higher than 0.75*bestScore
  float min_acceptable_score = 0.75f * best_acc_score;
  set<KeyFrame *> already_added_key_frames;
  vector<KeyFrame *> relocalization_candidates;
  relocalization_candidates.reserve(acc_score_and_match_pairs.size());
  for (list<pair<float, KeyFrame *> >::iterator it = acc_score_and_match_pairs.begin(), itend = acc_score_and_match_pairs.end();
       it != itend; it++) {
    const float & score = it->first;
    if (score > min_acceptable_score) {
      KeyFrame * match_key_frame = it->second;
/*      if (match_key_frame->GetMap() != map)
        continue;*/
      if (!already_added_key_frames.count(match_key_frame)) {
        relocalization_candidates.push_back(match_key_frame);
        already_added_key_frames.insert(match_key_frame);
      }
    }
  }

  return relocalization_candidates;
}

}
}