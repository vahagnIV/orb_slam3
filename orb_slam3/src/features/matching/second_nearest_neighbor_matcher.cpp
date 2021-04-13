//
// Created by vahagn on 02/02/21.
//

// == orb-slam3 ===
#include <features/matching/second_nearest_neighbor_matcher.h>
#include <features/matching/iterators/bow_iterator.h>
#include <features/feature_utils.h>

namespace orb_slam3 {
namespace features {
namespace matching {

const int SNNMatcher::TH_HIGH = 100;
const unsigned SNNMatcher::TH_LOW = 50;

SNNMatcher::SNNMatcher(const precision_t nearest_neighbour_ratio) : nearest_neighbour_ratio_(nearest_neighbour_ratio) {
}

template<typename IteratorType>
void SNNMatcher::MatchWithIterator(const DescriptorSet & descriptors_to,
                                   const DescriptorSet & descriptors_from,
                                   vector<features::Match> & out_matches,
                                   iterators::IJointDescriptorIterator<IteratorType> *iterator,
                                   std::vector<validators::IIndexValidator *> validators,
                                   validators::IMatchResultValidator *result_validator) {
  std::vector<int> matches;
  size_t nmatches =
      MatchWithIteratorInternal(descriptors_to, descriptors_from, matches, iterator, validators, result_validator);
  out_matches.reserve(nmatches);
  for (int i = 0; i < descriptors_to.rows(); ++i) {
    if (matches[i] >= 0)
      out_matches.emplace_back(i, matches[i]);
  }
}

template<typename IteratorType>
size_t SNNMatcher::MatchWithIteratorInternal(const DescriptorSet & descriptors_to,
                                             const DescriptorSet & descriptors_from,
                                             std::vector<int> & out_matches,
                                             iterators::IJointDescriptorIterator<IteratorType> *iterator,
                                             std::vector<validators::IIndexValidator *> validators,
                                             validators::IMatchResultValidator *result_validator) {
  size_t number_of_matches = 0;
  out_matches.resize(descriptors_to.rows(), -1);
  std::vector<unsigned> best_distances_from(descriptors_from.rows(), std::numeric_limits<unsigned>::max());
  std::vector<int> matches_from_to(descriptors_from.size(), -1);

  for (; iterator->IsValid(); ++(*iterator)) {
    size_t to_id = iterator->IdxTo(), best_from_idx;
    for (auto validator: validators)
      if (validator && !validator->ValidateIdxTo(to_id))
        continue;

    unsigned best_distance = std::numeric_limits<unsigned>::max(),
        best_distance2 = std::numeric_limits<unsigned>::max();
    for (size_t idx_from: *iterator) {

      bool indices_are_valid = true;
      for (auto validator: validators) {
        if (validator && !validator->ValidateIindices(to_id, idx_from)) {
          indices_are_valid = false;
          break;
        }
      }
      if (!indices_are_valid)
        continue;

      unsigned dist = DescriptorDistance(descriptors_to.row(to_id), descriptors_from.row(idx_from));
      if (dist < best_distance) {
        best_distance2 = best_distance;
        best_distance = dist;
        best_from_idx = idx_from;
      } else if (dist < best_distance2) {
        best_distance2 = dist;
      }
    }
    if (best_distance <= TH_LOW && best_distance < (float) best_distance2 * nearest_neighbour_ratio_) {
      if (best_distance2 > best_distances_from[best_from_idx])
        continue;
      if (matches_from_to[best_from_idx] > 0)
        out_matches[matches_from_to[best_from_idx]] = -1;
      else
        ++number_of_matches;
      out_matches[to_id] = best_from_idx;
      matches_from_to[best_from_idx] = to_id;
    } else
      out_matches[to_id] = -1;
  }
  if (result_validator)
    return number_of_matches - result_validator->Validate(out_matches);
  return number_of_matches;
}

template void SNNMatcher::MatchWithIterator<std::vector<size_t>::iterator>(const DescriptorSet & descriptors_to,
                                                                           const DescriptorSet & descriptors_from,
                                                                           vector<features::Match> & out_matches,
                                                                           iterators::IJointDescriptorIterator<std::vector<
                                                                               size_t>::iterator> *iterator,
                                                                           std::vector<validators::IIndexValidator *>,
                                                                           validators::IMatchResultValidator *);

template void SNNMatcher::MatchWithIterator<iterators::FeatureVectorTraverseIterator>(const DescriptorSet & descriptors_to,
                                                                                      const DescriptorSet & descriptors_from,
                                                                                      vector<features::Match> & out_matches,
                                                                                      iterators::IJointDescriptorIterator<
                                                                                          iterators::FeatureVectorTraverseIterator> *iterator,
                                                                                      std::vector<validators::IIndexValidator *>,
                                                                                      validators::IMatchResultValidator *);

}
}
}