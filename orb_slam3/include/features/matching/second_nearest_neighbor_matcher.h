//
// Created by vahagn on 02/02/21.
//

#ifndef ORB_SLAM3_WINDOW_MATCHER_H
#define ORB_SLAM3_WINDOW_MATCHER_H

// === stl ===
#include <unordered_set>

// == orb-slam3 ===
#include <features/features.h>
#include <features/match.h>
#include <features/matching/iterators/idescriptor_iterator.h>
#include <features/matching/validators/iindex_validator.h>
#include <features/matching/validators/imatch_result_validator.h>
#include <features/feature_utils.h>

namespace orb_slam3 {
namespace features {
namespace matching {

class SNNMatcher {
 public:
  SNNMatcher(const precision_t nearest_neighbour_ratio);

  template<typename IteratorTo>
  void MatchWithIteratorV2(IteratorTo to_begin,
                           IteratorTo to_end,
                           std::unordered_map<typename IteratorTo::ToIdType,
                                              typename IteratorTo::FromIdType> & out_matches) {

    std::unordered_map<typename IteratorTo::FromIdType, typename IteratorTo::ToIdType> matches_from_to;
    std::unordered_map<typename IteratorTo::FromIdType, unsigned> best_distances_from;

    unsigned best_distance = std::numeric_limits<unsigned>::max(),
        second_best_distance = std::numeric_limits<unsigned>::max();

    typename IteratorTo::value_type::iterator best_it;

    for (auto it_to = to_begin; it_to != to_end; ++it_to) {
      DescriptorType d1 = it_to->GetDescriptor();
      for (auto it_from = it_to->begin(), it_from_end = it_to->end(); it_from != it_from_end; ++it_from) {
        DescriptorType d2 = it_from->GetDescriptor();
        unsigned distance = DescriptorDistance(d1, d2);
        if (distance < best_distance) {
          best_it = it_from;
          second_best_distance = best_distance;
          best_distance = distance;
        } else if (distance < second_best_distance)
          second_best_distance = distance;
      }
      if (best_distance < nearest_neighbour_ratio_ * second_best_distance) {
        typename decltype(best_distances_from)::iterator best_dist_it = best_distances_from.find(best_it->GetId());
        if (best_dist_it != best_distances_from.end() && best_dist_it->second > best_distance) {
          out_matches.erase(matches_from_to[best_it->GetId()]);
        }
        best_distances_from[best_it->GetId()] = best_distance;
        matches_from_to[best_it->GetId()] = it_to->GetId();
        out_matches[it_to->GetId()] = best_it->GetId();
      }
    }
  }

  template<typename IteratorType>
  void MatchWithIterator(const DescriptorSet & descriptors_to,
                         const DescriptorSet & descriptors_from,
                         std::vector<features::Match> & out_matches,
                         iterators::IJointDescriptorIterator<IteratorType> * iterator,
                         std::vector<validators::IIndexValidator *> validators = {},
                         validators::IMatchResultValidator * result_validator = nullptr);

 private:
  template<typename IteratorType>
  size_t MatchWithIteratorInternal(const DescriptorSet & descriptors_to,
                                   const DescriptorSet & descriptors_from,
                                   std::vector<int> & out_matches,
                                   iterators::IJointDescriptorIterator<IteratorType> * iterator,
                                   std::vector<validators::IIndexValidator *> validators,
                                   validators::IMatchResultValidator * result_validator);
 public:
  static const unsigned TH_LOW;
  static const int TH_HIGH;

 private:
  const precision_t nearest_neighbour_ratio_;
};

}
}
}

#endif //ORB_SLAM3_WINDOW_MATCHER_H
