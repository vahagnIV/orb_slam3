//
// Created by vahagn on 02/02/21.
//

#ifndef ORB_SLAM3_WINDOW_MATCHER_H
#define ORB_SLAM3_WINDOW_MATCHER_H

// === stl ===
#include <unordered_set>

// == orb-slam3 ===
#include <features/ifeature_extractor.h>

namespace orb_slam3 {
namespace features {
namespace matching {

template<typename IteratorType>
class SNNMatcher {
 public:
  typedef std::unordered_map<typename IteratorType::value_type::id_type,
                             typename IteratorType::value_type::iterator::value_type::id_type> MatchMapType;
  typedef typename IteratorType::value_type::iterator::value_type::id_type FromIdType;
  typedef typename IteratorType::value_type::id_type ToIdType;

  SNNMatcher(const precision_t nearest_neighbour_ratio, const unsigned threshold) :
      nearest_neighbour_ratio_(nearest_neighbour_ratio),
      THRESHOLD_(threshold) {}

  void MatchWithIterators(IteratorType to_begin,
                          IteratorType to_end,
                          const IFeatureExtractor * feature_extractor,
                          MatchMapType & out_matches) {

    std::unordered_map<FromIdType, ToIdType> matches_from_to;
    std::unordered_map<FromIdType, unsigned> best_distances_from;

    typename IteratorType::value_type::iterator from_best_it;

    for (auto it_to = to_begin; it_to != to_end; ++it_to) {
      unsigned best_distance = std::numeric_limits<unsigned>::max(),
          second_best_distance = std::numeric_limits<unsigned>::max();
      DescriptorType d1 = it_to->GetDescriptor();
      for (auto it_from = it_to->begin(), it_from_end = it_to->end(); it_from != it_from_end; ++it_from) {
        DescriptorType d2 = it_from->GetDescriptor();
        unsigned distance = feature_extractor->ComputeDistance(d1, d2);
        if (distance > THRESHOLD_)
          continue;

        if (distance < best_distance) {
          from_best_it = it_from;
          second_best_distance = best_distance;
          best_distance = distance;
        } else if (distance < second_best_distance)
          second_best_distance = distance;
      }
      if (best_distance < nearest_neighbour_ratio_ * second_best_distance) {
        typename decltype(best_distances_from)::iterator best_dist_it = best_distances_from.find(from_best_it->GetId());
        if (best_dist_it != best_distances_from.end()) {
          if (best_dist_it->second > best_distance) {
            assert(matches_from_to.find(from_best_it->GetId()) != matches_from_to.end());
            assert(out_matches.find(matches_from_to[from_best_it->GetId()]) != out_matches.end());
            out_matches.erase(matches_from_to[from_best_it->GetId()]);
          } else continue;
        }

        best_distances_from[from_best_it->GetId()] = best_distance;
        matches_from_to[from_best_it->GetId()] = it_to->GetId();
        out_matches[it_to->GetId()] = from_best_it->GetId();
      }
    }

#ifndef NDEBUG
    std::unordered_set<FromIdType> validation_set;
    for (auto match: out_matches) {
      assert(validation_set.find(match.second) == validation_set.end());
      validation_set.insert(match.second);
    }
  }
#endif

 private:
  const precision_t nearest_neighbour_ratio_;
  const unsigned THRESHOLD_;
};

}
}
}

#endif //ORB_SLAM3_WINDOW_MATCHER_H
