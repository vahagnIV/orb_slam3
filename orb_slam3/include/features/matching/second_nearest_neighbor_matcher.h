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

namespace orb_slam3 {
namespace features {
namespace matching {

class SNNMatcher {
 public:
  SNNMatcher(const precision_t nearest_neighbour_ratio);

  template<typename IteratorType>
  void MatchWithIterator(const DescriptorSet &descriptors_to,
                         const DescriptorSet &descriptors_from,
                         std::vector<features::Match> &out_matches,
                         iterators::IJointDescriptorIterator<IteratorType> *iterator,
                         validators::IIndexValidator *validator = nullptr,
                         validators::IMatchResultValidator *result_validator = nullptr);

 private:
  template<typename IteratorType>
  size_t MatchWithIteratorInternal(const DescriptorSet &descriptors_to,
                                   const DescriptorSet &descriptors_from,
                                   std::vector<int> &out_matches,
                                   iterators::IJointDescriptorIterator<IteratorType> *iterator,
                                   validators::IIndexValidator *validator,
                                   validators::IMatchResultValidator *result_validator);
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
