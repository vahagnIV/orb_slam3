//
// Created by vahagn on 12/03/2021.
//

#ifndef ORB_SLAM3_ORB_SLAM3_INCLUDE_FEATURES_BOW_MATCHER_H_
#define ORB_SLAM3_ORB_SLAM3_INCLUDE_FEATURES_BOW_MATCHER_H_

// === stl ===
#include <unordered_set>

// === dbow2 ===
#include <DBoW2/FeatureVector.h>

// === orb-slam3 ===
#include <features/features.h>
#include <features/second_nearest_neighbor_matcher.h>

namespace orb_slam3 {
namespace features {

class BowMatcher {
 public:
  /*!
   * Constructor
   * @param nearest_neighbour_ratio The ratio threshold for nearest neighbour matching
   */
  BowMatcher(precision_t nearest_neighbour_ratio) : nearest_neighbour_ratio_(nearest_neighbour_ratio) {}

  /*!
   * Matches using Bag of Words.
   * @param features_from Features container for the first images.
   * @param features_to Features container for the second images.
   * @param mask_from Mask for the first container. The feature should be matched if its id exists in mask_from.
   * @param mask_to Negative mask for the second container. The feature should NOT be matched if its id is in mask_to.
   * @param out_matches The output vector of matches.
   */
  void Match(const Features &features_from,
             const Features &features_to,
             const std::unordered_set<size_t> &mask_from,
             const std::unordered_set<size_t> &mask_to,
             std::vector<Match> &out_matches) const;
 private:
  precision_t nearest_neighbour_ratio_;

};

}
}
#endif //ORB_SLAM3_ORB_SLAM3_INCLUDE_FEATURES_BOW_MATCHER_H_
