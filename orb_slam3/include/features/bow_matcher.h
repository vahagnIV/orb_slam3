//
// Created by vahagn on 12/03/2021.
//

#ifndef ORB_SLAM3_ORB_SLAM3_INCLUDE_FEATURES_BOW_MATCHER_H_
#define ORB_SLAM3_ORB_SLAM3_INCLUDE_FEATURES_BOW_MATCHER_H_

// === dbow2 ===
#include <DBoW2/FeatureVector.h>

// === orb-slam3 ===
#include <features/features.h>
#include <features/second_nearest_neighbor_matcher.h>

namespace orb_slam3 {
namespace features {

class BowMatcher {
 public:
  BowMatcher(precision_t nearest_neighbour_ratio) : nearest_neighbour_ratio_(nearest_neighbour_ratio) {}
  void Match(const DBoW2::FeatureVector &fv_from,
             const Features &features_from,
             const DBoW2::FeatureVector &fv_to,
             const Features &features_to,
             const std::vector<bool> &mask_from,
             const std::vector<bool> &mask_to,
             std::vector<Match> &out_matches) const;
 private:
  static bool BelongToTheSameNode(DBoW2::FeatureVector::const_iterator &it1, DBoW2::FeatureVector::const_iterator &it2);
 private:
  precision_t nearest_neighbour_ratio_;

};

}
}
#endif //ORB_SLAM3_ORB_SLAM3_INCLUDE_FEATURES_BOW_MATCHER_H_
