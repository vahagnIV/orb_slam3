//
// Created by vahagn on 12/03/2021.
//


// === orb-slam3 ===
#include "features/bow_matcher.h"
#include "features/feature_utils.h"

namespace orb_slam3 {
namespace features {

void BowMatcher::Match(const DBoW2::FeatureVector &fv_from,
                       const Features &features_from,
                       const DBoW2::FeatureVector &fv_to,
                       const Features &features_to,
                       const std::vector<bool> &mask_from,
                       const std::vector<bool> &mask_to,
                       std::vector<features::Match> &out_matches) const {
  // We perform the matching over ORB that belong to the same vocabulary node (at a certain level)
  auto to_it = fv_to.begin();
  auto from_it = fv_from.begin();
  auto to_end = fv_to.end();
  auto from_end = fv_from.end();

  std::vector<int> matches12(features_to.descriptors.size(), -1);
  int number_of_matches = 0;
  while (to_it != to_end && from_it != from_end) {
    if (BelongToTheSameNode(from_it, to_it)) {
      const std::vector<unsigned int> &to_indices = to_it->second;
      const std::vector<unsigned int> &from_indices = from_it->second;
      for (size_t i_to = 0; i_to < to_indices.size(); i_to++) {
        const unsigned int real_idx_to = to_indices[i_to];
        if (!mask_to[real_idx_to])
          continue;

        const auto &descriptor_to = features_to.descriptors.row(real_idx_to);

        unsigned best_distance1 = 256, best_distance2 = 256;
        int best_idx_from;

        for (unsigned int real_idx_from : from_indices) {
          if (mask_from[real_idx_from])
            continue;

          const auto &descriptor_from = features_from.descriptors.row(real_idx_from);

          const unsigned distance = DescriptorDistance(descriptor_to, descriptor_from);

          if (distance < best_distance1) {
            best_distance2 = best_distance1;
            best_distance1 = distance;
            best_idx_from = real_idx_from;
          } else if (distance < best_distance2) {
            best_distance2 = distance;
          }
        }
        if (best_distance1 <= SNNMatcher::TH_LOW
            && static_cast<float>(best_distance1) < nearest_neighbour_ratio_ * static_cast<float>(best_distance2)) {
          matches12[real_idx_to] = best_idx_from;
          ++number_of_matches;
        }
      }
      to_it++;
      from_it++;

    } else if (to_it->first < from_it->first) {
      to_it = fv_to.lower_bound(from_it->first);
    } else {
      from_it = fv_from.lower_bound(to_it->first);
    }
  }

  number_of_matches -= SNNMatcher::FilterByOrientation(matches12, features_to, features_from);
  out_matches.reserve(number_of_matches);
  for (size_t i_to = 0; i_to < features_to.keypoints.size(); ++i_to) {
    if (matches12[i_to] > 0)
      out_matches.emplace_back(i_to, matches12[i_to]);
  }

}

bool BowMatcher::BelongToTheSameNode(DBoW2::FeatureVector::const_iterator &it1,
                                     DBoW2::FeatureVector::const_iterator &it2) {
  return it1->first == it2->first;
}

}
}