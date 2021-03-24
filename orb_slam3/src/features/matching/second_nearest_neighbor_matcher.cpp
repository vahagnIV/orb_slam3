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
const int SNNMatcher::HISTO_LENGTH = 30;

SNNMatcher::SNNMatcher(const precision_t nearest_neighbour_ratio,
                       const bool check_orientation) : nearest_neighbour_ratio_(nearest_neighbour_ratio),
                                                       check_orientation_(check_orientation) {
}

void SNNMatcher::Match(const features::Features &features_to,
                       const features::Features &features_from,
                       std::vector<features::Match> &out_matches,
                       size_t window_size) const {
  std::vector<int> matches12;
  int matches = SNNMatch(features_to, features_from, matches12, window_size);
  out_matches.reserve(matches);
  for (size_t i = 0; i < matches12.size(); ++i) {
    if (matches12[i] >= 0)
      out_matches.emplace_back(i, matches12[i]);
  }
}

int SNNMatcher::SNNMatch(const features::Features &features1,
                         const features::Features &features2,
                         std::vector<int> &out_matches_12,
                         size_t window_size) const {

  int number_of_matches = 0;
  out_matches_12.resize(features1.Size(), -1);

  std::vector<unsigned> matched_distance(features2.Size(), std::numeric_limits<unsigned>::max());
  std::vector<int> matches21(features2.Size(), -1);

  for (size_t i1 = 0; i1 < features1.Size(); i1++) {
    const KeyPoint &kp1 = features1.keypoints[i1];
    int level1 = kp1.level;
    if (level1 > 0)
      continue;

    std::vector<size_t> f2_indices_in_window;
    if (window_size != 0) {
      features2.ListFeaturesInArea(kp1.X(),
                                   kp1.Y(),
                                   window_size,
                                   level1,
                                   level1,
                                   f2_indices_in_window);
    } else {
      f2_indices_in_window.resize(features2.Size());
      std::iota(f2_indices_in_window.begin(), f2_indices_in_window.end(), 0);
    }

    if (f2_indices_in_window.empty())
      continue;

    auto d1 = features1.descriptors.row(i1);
    int &best_idx2 = out_matches_12[i1];
    unsigned distance;

    FindDescriptorInSet(d1, features2.descriptors, f2_indices_in_window, best_idx2, distance);
    if (best_idx2 < 0)
      continue;
    if (matched_distance[best_idx2] < distance)
      continue;

    matched_distance[best_idx2] = best_idx2;

    matches21[best_idx2] = i1;
    ++number_of_matches;
  }
  if (check_orientation_)
    return number_of_matches - FilterByOrientation(out_matches_12, features1, features2);

  return number_of_matches;

}

void SNNMatcher::FindDescriptorInSet(const DescriptorType &d1,
                                     const DescriptorSet &descriptors2,
                                     const std::vector<size_t> &allowed_inidces,
                                     int &out_idx2,
                                     unsigned &dist) const {

  unsigned best_distance = std::numeric_limits<unsigned>::max();
  unsigned best_distance2 = std::numeric_limits<unsigned>::max();
  int best_idx2 = -1;
  for (const size_t &i2:  allowed_inidces) {

    auto d2 = descriptors2.row(i2);
    unsigned dist = DescriptorDistance(d1, d2);

    if (dist < best_distance) {
      best_distance2 = best_distance;
      best_distance = dist;
      best_idx2 = i2;
    } else if (dist < best_distance2) {
      best_distance2 = dist;
    }
  }

  if (best_distance <= TH_LOW && best_distance < (float) best_distance2 * nearest_neighbour_ratio_) {
    out_idx2 = best_idx2;
    dist = best_distance;
  } else
    out_idx2 = -1;
}

void SNNMatcher::ComputeThreeMaxima(std::vector<int> *histo,
                                    int &ind1,
                                    int &ind2,
                                    int &ind3) {
  int max1 = 0;
  int max2 = 0;
  int max3 = 0;

  for (int i = 0; i < HISTO_LENGTH; i++) {
    const int s = histo[i].size();
    if (s > max1) {
      max3 = max2;
      max2 = max1;
      max1 = s;
      ind3 = ind2;
      ind2 = ind1;
      ind1 = i;
    } else if (s > max2) {
      max3 = max2;
      max2 = s;
      ind3 = ind2;
      ind2 = i;
    } else if (s > max3) {
      max3 = s;
      ind3 = i;
    }
  }

  if (static_cast<float>(max2) < 0.1f * static_cast<float>(max1)) {
    ind2 = -1;
    ind3 = -1;
  } else if (static_cast<float>(max3) < 0.1f * static_cast<float>(max1)) {
    ind3 = -1;
  }
}

void SNNMatcher::ComputeRotationHistogram(std::vector<int> *rotation_histogram,
                                          const std::vector<int> &inout_matches_12,
                                          const Features &features1,
                                          const Features &features2) {
  const precision_t factor = 1.0f / HISTO_LENGTH;
  for (int i = 0; i < HISTO_LENGTH; i++)
    rotation_histogram[i].reserve(500);

  for (size_t i = 0; i < inout_matches_12.size(); ++i) {
    if (inout_matches_12[i] > 0) {

      float rot = features1.keypoints[i].angle - features2.keypoints[inout_matches_12[i]].angle;
      if (rot < 0.0)
        rot += 360.0f;
      int bin = std::round(rot * factor);
      if (bin == HISTO_LENGTH)
        bin = 0;
      assert(bin >= 0 && bin < HISTO_LENGTH);
      rotation_histogram[bin].push_back(i);
    }
  }

}

int SNNMatcher::FilterByOrientation(std::vector<int> &inout_matches_12,
                                    const Features &features1,
                                    const Features &features2) {
  std::vector<int> rotation_histogram[HISTO_LENGTH];
  ComputeRotationHistogram(rotation_histogram, inout_matches_12, features1, features2);
  int number_of_discarded_matches = 0;

  int ind1 = -1;
  int ind2 = -1;
  int ind3 = -1;

  ComputeThreeMaxima(rotation_histogram, ind1, ind2, ind3);

  for (int i = 0; i < HISTO_LENGTH; i++) {

    if (i == ind1 || i == ind2 || i == ind3)
      continue;
    for (size_t j = 0, jend = rotation_histogram[i].size(); j < jend; j++) {
      int idx1 = rotation_histogram[i][j];
      if (inout_matches_12[idx1] >= 0) {
        inout_matches_12[idx1] = -1;
        number_of_discarded_matches++;
      }
    }
  }
  return number_of_discarded_matches;
}

void SNNMatcher::MatchByBoW(const Features &features_from,
                            const Features &features_to,
                            const std::unordered_set<size_t> &mask_from,
                            const std::unordered_set<size_t> &mask_to,
                            std::vector<features::Match> &out_matches) const {

  std::vector<int> matches12(features_to.descriptors.size(), -1);
  int number_of_matches = 0;
  for (auto joint_iterator = features_to.bow_container.Begin(features_from.bow_container);
       joint_iterator != features_to.bow_container.End(); ++joint_iterator) {

    const std::vector<unsigned int> &to_indices = joint_iterator.ToIdx();
    const std::vector<unsigned int> &from_indices = joint_iterator.FromIdx();
    for (size_t i_to = 0; i_to < to_indices.size(); i_to++) {
      const unsigned int real_idx_to = to_indices[i_to];
      if (mask_to.find(real_idx_to) == mask_to.end())
        continue;

      const auto &descriptor_to = features_to.descriptors.row(real_idx_to);

      unsigned best_distance1 = 256, best_distance2 = 256;
      int best_idx_from;

      for (unsigned int real_idx_from : from_indices) {
        if (mask_from.find(real_idx_from) != mask_from.end())
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
      if (best_distance1 <= TH_LOW
          && static_cast<float>(best_distance1) < nearest_neighbour_ratio_ * static_cast<float>(best_distance2)) {
        matches12[real_idx_to] = best_idx_from;
        ++number_of_matches;
      }
    }

  }

  number_of_matches -= FilterByOrientation(matches12, features_to, features_from);
  out_matches.reserve(number_of_matches);
  for (size_t i_to = 0; i_to < features_to.keypoints.size(); ++i_to) {
    if (matches12[i_to] > 0)
      out_matches.emplace_back(i_to, matches12[i_to]);
  }

}

template<typename IteratorType>
void SNNMatcher::MatchWithIterator(const DescriptorSet &descriptors_to,
                                   const DescriptorSet &descriptors_from,
                                   vector<features::Match> &out_matches,
                                   iterators::IJointDescriptorIterator<IteratorType> *iterator) {
  vector<int> matches;
  size_t nmatches = MatchWithIteratorInternal(descriptors_to, descriptors_from, matches, iterator);
  out_matches.reserve(nmatches);
  for (int i = 0; i < descriptors_to.rows(); ++i) {
    if (matches[i] >= 0)
      out_matches.emplace_back(i, matches[i]);
  }
}

template<typename IteratorType>
size_t SNNMatcher::MatchWithIteratorInternal(const DescriptorSet &descriptors_to,
                                             const DescriptorSet &descriptors_from,
                                             std::vector<int> &out_matches,
                                             iterators::IJointDescriptorIterator<IteratorType> *iterator) {
  size_t number_of_matches = 0;
  out_matches.resize(descriptors_to.rows());
  std::vector<unsigned> best_distances_from(descriptors_from.rows(), std::numeric_limits<unsigned>::max());
  std::vector<int> matches_from_to(descriptors_from.size(), -1);
  for (; iterator->IsValid(); ++(*iterator)) {
    size_t to_id = iterator->IdxTo(), best_from_idx;
    unsigned best_distance = std::numeric_limits<unsigned>::max(),
        best_distance2 = std::numeric_limits<unsigned>::max();
    for (size_t idx_from: *iterator) {
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
      ++number_of_matches;
      out_matches[to_id] = best_from_idx;
    } else
      out_matches[to_id] = -1;
  }
//  if(check_orientation_)
//    return number_of_matches - FilterByOrientation(out_matches, )
  return number_of_matches;
}

template void SNNMatcher::MatchWithIterator<std::vector<size_t>::iterator>(const DescriptorSet &descriptors_to,
                                                                           const DescriptorSet &descriptors_from,
                                                                           vector<features::Match> &out_matches,
                                                                           iterators::IJointDescriptorIterator<std::vector<size_t>::iterator> *iterator);

template void SNNMatcher::MatchWithIterator<iterators::FeatureVectorTraverseIterator>(const DescriptorSet &descriptors_to,
                                                                                      const DescriptorSet &descriptors_from,
                                                                                      vector<features::Match> &out_matches,
                                                                                      iterators::IJointDescriptorIterator<iterators::FeatureVectorTraverseIterator> *iterator);

}
}
}