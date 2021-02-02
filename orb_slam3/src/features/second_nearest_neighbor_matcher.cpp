//
// Created by vahagn on 02/02/21.
//

#include "features/second_nearest_neighbor_matcher.h"

namespace orb_slam3 {
namespace features {

const int SecondNearestNeighborMatcher::TH_HIGH = 100;
const int SecondNearestNeighborMatcher::TH_LOW = 50;
const int SecondNearestNeighborMatcher::HISTO_LENGTH = 30;

SecondNearestNeighborMatcher::SecondNearestNeighborMatcher(const size_t window_size,
                                                           const precision_t nearest_neighbour_ratio,
                                                           const bool check_orientation,
                                                           const precision_t min_X,
                                                           const precision_t min_Y,
                                                           const precision_t grid_element_width_inv,
                                                           const precision_t grid_element_height_inv)
    : window_size_(window_size),
      nearest_neighbour_ratio_(nearest_neighbour_ratio),
      check_orientation_(check_orientation),
      min_X_(min_X),
      min_Y_(min_Y),
      grid_element_width_inv_(grid_element_width_inv),
      grid_element_height_inv_(grid_element_height_inv) {
}

int SecondNearestNeighborMatcher::Match(const features::Features & features1,
                                        const features::Features & features2,
                                        std::vector<int> & out_matches_12) {
  int nmatches = 0;
  out_matches_12.resize(features1.Size(), -1);

  const precision_t factor = 1.0f / HISTO_LENGTH;

  std::vector<int> matched_distance(features1.Size(), std::numeric_limits<int>::max());
  std::vector<int> matches21(features2.Size(), -1);

  std::vector<int> rotation_hostogram[HISTO_LENGTH];
  if (check_orientation_) {
    for (int i = 0; i < HISTO_LENGTH; i++)
      rotation_hostogram[i].reserve(500);
  }

  for (size_t i1 = 0; i1 < features1.Size(); i1++) {
    const KeyPoint & kp1 = features1.undistorted_keypoints[i1];
    int level1 = kp1.level;
    if (level1 > 0)
      continue;

    std::vector<size_t> f2_indices_in_window;
    features2.ListFeaturesInArea(kp1.X(),
                                 kp1.Y(),
                                 window_size_,
                                 level1,
                                 level1,
                                 min_X_,
                                 min_Y_,
                                 grid_element_width_inv_,
                                 grid_element_height_inv_,
                                 f2_indices_in_window);

    if (f2_indices_in_window.empty())
      continue;

    auto d1 = features1.descriptors.row(i1);
    int best_distance = std::numeric_limits<int>::max();
    int best_distance2 = std::numeric_limits<int>::max();
    int best_idx2 = -1;
    for (const size_t & i2:  f2_indices_in_window) {

      auto d2 = features2.descriptors.row(i2);
      int dist = DescriptorDistance(d1, d2);

      if (matched_distance[i2] <= dist)
        continue;

      if (dist < best_distance) {
        best_distance2 = best_distance;
        best_distance = dist;
        best_idx2 = i2;
      } else if (dist < best_distance2) {
        best_distance2 = dist;
      }
    }

    if (best_distance <= TH_LOW) {

      if (best_distance < (float) best_distance2 * nearest_neighbour_ratio_) {
        if (matches21[best_idx2] >= 0) {
          out_matches_12[matches21[best_idx2]] = -1;
          nmatches--;
        }
        out_matches_12[i1] = best_idx2;
        matches21[best_idx2] = i1;
        matched_distance[best_idx2] = best_distance;
        nmatches++;

        if (check_orientation_) {
          float rot = features1.undistorted_keypoints[i1].angle - features2.undistorted_keypoints[best_idx2].angle;
          if (rot < 0.0)
            rot += 360.0f;
          int bin = std::round(rot * factor);
          if (bin == HISTO_LENGTH)
            bin = 0;
          assert(bin >= 0 && bin < HISTO_LENGTH);
          rotation_hostogram[bin].push_back(i1);
        }
      }
    }

  }

  if (check_orientation_) {
    int ind1 = -1;
    int ind2 = -1;
    int ind3 = -1;

    ComputeThreeMaxima(rotation_hostogram, HISTO_LENGTH, ind1, ind2, ind3);

    for (int i = 0; i < HISTO_LENGTH; i++) {
      if (i == ind1 || i == ind2 || i == ind3)
        continue;
      for (size_t j = 0, jend = rotation_hostogram[i].size(); j < jend; j++) {
        int idx1 = rotation_hostogram[i][j];
        if (out_matches_12[idx1] >= 0) {
          out_matches_12[idx1] = -1;
          nmatches--;
        }
      }
    }

  }

  return nmatches;
}

int SecondNearestNeighborMatcher::DescriptorDistance(const Eigen::Matrix<uint8_t,
                                                                         Eigen::Dynamic,
                                                                         Eigen::Dynamic,
                                                                         Eigen::RowMajor> & a,
                                                     const Eigen::Matrix<uint8_t,
                                                                         Eigen::Dynamic,
                                                                         Eigen::Dynamic,
                                                                         Eigen::RowMajor> & b) const {

  const int32_t * pa = reinterpret_cast<const int32_t *>(a.data());
  const int32_t * pb = reinterpret_cast<const int32_t *>(b.data());

  int dist = 0;

  for (int i = 0; i < a.cols() >> 2; ++i, ++pa, ++pb) {
    unsigned int v = *pa ^*pb;
    v -= ((v >> 1) & 0x55555555);
    v = (v & 0x33333333) + ((v >> 2) & 0x33333333);
    dist += (((v + (v >> 4)) & 0xF0F0F0F) * 0x1010101) >> 24;
  }

  return dist;
}

void SecondNearestNeighborMatcher::ComputeThreeMaxima(std::vector<int> * histo,
                                                      const int L,
                                                      int & ind1,
                                                      int & ind2,
                                                      int & ind3) const {
  int max1 = 0;
  int max2 = 0;
  int max3 = 0;

  for (int i = 0; i < L; i++) {
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

  if (max2 < 0.1f * (float) max1) {
    ind2 = -1;
    ind3 = -1;
  } else if (max3 < 0.1f * (float) max1) {
    ind3 = -1;
  }
}

}
}