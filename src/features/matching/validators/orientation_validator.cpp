//
// Created by vahagn on 25/03/2021.
//

#include "orientation_validator.h"
namespace orb_slam3 {
namespace features {
namespace matching {

const int OrientationValidator::HISTO_LENGTH = 30;
OrientationValidator::OrientationValidator(const std::vector<KeyPoint> & kp_to, const std::vector<KeyPoint> & kp_from) : kp_to_(
    &kp_to), kp_from_(&kp_from) {

}

int OrientationValidator::Validate(std::unordered_map<std::size_t, std::size_t> & inout_matches_12) {
  std::vector<int> rotation_histogram[HISTO_LENGTH];
  ComputeRotationHistogram(rotation_histogram, inout_matches_12);
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
      if (inout_matches_12.find(idx1) != inout_matches_12.end()) {
        inout_matches_12.erase(idx1);
        number_of_discarded_matches++;
      }
    }
  }
  return number_of_discarded_matches;
}

void OrientationValidator::ComputeThreeMaxima(std::vector<int> * histo,
                                              int & ind1,
                                              int & ind2,
                                              int & ind3) {
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

void OrientationValidator::ComputeRotationHistogram(std::vector<int> * rotation_histogram,
                                                    const std::unordered_map<std::size_t,
                                                                             std::size_t> & inout_matches_12) {
  const precision_t factor = 1.0f / HISTO_LENGTH;
  for (int i = 0; i < HISTO_LENGTH; i++)
    rotation_histogram[i].reserve(500);

  for (auto & match: inout_matches_12) {

    float rot = kp_to_->at(match.first).angle - kp_from_->at(match.second).angle;
    if (rot < 0.0)
      rot += 360.0f;
    int bin = std::round(rot * factor);
    if (bin == HISTO_LENGTH)
      bin = 0;
    assert(bin >= 0 && bin < HISTO_LENGTH);
    rotation_histogram[bin].push_back(match.first);
  }

}

}
}
}