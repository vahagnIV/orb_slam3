//
// Created by vahagn on 25/03/2021.
//

#ifndef ORB_SLAM3_ORB_SLAM3_INCLUDE_FEATURES_MATCHING_VALIDATORS_ORIENTATION_VALIDATOR_H_
#define ORB_SLAM3_ORB_SLAM3_INCLUDE_FEATURES_MATCHING_VALIDATORS_ORIENTATION_VALIDATOR_H_

#include "../features.h"

namespace orb_slam3 {
namespace features {
namespace matching {


class OrientationValidator {
 public:
  OrientationValidator(const std::vector<KeyPoint> &kp_to, const std::vector<KeyPoint> &kp_from);
  int Validate(std::unordered_map<std::size_t, std::size_t> &inout_matches_12);
 private:
  static void ComputeThreeMaxima(std::vector<int> *histo, int &ind1, int &ind2, int &ind3);
  inline void ComputeRotationHistogram(std::vector<int> *rotation_histogram,
                                              const std::unordered_map<std::size_t, std::size_t> &inout_matches_12);
 private:
  const std::vector<KeyPoint> *kp_to_;
  const std::vector<KeyPoint> *kp_from_;
  static const int HISTO_LENGTH;

};


}
}
}
#endif //ORB_SLAM3_ORB_SLAM3_INCLUDE_FEATURES_MATCHING_VALIDATORS_ORIENTATION_VALIDATOR_H_
