//
// Created by vahagn on 02/02/21.
//

#ifndef ORB_SLAM3_WINDOW_MATCHER_H
#define ORB_SLAM3_WINDOW_MATCHER_H

// == orb-slam3 ===
#include <features/imatcher.h>

namespace orb_slam3 {
namespace features {

class SNNMatcher : public IMatcher {
 public:
  SNNMatcher(const size_t window_size,
             const precision_t nearest_neighbour_ratio,
             const bool check_orientation);

  void Match(const features::Features &features_to,
             const features::Features &features_from,
             std::vector<features::Match> &out_matches) const override;
  // returns the number of filtered matches
  static int FilterByOrientation(std::vector<int> &inout_matches_12,
                                 const Features &features1,
                                 const Features &features2);

 private:

  static void ComputeThreeMaxima(std::vector<int> *histo, int &ind1, int &ind2, int &ind3);

  int Match(const features::Features &features1,
            const features::Features &features2,
            std::vector<int> &out_matches_12) const;

  void Match(const DescriptorType &d1,
             const features::DescriptorSet &descriptors2,
             const std::vector<size_t> &allowed_inidces,
             int &out_idx2,
             unsigned &dist) const;

  static inline void ComputeRotationHistogram(std::vector<int> *rotation_histogram,
                                              const std::vector<int> &inout_matches_12,
                                              const Features &features1,
                                              const Features &features2);
 public:
  static const unsigned TH_LOW;
  static const int TH_HIGH;
  static const int HISTO_LENGTH;

 private:
  const size_t window_size_;
  const precision_t nearest_neighbour_ratio_;
  const bool check_orientation_;

};

}
}

#endif //ORB_SLAM3_WINDOW_MATCHER_H
