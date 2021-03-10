//
// Created by vahagn on 02/02/21.
//

#ifndef ORB_SLAM3_WINDOW_MATCHER_H
#define ORB_SLAM3_WINDOW_MATCHER_H

// == orb-slam3 ===
#include <features/imatcher.h>

namespace orb_slam3 {
namespace features {

class SecondNearestNeighborMatcher : public IMatcher {
 public:
  SecondNearestNeighborMatcher(const size_t window_size,
                const precision_t nearest_neighbour_ratio,
                const bool check_orientation);

  void Match(const features::Features & features_to,
            const features::Features & features_from,
            std::vector<features::Match> & out_matches) const override;
 private:

  void ComputeThreeMaxima(std::vector<int> *histo, const int L, int & ind1, int & ind2, int & ind3) const;

  int Match(const features::Features & features1,
            const features::Features & features2,
            std::vector<int> & out_matches_12) const;

 private:
  static const int TH_LOW;
  static const int TH_HIGH;
  static const int HISTO_LENGTH;

  const size_t window_size_;
  const precision_t nearest_neighbour_ratio_;
  const bool check_orientation_;

};

}
}

#endif //ORB_SLAM3_WINDOW_MATCHER_H
