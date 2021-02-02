//
// Created by vahagn on 02/02/21.
//

#ifndef ORB_SLAM3_WINDOW_MATCHER_H
#define ORB_SLAM3_WINDOW_MATCHER_H

#include <features/imatcher.h>

namespace orb_slam3 {
namespace features {

class SecondNearestNeighborMatcher : public IMatcher {
 public:
  SecondNearestNeighborMatcher(const size_t window_size,
                const precision_t nearest_neighbour_ratio,
                const bool check_orientation,
                const precision_t min_X,
                const precision_t min_Y,
                const precision_t grid_element_width_inv,
                const precision_t grid_element_height_inv);

  int Match(const features::Features & features1,
            const features::Features & features2,
            std::vector<int> & out_matches_12) override;
 private:
  int DescriptorDistance(const Eigen::Matrix<uint8_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> & a,
                         const Eigen::Matrix<uint8_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> & b) const;
  void ComputeThreeMaxima(std::vector<int> *histo, const int L, int & ind1, int & ind2, int & ind3) const;

 private:
  static const int TH_LOW;
  static const int TH_HIGH;
  static const int HISTO_LENGTH;

  const size_t window_size_;
  const precision_t nearest_neighbour_ratio_;
  const bool check_orientation_;
  const precision_t min_X_;
  const precision_t min_Y_;
  const precision_t grid_element_width_inv_;
  const precision_t grid_element_height_inv_;

};

}
}

#endif //ORB_SLAM3_WINDOW_MATCHER_H
