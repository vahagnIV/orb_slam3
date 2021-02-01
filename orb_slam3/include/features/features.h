//
// Created by vahagn on 01/02/21.
//

#ifndef ORB_SLAM3_FEATURES_H
#define ORB_SLAM3_FEATURES_H

#include <Eigen/Eigen>
#include <typedefs.h>
#include <constants.h>
#include "key_point.h"

namespace orb_slam3 {
namespace features {

typedef Eigen::Matrix<precision_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> DescriptorSet;

class Features {
 public:
  DescriptorSet descriptors;
  std::vector<KeyPoint> keypoints;
  std::vector<KeyPoint> undistorted_keypoints;
  std::vector<size_t> grid[constants::FRAME_GRID_COLS][constants::FRAME_GRID_ROWS];
  void ListFeaturesInArea(const precision_t & x,
                          const precision_t & y,
                          const size_t & window_size,
                          const precision_t & level1,
                          const precision_t & level2,
                          std::vector<size_t> & out_idx);
  void AssignFeaturesToGrid(const precision_t & min_X,
                            const precision_t & min_Y,
                            const precision_t & grid_element_width_inv_,
                            const precision_t & grid_element_height_inv_);
 private:
  bool PosInGrid(const KeyPoint & kp,
                 const precision_t & min_X,
                 const precision_t & min_Y,
                 const precision_t & grid_element_width_inv_,
                 const precision_t & grid_element_height_inv_,
                 size_t & posX,
                 size_t & posY) const;

};

}
}
#endif //ORB_SLAM3_FEATURES_H
