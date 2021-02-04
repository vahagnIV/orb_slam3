//
// Created by vahagn on 01/02/21.
//

#ifndef ORB_SLAM3_FEATURES_H
#define ORB_SLAM3_FEATURES_H

#include <memory>

#include <Eigen/Eigen>

#include <typedefs.h>
#include <constants.h>
#include <camera/monocular_camera.h>
#include "key_point.h"

namespace orb_slam3 {
namespace features {

typedef Eigen::Matrix<uint8_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> DescriptorSet;

class Features {
 public:
  DescriptorSet descriptors;
  std::vector<KeyPoint> keypoints;
  std::vector<KeyPoint> undistorted_keypoints;
  std::vector<size_t> grid[constants::FRAME_GRID_COLS][constants::FRAME_GRID_ROWS];

  size_t Size() const { return keypoints.size(); }

  void ListFeaturesInArea(const precision_t & x,
                          const precision_t & y,
                          const size_t & window_size,
                          const precision_t & minLevel,
                          const precision_t & maxLevel,
                          const precision_t & min_X,
                          const precision_t & min_Y,
                          const precision_t & grid_element_width_inv,
                          const precision_t & grid_element_height_inv,
                          std::vector<size_t> & out_idx) const;

  void AssignFeaturesToGrid(const precision_t & min_X,
                            const precision_t & min_Y,
                            const precision_t & grid_element_width_inv_,
                            const precision_t & grid_element_height_inv_);


  void UndistortKeyPoints(const std::shared_ptr<camera::MonocularCamera> & camera) {
    undistorted_keypoints.resize(keypoints.size());
    for (size_t i = 0; i < undistorted_keypoints.size(); ++i) {
      camera->UndistortPoint(keypoints[i].pt, undistorted_keypoints[i].pt);
      undistorted_keypoints[i].angle = keypoints[i].angle;
      undistorted_keypoints[i].size = keypoints[i].size;
      undistorted_keypoints[i].level = keypoints[i].level;
    }
  }
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
