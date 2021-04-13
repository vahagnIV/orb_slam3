//
// Created by vahagn on 01/02/21.
//

#ifndef ORB_SLAM3_FEATURES_H
#define ORB_SLAM3_FEATURES_H

// === stl ===
#include <memory>

// === Eigen ===
#include <Eigen/Eigen>

// == orb-slam3 ===
#include <typedefs.h>
#include <constants.h>
#include <camera/monocular_camera.h>
#include "key_point.h"
#include <features/bow_container.h>

namespace orb_slam3 {
namespace features {

typedef Eigen::Matrix<uint8_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> DescriptorSet;

typedef Eigen::Matrix<uint8_t, 1, Eigen::Dynamic, Eigen::RowMajor> DescriptorType;

class Features {
 public:
  Features(size_t image_width, size_t image_height);

  DescriptorSet descriptors;
  std::vector<KeyPoint> keypoints;
  std::vector<HomogenousPoint> undistorted_keypoints;
  std::vector<size_t> grid[constants::FRAME_GRID_COLS][constants::FRAME_GRID_ROWS];
  BowContainer bow_container;

  void SetVocabulary(BowVocabulary *vocabulary);
  void ComputeBow();
  size_t Size() const { return keypoints.size(); }
  void ListFeaturesInArea(const precision_t & x,
                          const precision_t & y,
                          const size_t & window_size,
                          const precision_t & minLevel,
                          const precision_t & maxLevel,
                          std::vector<size_t> & out_idx) const;

  void AssignFeaturesToGrid();

  void UndistortKeyPoints(const std::shared_ptr<camera::MonocularCamera> & camera);
 private:
  bool PosInGrid(const TPoint2D & kp,
                 const precision_t & min_X,
                 const precision_t & min_Y,
                 size_t & posX,
                 size_t & posY) const;

 private:
  const size_t image_width_;
  const size_t image_height_;
  const precision_t grid_element_width_inv_;
  const precision_t grid_element_height_inv_;
};

}
}
#endif //ORB_SLAM3_FEATURES_H
