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
#include "../typedefs.h"
#include "../constants.h"
#include "../camera/monocular_camera.h"
#include "key_point.h"
#include "bow_container.h"

namespace orb_slam3 {
namespace features {

typedef Eigen::Matrix<uint8_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> DescriptorSet;

typedef Eigen::Matrix<uint8_t, 1, 32, Eigen::RowMajor> DescriptorType;

class Features {
 public:
  Features(const camera::MonocularCamera * camera);

  DescriptorSet descriptors;
  std::vector<KeyPoint> keypoints;
  std::vector<HomogenousPoint> undistorted_and_unprojected_keypoints;
  std::vector<TPoint2D> undistorted_keypoints;
  std::vector<size_t> grid[constants::FRAME_GRID_COLS][constants::FRAME_GRID_ROWS];
  BowContainer bow_container;

  void SetVocabulary(const BowVocabulary *vocabulary);
  void ComputeBow();
  size_t Size() const { return keypoints.size(); }
  void ListFeaturesInArea(const TPoint2D & point,
                          const size_t & window_size,
                          const int & minLevel,
                          const int & maxLevel,
                          std::vector<size_t> & out_idx) const;

  void AssignFeaturesToGrid();

  void UndistortKeyPoints();
 private:
  bool PosInGrid(const TPoint2D & kp,
                 size_t & posX,
                 size_t & posY) const;

 private:
  const camera::MonocularCamera * camera_;
  const precision_t grid_element_width_inv_;
  const precision_t grid_element_height_inv_;
};

}
}
#endif //ORB_SLAM3_FEATURES_H
