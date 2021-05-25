//
// Created by vahagn on 01/02/21.
//

// == orb-slam3 ===
#include "features.h"

namespace orb_slam3 {
namespace features {

Features::Features(const camera::MonocularCamera * camera)
    : camera_(camera),
      grid_element_width_inv_(static_cast<precision_t >(constants::FRAME_GRID_COLS)
                                  / (camera->ImageBoundMaxX() - camera->ImageBoundMinX())),
      grid_element_height_inv_(static_cast<precision_t >(constants::FRAME_GRID_ROWS)
                                   / (camera->ImageBoundMaxY() - camera->ImageBoundMinY())) {
}

void Features::ListFeaturesInArea(const TPoint2D & point,
                                  const size_t & window_size,
                                  const int & minLevel,
                                  const int & maxLevel,
                                  std::vector<size_t> & out_idx) const {
  out_idx.clear();
  out_idx.reserve(keypoints.size());

  size_t min_cell_x, min_cell_y, max_cell_x, max_cell_y;

  PosInGrid(TPoint2D{point.x() - window_size, point.y() - window_size}, min_cell_x, min_cell_y);
  PosInGrid(TPoint2D{point.x() + window_size, point.y() + window_size}, max_cell_x, max_cell_y);
  if(min_cell_x > constants::FRAME_GRID_COLS)
    min_cell_x = 0;
  if(min_cell_y > constants::FRAME_GRID_ROWS)
    min_cell_y = 0;
  if(max_cell_x > constants::FRAME_GRID_COLS)
    max_cell_x = constants::FRAME_GRID_COLS -1;
  if(max_cell_y > constants::FRAME_GRID_ROWS)
    max_cell_y = constants::FRAME_GRID_ROWS -1;

  const bool bCheckLevels = (minLevel > 0) || (maxLevel >= 0);

  for (size_t ix = min_cell_x; ix <= max_cell_x; ix++) {
    for (size_t iy = min_cell_y; iy <= max_cell_y; iy++) {
      const std::vector<size_t> & cell = grid[ix][iy];
      if (cell.empty())
        continue;

      for (size_t j = 0; j < cell.size(); j++) {
//      for (size_t j = 0; j < keypoints.size(); j++) {
        const KeyPoint & candidate_keypoint = undistorted_keypoints[cell[j]];
//        const KeyPoint & candidate_keypoint = undistorted_keypoints[j];
        const int & level = keypoints[cell[j]].level;
//        const int & level = keypoints[j].level;

        if (bCheckLevels) {
          if (level < minLevel)
            continue;
          if (maxLevel >= 0)
            if (level > maxLevel)
              continue;
        }

        const precision_t distance_x = candidate_keypoint.pt.x() - point.x();
        const precision_t distance_y = candidate_keypoint.pt.y() - point.y();

        if (fabs(distance_x) < window_size && fabs(distance_y) < window_size)
          out_idx.push_back(cell[j]);
      }
    }
  }

}

void Features::AssignFeaturesToGrid() {
  for (size_t i = 0; i < undistorted_keypoints.size(); i++) {
    size_t pos_X, pos_Y;
    if (PosInGrid(undistorted_keypoints[i], pos_X, pos_Y)) {
      grid[pos_X][pos_Y].push_back(i);
    }
  }

//  for (int j = 0; j < constants::FRAME_GRID_ROWS; ++j) {
//    for (int i = 0; i < constants::FRAME_GRID_COLS; ++i) {
//      std::cout << grid[i][j].size() << " \t";
//    }
//    std::cout << std::endl;
//  }
}

bool Features::PosInGrid(const TPoint2D & kp,
                         size_t & posX,
                         size_t & posY) const {
  if(kp.x() < camera_->ImageBoundMinX() )
    return false;
  if(kp.x() > camera_->ImageBoundMaxX() )
    return false;

  if(kp.y() < camera_->ImageBoundMinY() )
    return false;
  if(kp.y() > camera_->ImageBoundMaxY() )
    return false;


  posX = (kp.x() - camera_->ImageBoundMinX()) * grid_element_width_inv_;
  posY = (kp.y() - camera_->ImageBoundMinY()) * grid_element_height_inv_;

  return true;
}

void Features::UndistortKeyPoints() {
  undistorted_and_unprojected_keypoints.resize(keypoints.size());
  undistorted_keypoints.resize(keypoints.size());
  for (size_t i = 0; i < undistorted_and_unprojected_keypoints.size(); ++i) {
    camera_->UndistortPoint(keypoints[i].pt, undistorted_keypoints[i]);
    camera_->UnprojectPoint(undistorted_keypoints[i], undistorted_and_unprojected_keypoints[i]);
  }
}

void Features::SetVocabulary(const BowVocabulary * vocabulary) {
  bow_container.vocabulary = vocabulary;
}

void Features::ComputeBow() {
  if (!bow_container.feature_vector.empty() && !bow_container.bow_vector.empty())
    return;
  std::vector<cv::Mat> current_descriptors;
  current_descriptors.reserve(descriptors.rows());
  for (int i = 0; i < descriptors.rows(); ++i) {
    current_descriptors.push_back(cv::Mat(1,
                                          descriptors.cols(),
                                          cv::DataType<decltype(descriptors)::Scalar>::type,
                                          (void *) descriptors.row(i).data()));
  }
  bow_container.ComputeBow(current_descriptors);
}

}
}