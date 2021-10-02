//
// Created by vahagn on 01/02/21.
//

// == orb-slam3 ===
#include "features.h"

namespace orb_slam3 {
namespace features {

Features::Features(precision_t width, precision_t height)
    : width_(width), height_(height),
      grid_element_width_inv_(static_cast<precision_t >(constants::FRAME_GRID_COLS)
                                  / width),
      grid_element_height_inv_(static_cast<precision_t >(constants::FRAME_GRID_ROWS)
                                   / height) {
}

Features::Features(std::istream &istream) {
  READ_FROM_STREAM(width_, istream);
  READ_FROM_STREAM(height_, istream);
  grid_element_width_inv_ = static_cast<precision_t >(constants::FRAME_GRID_COLS)
      / width_;
  grid_element_height_inv_ = static_cast<precision_t >(constants::FRAME_GRID_ROWS)
      / height_;
  Eigen::Index features_width;
  Eigen::Index features_height;
  READ_FROM_STREAM(features_width, istream);
  READ_FROM_STREAM(features_height, istream);
  descriptors.resize(features_height, features_width);

  istream.read((char *) descriptors.data(),
               features_width * features_height * sizeof(decltype(descriptors)::Scalar));
  for (Eigen::Index i = 0; i < features_height; ++i) {
    features::KeyPoint kp;
    READ_FROM_STREAM(kp.level, istream);
    READ_FROM_STREAM(kp.size, istream);
    READ_FROM_STREAM(kp.angle, istream);
    istream.read((char *) kp.pt.data(), kp.pt.size() * sizeof(decltype(kp.pt)::Scalar));
    keypoints.push_back(kp);
  }

  for (Eigen::Index i = 0; i < features_height; ++i) {
    TPoint2D ukp;
    istream.read((char *) ukp.data(), ukp.size() * sizeof(std::remove_reference<decltype(ukp)>::type::Scalar));
    undistorted_keypoints.emplace_back(ukp);
  }

  for (Eigen::Index i = 0; i < features_height; ++i) {
    TPoint3D uukp;
    istream.read((char *) uukp.data(), uukp.size() * sizeof(std::remove_reference<decltype(uukp)>::type::Scalar));
    undistorted_and_unprojected_keypoints.emplace_back(uukp);
  }
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

  const bool bCheckLevels = (minLevel > 0) || (maxLevel >= 0);

  for (size_t ix = min_cell_x; ix <= max_cell_x; ix++) {
    for (size_t iy = min_cell_y; iy <= max_cell_y; iy++) {
      const std::vector<size_t> & cell = grid[ix][iy];
      if (cell.empty())
        continue;

      for (size_t j = 0; j < cell.size(); j++) {
//      for (size_t j = 0; j < keypoints.size(); j++) {
        const KeyPoint & candidate_keypoint = keypoints[cell[j]].pt;
//        const KeyPoint & candidate_keypoint = undistorted_keypoints[j];
        const int & level = keypoints[cell[j]].level;
//        const int & level = keypoints[j].level;

        if (bCheckLevels) {
          if (level < minLevel)
            continue;
          if (maxLevel >= 0 && level > maxLevel)
            continue;
        }

        const precision_t distance_x = candidate_keypoint.pt.x() - point.x();
        const precision_t distance_y = candidate_keypoint.pt.y() - point.y();

        if (std::abs(distance_x) < window_size && std::abs(distance_y) < window_size)
          out_idx.push_back(cell[j]);
      }
    }
  }

}

void Features::AssignFeaturesToGrid() {
  for (size_t i = 0; i < keypoints.size(); i++) {
    size_t pos_X, pos_Y;
    if (PosInGrid(keypoints[i].pt, pos_X, pos_Y)) {
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
  posX = 0;
  posY = 0;
  return true;

  precision_t x = std::max(kp.x(), 0.);
  x = std::min(x, width_);
  precision_t y = std::max(kp.y(), 0.);
  y = std::min(y, height_);

  posX = std::min(constants::FRAME_GRID_COLS - 1,
                  static_cast<size_t>(x * grid_element_width_inv_));
  posY = std::min(constants::FRAME_GRID_ROWS - 1,
                  static_cast<size_t>(y * grid_element_height_inv_));

  return true;
}

std::ostream & operator<<(std::ostream & stream, const Features & features) {
  WRITE_TO_STREAM(features.width_, stream);
  WRITE_TO_STREAM(features.height_, stream);

  Eigen::Index features_width = features.descriptors.cols();
  Eigen::Index features_height = features.descriptors.rows();
  WRITE_TO_STREAM(features_width, stream);
  WRITE_TO_STREAM(features_height, stream);
  stream.write((char *) features.descriptors.data(),
               features_width * features_height * sizeof(decltype(features.descriptors)::Scalar));
  for (const auto &kp: features.keypoints) {
    WRITE_TO_STREAM(kp.level, stream);
    WRITE_TO_STREAM(kp.size, stream);
    WRITE_TO_STREAM(kp.angle, stream);
    stream.write((char *) kp.pt.data(), kp.pt.size() * sizeof(decltype(kp.pt)::Scalar));
  }

  for (const auto &ukp: features.undistorted_keypoints)
    stream.write((char *) ukp.data(), ukp.size() * sizeof(std::remove_reference<decltype(ukp)>::type::Scalar));

  for (const auto &ukp: features.undistorted_and_unprojected_keypoints)
    stream.write((char *) ukp.data(), ukp.size() * sizeof(std::remove_reference<decltype(ukp)>::type::Scalar));
  return stream;
}

void Features::SetWidth(precision_t width) {
  width_ = width;
  grid_element_width_inv_ = static_cast<precision_t >(constants::FRAME_GRID_COLS)
      / width_;
}

void Features::SetHeight(precision_t height) {
  height_ = height;
  grid_element_height_inv_ = static_cast<precision_t >(constants::FRAME_GRID_ROWS)
      / height_;
}

}
}