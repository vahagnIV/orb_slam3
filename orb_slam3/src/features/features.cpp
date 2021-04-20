//
// Created by vahagn on 01/02/21.
//

// == orb-slam3 ===
#include <features/features.h>

namespace orb_slam3 {
namespace features {

Features::Features(size_t image_width, size_t image_height)
    : image_width_(image_width), image_height_(image_height), grid_element_width_inv_(
    static_cast<precision_t >(constants::FRAME_GRID_COLS) / image_width), grid_element_height_inv_(
    static_cast<precision_t >(constants::FRAME_GRID_ROWS) / image_height) {

}

void Features::ListFeaturesInArea(const precision_t & x,
                                  const precision_t & y,
                                  const size_t & window_size,
                                  const precision_t & minLevel,
                                  const precision_t & maxLevel,
                                  std::vector<size_t> & out_idx) const {
  out_idx.clear();

  const precision_t min_X = 0;
  const precision_t min_Y = 0;

  out_idx.reserve(keypoints.size());

  float factorX = window_size;
  float factorY = window_size;

  /*cout << "fX " << factorX << endl;
  cout << "fY " << factorY << endl;*/

  const size_t nMinCellX =
      std::max(0, (int) floor((x - min_X - factorX) * grid_element_width_inv_));
  if (nMinCellX >= constants::FRAME_GRID_COLS) {
    return;
  }

  const int nMaxCellX =
      std::min((int) constants::FRAME_GRID_COLS - 1,
               (int) ceil((x - min_X + factorX) * grid_element_width_inv_));
  if (nMaxCellX < 0) {
    return;
  }

  const size_t nMinCellY =
      std::max(0, (int) floor((y - min_Y - factorY) * grid_element_height_inv_));
  if (nMinCellY >= constants::FRAME_GRID_ROWS) {
    return;
  }

  const int nMaxCellY =
      std::min((int) constants::FRAME_GRID_ROWS - 1,
               (int) ceil((y - min_Y + factorY) * grid_element_height_inv_));
  if (nMaxCellY < 0) {
    return;
  }

  const bool bCheckLevels = (minLevel > 0) || (maxLevel >= 0);

  for (int ix = nMinCellX; ix <= nMaxCellX; ix++) {
    for (int iy = nMinCellY; iy <= nMaxCellY; iy++) {
      const std::vector<size_t> & vCell = grid[ix][iy];
      if (vCell.empty())
        continue;

      for (size_t j = 0, jend = vCell.size(); j < jend; j++) {
        const KeyPoint & kpUn = keypoints[vCell[j]];

        if (bCheckLevels) {
          if (kpUn.level < minLevel)
            continue;
          if (maxLevel >= 0)
            if (kpUn.level > maxLevel)
              continue;
        }

        const float distance_x = kpUn.pt[0] - x;
        const float distance_y = kpUn.pt[1] - y;

        if (fabs(distance_x) < factorX && fabs(distance_y) < factorY)
          out_idx.push_back(vCell[j]);
      }
    }
  }

}

void Features::AssignFeaturesToGrid() {
  const precision_t min_X = 0;
  const precision_t min_Y = 0;

  for (size_t i = 0; i < keypoints.size(); i++) {
    const TPoint2D & kp = keypoints[i].pt;
    size_t pos_X, pos_Y;
    if (PosInGrid(kp, min_X, min_Y, pos_X, pos_Y)) {
      grid[pos_X][pos_Y].push_back(i);
    }
  }
}

bool Features::PosInGrid(const TPoint2D & kp,
                         const precision_t & min_X,
                         const precision_t & min_Y,
                         size_t & posX,
                         size_t & posY) const {
  if (kp[0] < min_X || kp[1] < min_Y)
    return false;

  posX = round((kp[0] - min_X) * grid_element_width_inv_);
  posY = round((kp[1] - min_Y) * grid_element_height_inv_);

  //Keypoint's coordinates are undistorted, which could cause to go out of the image
  if (posX >= constants::FRAME_GRID_COLS || posY >= constants::FRAME_GRID_ROWS)
    return false;

  return true;
}

void Features::UndistortKeyPoints(const shared_ptr<camera::MonocularCamera> & camera) {
  undistorted_and_unprojected_keypoints.resize(keypoints.size());
  unprojected_keypoints.resize(keypoints.size());
  for (size_t i = 0; i < undistorted_and_unprojected_keypoints.size(); ++i) {
    camera->UnprojectPoint(keypoints[i].pt, unprojected_keypoints[i]);
    camera->GetDistortionModel()->UnDistortPoint(unprojected_keypoints[i], undistorted_and_unprojected_keypoints[i]);
  }
}

void Features::SetVocabulary(BowVocabulary * vocabulary) {
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