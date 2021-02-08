//
// Created by vahagn on 01/02/21.
//

#include <features/features.h>
#include <boost/shared_ptr.hpp>

namespace orb_slam3 {
namespace features {

void Features::ListFeaturesInArea(const precision_t & x,
                                  const precision_t & y,
                                  const size_t & window_size,
                                  const precision_t & minLevel,
                                  const precision_t & maxLevel,
                                  const precision_t & min_X,
                                  const precision_t & min_Y,
                                  const precision_t & grid_element_width_inv,
                                  const precision_t & grid_element_height_inv,
                                  std::vector<size_t> & out_idx) const {

  out_idx.reserve(keypoints.size());

  float factorX = window_size;
  float factorY = window_size;

  /*cout << "fX " << factorX << endl;
  cout << "fY " << factorY << endl;*/

  const size_t nMinCellX =
      std::max(0, (int) floor((x - min_X - factorX) * grid_element_width_inv));
  if (nMinCellX >= constants::FRAME_GRID_COLS) {
    return;
  }

  const int nMaxCellX =
      std::min((int) constants::FRAME_GRID_COLS - 1,
               (int) ceil((x - min_X + factorX) * grid_element_width_inv));
  if (nMaxCellX < 0) {
    return;
  }

  const size_t nMinCellY =
      std::max(0, (int) floor((y - min_Y - factorY) * grid_element_height_inv));
  if (nMinCellY >= constants::FRAME_GRID_ROWS) {
    return;
  }

  const int nMaxCellY =
      std::min((int) constants::FRAME_GRID_ROWS - 1,
               (int) ceil((y - min_Y + factorY) * grid_element_height_inv));
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

        const float distance_x = undistorted_keypoints[j][0] - x;
        const float distance_y = undistorted_keypoints[j][1] - y;

        if (fabs(distance_x) < factorX && fabs(distance_y) < factorY)
          out_idx.push_back(vCell[j]);
      }
    }
  }

}

void Features::AssignFeaturesToGrid(const precision_t & min_X,
                                    const precision_t & min_Y,
                                    const precision_t & grid_element_width_inv_,
                                    const precision_t & grid_element_height_inv_) {

  for (size_t i = 0; i < keypoints.size(); i++) {
    const TPoint2D kp = undistorted_keypoints[i];
    size_t pos_X, pos_Y;
    if (PosInGrid(kp, min_X, min_Y, grid_element_width_inv_, grid_element_height_inv_, pos_X, pos_Y)) {
      grid[pos_X][pos_Y].push_back(i);
    }
  }
}

bool Features::PosInGrid(const TPoint2D & kp,
                         const precision_t & min_X,
                         const precision_t & min_Y,
                         const precision_t & grid_element_width_inv_,
                         const precision_t & grid_element_height_inv_,
                         size_t & posX,
                         size_t & posY) const {
  if(kp[0] < min_X || kp[1] < min_Y)
    return false;

  posX = round((kp[0] - min_X) * grid_element_width_inv_);
  posY = round((kp[1] - min_Y) * grid_element_height_inv_);

  //Keypoint's coordinates are undistorted, which could cause to go out of the image
  if (posX >= constants::FRAME_GRID_COLS || posY >= constants::FRAME_GRID_ROWS)
    return false;

  return true;
}

}
}