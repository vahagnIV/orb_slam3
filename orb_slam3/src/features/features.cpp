//
// Created by vahagn on 01/02/21.
//

#include <features/features.h>

namespace orb_slam3 {
namespace features {

void Features::ListFeaturesInArea(const precision_t & x,
                                  const precision_t & y,
                                  const size_t & window_size,
                                  const precision_t & level1,
                                  const precision_t & level2,
                                  std::vector<size_t> & out_idx) {


/*  out_idx.reserve(keypoints.size());

  float factorX = window_size;
  float factorY = window_size;*/

  /*cout << "fX " << factorX << endl;
  cout << "fY " << factorY << endl;*/

  /* const int nMinCellX =
       max(0, (int) floor((x - GetCamera()->ImageBoundMinX() - factorX) * GetCamera()->GridElementWidthInv()));
   if (nMinCellX >= FRAME_GRID_COLS) {
     return ;
   }

   const int nMaxCellX =
       min((int) FRAME_GRID_COLS - 1,
           (int) ceil((x - GetCamera()->ImageBoundMinX() + factorX) * GetCamera()->GridElementWidthInv()));
   if (nMaxCellX < 0) {
     return ;
   }

   const int nMinCellY =
       max(0, (int) floor((y - GetCamera()->ImageBoundMinY() - factorY) * GetCamera()->GridElementHeightInv()));
   if (nMinCellY >= FRAME_GRID_ROWS) {
     return ;
   }

   const int nMaxCellY =
       min((int) FRAME_GRID_ROWS - 1,
           (int) ceil((y - GetCamera()->ImageBoundMinY() + factorY) * GetCamera()->GridElementHeightInv()));
   if (nMaxCellY < 0) {
     return ;
   }

   const bool bCheckLevels = (minLevel > 0) || (maxLevel >= 0);

   for (int ix = nMinCellX; ix <= nMaxCellX; ix++) {
     for (int iy = nMinCellY; iy <= nMaxCellY; iy++) {
       const vector<size_t> vCell = (!bRight) ? mGrid[ix][iy] : mGridRight[ix][iy];
       if (vCell.empty())
         continue;

       for (size_t j = 0, jend = vCell.size(); j < jend; j++) {
         const cv::KeyPoint &kpUn = (Nleft == -1) ? undistorted_key_poinst_[vCell[j]]
                                                  : (!bRight) ? key_points_[vCell[j]]
                                                              : right_key_points_[vCell[j]];
         if (bCheckLevels) {
           if (kpUn.octave < minLevel)
             continue;
           if (maxLevel >= 0)
             if (kpUn.octave > maxLevel)
               continue;
         }

         const float distx = kpUn.pt.x - x;
         const float disty = kpUn.pt.y - y;

         if (fabs(distx) < factorX && fabs(disty) < factorY)
           out_idx.push_back(vCell[j]);
       }
     }
   }*/

}

void Features::AssignFeaturesToGrid(const precision_t & min_X,
                                    const precision_t & min_Y,
                                    const precision_t & grid_element_width_inv_,
                                    const precision_t & grid_element_height_inv_) {

  for (size_t i = 0; i < keypoints.size(); i++) {
    const KeyPoint & kp = undistorted_keypoints[i];
    size_t pos_X, pos_Y;
    if (PosInGrid(kp, min_X, min_Y, grid_element_width_inv_, grid_element_height_inv_, pos_X, pos_Y)) {
      grid[pos_X][pos_Y].push_back(i);
    }
  }
}

bool Features::PosInGrid(const KeyPoint & kp,
                         const precision_t & min_X,
                         const precision_t & min_Y,
                         const precision_t & grid_element_width_inv_,
                         const precision_t & grid_element_height_inv_,
                         size_t & posX,
                         size_t & posY) const {
  posX = round((kp.X() - min_X) * grid_element_width_inv_);
  posY = round((kp.Y() - min_Y) * grid_element_height_inv_);

  //Keypoint's coordinates are undistorted, which could cause to go out of the image
  if (posX < 0 || posX >= constants::FRAME_GRID_COLS || posY < 0 || posY >= constants::FRAME_GRID_ROWS)
    return false;

  return true;
}

}
}