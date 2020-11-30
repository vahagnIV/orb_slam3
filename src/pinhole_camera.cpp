//
// Created by vahagn on 11/30/20.
//

#include "pinhole_camera.h"

namespace nvision {
void PinholeCamera::UndistortKeyPoints(std::vector<KeyPoint> &in_out_keypoints) {
  if (0 == distortion_coefficients_.at<float>(0)) return;

  // Fill matrix with points
  cv::Mat mat(in_out_keypoints.size(), 2, CV_32F);

  for (int i = 0; i < in_out_keypoints.size(); i++) {
    mat.at<float>(i, 0) = in_out_keypoints[i].pt.x;
    mat.at<float>(i, 1) = in_out_keypoints[i].pt.y;
  }

  // Undistort points
  mat = mat.reshape(2);
  // TODO: fix the last parameter
  cv::undistortPoints(mat, mat, camera_matrix_, distortion_coefficients_, cv::Mat(), camera_matrix_);
  mat = mat.reshape(1);

  for (int i = 0; i < in_out_keypoints.size(); i++) {
    in_out_keypoints[i].pt.x = mat.at<float>(i, 0);
    in_out_keypoints[i].pt.y = mat.at<float>(i, 1);
  }

}
}